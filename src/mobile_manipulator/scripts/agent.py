#!/usr/bin/env python3

from __future__ import annotations
import os
import rospy
import json
import yaml
import math
from collections import defaultdict
import mobile_manipulator.genai_rest as genai
from pydantic import BaseModel, Field, ValidationError

from mobile_manipulator.master_control import MasterControl

# ==========================================
# 0. THE MAP READER (Reads your YAML)
# ==========================================
def load_semantic_map(yaml_path: str) -> tuple[dict, dict, dict]:
    """
    Reads the YAML file and returns:
      1. semantic_dict:     {name: {x, y, yaw_rad, room}} for coordinate lookup
      2. room_groups:       {room: [name1, name2, ...]} for LLM context
      3. object_inventory:  {object_class: location_name} built from 'objects' fields
    """
    try:
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)

        semantic_dict = {}
        room_groups = defaultdict(list)
        object_inventory = {}

        for region in data.get('regions', []):
            name = region['name'].lower()
            room = region.get('room', 'unknown').lower()

            semantic_dict[name] = {
                'x': region['x'],
                'y': region['y'],
                'yaw_rad': region['yaw'],
                'room': room
            }
            room_groups[room].append(name)

            for obj in region.get('objects', []):
                object_inventory[obj.lower()] = name

        rospy.loginfo(
            f"Loaded semantic map: {len(semantic_dict)} regions, "
            f"{len(room_groups)} rooms, {len(object_inventory)} objects."
        )
        return semantic_dict, dict(room_groups), object_inventory
    except Exception as e:
        rospy.logerr(f"Failed to load semantic map: {e}")
        return {}, {}, {}


# ==========================================
# 1. THE BOUNCERS (Pydantic Models)
# ==========================================
class DriveCommand(BaseModel):
    location_name: str = Field(..., description="The semantic name of the destination")

class PickCommand(BaseModel):
    target_class: str = Field(..., min_length=2, description="Name of the object to pick")


# ==========================================
# 2. THE MENU (JSON Schema for Gemini)
# ==========================================
robot_tools_json = [
    {
        "function_declarations": [
            {
                "name": "drive_to_location",
                "description": "Drives the robot to a specific semantic location.",
                "parameters": {
                    "type_": "OBJECT",
                    "properties": {
                        "location_name": {"type_": "STRING"}
                    },
                    "required": ["location_name"]
                }
            },
            {
                "name": "pick_object",
                "description": "Commands the UR5 robotic arm to pick up a specific object. You MUST be near the object before calling this.",
                "parameters": {
                    "type_": "OBJECT",
                    "properties": {
                        "target_class": {"type_": "STRING"}
                    },
                    "required": ["target_class"]
                }
            },
            {
                "name": "list_locations",
                "description": (
                    "Returns all navigable locations the robot knows about, "
                    "grouped by room. Use this when you need to recall which "
                    "locations are available or which room something is in."
                ),
                "parameters": {
                    "type_": "OBJECT",
                    "properties": {},
                    "required": []
                }
            }
        ]
    }
]


# ==========================================
# 3. THE LLM AGENT
# ==========================================
class GeminiHardenedAgent:
    def __init__(self):
        genai.configure(api_key=os.environ["GEMINI_API_KEY"])
        self.mc = MasterControl()

        # Load YAML Semantic Map — also populates object_inventory from 'objects' fields.
        # To add a new grabbable object: add it to the region's 'objects' list in semantic_map.yaml.
        map_path = os.path.expanduser("~/learning_ws/src/mobile_manipulator/config/semantic_map.yaml")
        self.semantic_map, self.room_groups, self.object_inventory = load_semantic_map(map_path)

        # 3. Build the system prompt from map data (auto-generated, never hardcoded)
        system_prompt = self._build_system_prompt()

        self.model = genai.GenerativeModel(
            model_name='gemini-2.5-flash',
            tools=robot_tools_json,
            system_instruction=system_prompt
        )
        self.chat = self.model.start_chat()
        rospy.loginfo("Hardened Gemini Agent Booted with YAML Map and Object Inventory!")

    # ------------------------------------------
    # Prompt Builder — auto-generates from YAML
    # ------------------------------------------
    def _build_system_prompt(self) -> str:
        """
        Generates the system prompt from the live semantic map and object inventory.
        The LLM sees semantic names and room groupings, NEVER coordinates.
        """
        # Build room layout section
        room_lines = []
        for room, locations in sorted(self.room_groups.items()):
            loc_str = ", ".join(sorted(locations))
            room_lines.append(f"  - {room}: {loc_str}")
        rooms_section = "\n".join(room_lines)

        # Build object inventory section
        if self.object_inventory:
            inv_lines = [f"  - {obj} → {loc}" for obj, loc in sorted(self.object_inventory.items())]
            inventory_section = "\n".join(inv_lines)
        else:
            inventory_section = "  (No objects discovered yet.)"

        return (
            "You are an autonomous mobile manipulator robot (Husky base + UR5 arm) in a house.\n"
            "\n"
            "## Rooms & Navigable Locations:\n"
            f"{rooms_section}\n"
            "\n"
            "## Known Object Locations:\n"
            f"{inventory_section}\n"
            "\n"
            "## Rules:\n"
            "1. To pick up an object, you MUST call drive_to_location to go to its location FIRST, "
            "then call pick_object once you arrive.\n"
            "2. If you are unsure where an object is, use list_locations to review available locations.\n"
            "3. NEVER invent location names that are not in the list above.\n"
            "4. If asked about an object not in your inventory, say you don't know where it is "
            "and suggest the user tell you which room to search.\n"
        )

    # ------------------------------------------
    # Dynamic Inventory Update
    # ------------------------------------------
    def update_inventory(self, object_name: str, location_name: str):
        """
        Called after a successful YOLO detection to keep the object inventory
        in sync with the real world. The updated mapping will be available
        to the LLM in subsequent queries via the list_locations tool
        or when the chat session is rebuilt.
        """
        object_name = object_name.lower()
        location_name = location_name.lower()

        old_location = self.object_inventory.get(object_name)
        self.object_inventory[object_name] = location_name

        if old_location and old_location != location_name:
            rospy.loginfo(f"[INVENTORY] Updated: '{object_name}' moved from '{old_location}' → '{location_name}'")
        else:
            rospy.loginfo(f"[INVENTORY] Registered: '{object_name}' at '{location_name}'")

    # ------------------------------------------
    # Active Object Search
    # ------------------------------------------
    def _scan_for_object(self, target_class: str) -> tuple:
        """Sweeps the UR5 arm through viewing poses to find target_class with YOLO."""
        return self.mc.scan_with_arm(target_class)

    # ------------------------------------------
    # Tool Executor
    # ------------------------------------------
    def execute_physical_tool(self, tool_name: str, args_dict: dict) -> str:
        # --- TOOL: DRIVE ---
        if tool_name == "drive_to_location":
            try:
                valid_cmd = DriveCommand(**args_dict)
                target_name = valid_cmd.location_name.lower()

                # Check the YAML map data
                if target_name not in self.semantic_map:
                    available = ", ".join(sorted(self.semantic_map.keys()))
                    return f"FAILURE: '{target_name}' is not on the map. I can only go to: {available}"
                
                coords = self.semantic_map[target_name]
                x = coords['x']
                y = coords['y']
                
                # Convert the YAML radians to Degrees for our MasterControl script!
                yaw_deg = math.degrees(coords['yaw_rad'])
                
                success = self.mc.move_base_to(x, y, yaw_deg)
                
                if success:
                    return f"SUCCESS: Base arrived at the {target_name}."
                return f"FAILURE: Navigation failed to reach the {target_name}."
                
            except ValidationError as e:
                return f"SYSTEM ERROR: Invalid parameters. {e.errors()[0]['msg']}"

        # --- TOOL: PICK ---
        elif tool_name == "pick_object":
            try:
                valid_cmd = PickCommand(**args_dict)
                rospy.loginfo(f"Valid pick command received for: {valid_cmd.target_class}")
                
                u, v = self._scan_for_object(target_class=valid_cmd.target_class)
                if u is not None and v is not None:
                    pose = self.mc.get_3d_coordinates(u, v)
                    if pose and self.mc.execute_pick(pose):
                        # On success, update the inventory to reflect that
                        # this object was found at the robot's current location
                        current_location = self._get_current_location()
                        if current_location:
                            self.update_inventory(valid_cmd.target_class, current_location)
                        return f"SUCCESS: Picked up the {valid_cmd.target_class}."
                return f"FAILURE: Could not find '{valid_cmd.target_class}'. Are you in the right room?"
                
            except ValidationError as e:
                return f"SYSTEM ERROR: Invalid parameters. {e.errors()[0]['msg']}"

        # --- TOOL: LIST LOCATIONS ---
        elif tool_name == "list_locations":
            result_lines = ["Available locations by room:"]
            for room, locations in sorted(self.room_groups.items()):
                result_lines.append(f"  {room}: {', '.join(sorted(locations))}")
            
            result_lines.append("\nKnown object locations:")
            if self.object_inventory:
                for obj, loc in sorted(self.object_inventory.items()):
                    result_lines.append(f"  {obj} → {loc}")
            else:
                result_lines.append("  (No objects discovered yet.)")
            
            return "\n".join(result_lines)

        return f"SYSTEM ERROR: Tool '{tool_name}' does not exist."

    def _get_current_location(self) -> str:
        """
        Determines which semantic location the robot is currently closest to.
        Uses a simple Euclidean distance check against all known map regions.
        Returns None if no location is within a reasonable threshold (2.0m).
        """
        try:
            # Get the robot's current pose from move_base (or tf if available)
            robot_x = self.mc.current_x
            robot_y = self.mc.current_y
        except AttributeError:
            rospy.logwarn("[INVENTORY] Cannot determine current location — MasterControl has no pose tracking.")
            return None
        
        closest_name = None
        closest_dist = float('inf')
        
        for name, data in self.semantic_map.items():
            dist = math.sqrt((robot_x - data['x'])**2 + (robot_y - data['y'])**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_name = name
        
        # Only return if we're reasonably close to a known location
        if closest_dist < 2.0:
            return closest_name
        return None

    def give_command(self, user_prompt: str):
        rospy.loginfo(f"\n[HUMAN]: {user_prompt}")
        
        response = self.chat.send_message(user_prompt)
        
        while response.parts and response.parts[0].function_call:
            fc = response.parts[0].function_call
            tool_name = fc.name
            args_dict = getattr(fc, "args", {})
            
            rospy.loginfo(f"[GEMINI REASONING]: Calling {tool_name} with {args_dict}")
            
            tool_result = self.execute_physical_tool(tool_name, args_dict)
            
            response = self.chat.send_message(
                genai.types.Part.from_function_response(
                    name=tool_name,
                    response={"result": tool_result}
                )
            )

        rospy.loginfo(f"[ROBOT]: {response.text}\n")
        return response.text


if __name__ == '__main__':
    try:
        rospy.init_node('gemini_robot_brain', anonymous=True)
        agent = GeminiHardenedAgent()
        agent.give_command("Can you grab my cup please?")
    except rospy.ROSInterruptException:
        pass