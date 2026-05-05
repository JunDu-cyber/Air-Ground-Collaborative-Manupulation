#!/usr/bin/env python3

from __future__ import annotations
import json
import math
import os
from collections import defaultdict

import rospy
import yaml
from openai import OpenAI
from pydantic import BaseModel, Field, ValidationError

from mobile_manipulator.master_control import MasterControl


# ── Semantic map loader ────────────────────────────────────────────────────────

def load_semantic_map(yaml_path: str) -> tuple[dict, dict, dict]:
    """
    Returns:
      semantic_dict    — {name: {x, y, yaw_rad, room}}
      room_groups      — {room: [location_name, ...]}
      object_inventory — {coco_class: location_name}  (from 'objects' fields in YAML)
    """
    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f)

        semantic_dict    = {}
        room_groups      = defaultdict(list)
        object_inventory = {}

        for region in data.get('regions', []):
            name = region['name'].lower()
            room = region.get('room', 'unknown').lower()
            semantic_dict[name] = {
                'x': region['x'], 'y': region['y'],
                'yaw_rad': region['yaw'], 'room': room,
            }
            room_groups[room].append(name)
            for obj in region.get('objects', []):
                object_inventory[obj.lower()] = name

        rospy.loginfo(
            f"[MAP] {len(semantic_dict)} regions · "
            f"{len(room_groups)} rooms · {len(object_inventory)} objects"
            f" · Object inventory: {list(object_inventory.keys())}"
        )
        return semantic_dict, dict(room_groups), object_inventory
    except Exception as e:
        rospy.logerr(f"[MAP] Failed to load semantic map: {e}")
        return {}, {}, {}


# ── Pydantic validators ────────────────────────────────────────────────────────

class DriveCommand(BaseModel):
    location_name: str = Field(..., description="Semantic name of the destination")

class PickCommand(BaseModel):
    target_class: str = Field(..., min_length=2, description="COCO-80 class name of the object")


# ── OpenAI-format tool definitions ────────────────────────────────────────────
# DeepSeek's API is OpenAI-compatible — same schema works unchanged.

ROBOT_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "drive_to_location",
            "description": "Drives the robot base to a named semantic location.",
            "parameters": {
                "type": "object",
                "properties": {
                    "location_name": {
                        "type": "string",
                        "description": "The semantic location name (e.g. 'kitchen_table')."
                    }
                },
                "required": ["location_name"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "pick_object",
            "description": (
                "Commands the UR5 arm to pick up an object. "
                "You MUST call drive_to_location first to be near the object."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "target_class": {
                        "type": "string",
                        "description": "COCO-80 class name of the object to pick (e.g. 'cup')."
                    }
                },
                "required": ["target_class"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "place_object",
            "description": (
                "Releases the held object at a named semantic location. "
                "You MUST call drive_to_location first to be near the destination, "
                "then call place_object. Use for putting, dropping, or delivering objects."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "location_name": {
                        "type": "string",
                        "description": "The semantic location name where to place the object (e.g. 'trash_kitchen', 'coffee_table')."
                    },
                    "surface_height": {
                        "type": "number",
                        "description": (
                            "Estimated height (metres) of the target surface in the map frame. "
                            "Defaults to 0.75 (typical table). Use ~0.0 for floor-level bins."
                        )
                    }
                },
                "required": ["location_name"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "list_locations",
            "description": (
                "Returns all navigable locations grouped by room, "
                "and the known object→location inventory. "
                "Use this when unsure which locations exist."
            ),
            "parameters": {"type": "object", "properties": {}, "required": []}
        }
    }
]


# ── Agent ─────────────────────────────────────────────────────────────────────

class RobotAgent:
    def __init__(self):
        # Provider is controlled by env vars — swap without touching code:
        #   Groq (free):    LLM_BASE_URL=https://api.groq.com/openai/v1  LLM_API_KEY=gsk_...
        #   DeepSeek:       LLM_BASE_URL=https://api.deepseek.com         LLM_API_KEY=sk-...
        #   OpenAI:         LLM_BASE_URL=https://api.openai.com/v1        LLM_API_KEY=sk-...
        #   Ollama (local): LLM_BASE_URL=http://localhost:11434/v1         LLM_API_KEY=ollama
        self.client = OpenAI(
            api_key=os.environ.get("LLM_API_KEY") or os.environ.get("DEEPSEEK_API_KEY"),
            base_url=os.environ.get("LLM_BASE_URL", "https://api.deepseek.com"),
        )
        self.model = os.environ.get("LLM_MODEL", "deepseek-v4-pro")
        self.mc = MasterControl()

        map_path = os.path.expanduser(
            "~/learning_ws/src/mobile_manipulator/config/semantic_map.yaml"
        )
        self.semantic_map, self.room_groups, self.object_inventory = \
            load_semantic_map(map_path)

        self.messages = [
            {"role": "system", "content": self._build_system_prompt()}
        ]
        rospy.loginfo(f"[Agent] Ready — model: {self.model}")

    # ── Prompt builder ─────────────────────────────────────────────────────

    def _build_system_prompt(self) -> str:
        rooms_section = "\n".join(
            f"  - {room}: {', '.join(sorted(locs))}"
            for room, locs in sorted(self.room_groups.items())
        )
        if self.object_inventory:
            inv_section = "\n".join(
                f"  - {obj} → {loc}"
                for obj, loc in sorted(self.object_inventory.items())
            )
        else:
            inv_section = "  (none yet)"

        return (
            "You are an autonomous mobile manipulator robot (Husky base + UR5 arm) inside a house.\n"
            "\n"
            "## Rooms & Navigable Locations\n"
            f"{rooms_section}\n"
            "\n"
            "## Known Object Locations\n"
            f"{inv_section}\n"
            "\n"
            "## Rules\n"
            "1. To pick up an object you MUST call drive_to_location first, then pick_object.\n"
            "2. To place an object you MUST call drive_to_location first, then place_object.\n"
            "3. A full pick-and-place task requires: drive → pick → drive → place (four calls).\n"
            "4. Use list_locations if you are unsure which locations exist.\n"
            "5. NEVER invent location names not listed above.\n"
            "6. If an object is not in your inventory, say so and ask which room to search.\n"
        )

    # ── Inventory ──────────────────────────────────────────────────────────

    def update_inventory(self, object_name: str, location_name: str):
        object_name   = object_name.lower()
        location_name = location_name.lower()
        old = self.object_inventory.get(object_name)
        self.object_inventory[object_name] = location_name
        if old and old != location_name:
            rospy.loginfo(f"[INV] '{object_name}': {old} → {location_name}")
        else:
            rospy.loginfo(f"[INV] Registered '{object_name}' @ {location_name}")

    # ── Object search ──────────────────────────────────────────────────────

    def _scan_for_object(self, target_class: str) -> tuple:
        return self.mc.scan_with_arm(target_class)

    def _get_current_location(self) -> str | None:
        pose = self.mc.get_current_pose()
        if pose is None:
            return None
        robot_x, robot_y, _ = pose
        closest, closest_dist = None, float('inf')
        for name, data in self.semantic_map.items():
            d = math.hypot(robot_x - data['x'], robot_y - data['y'])
            if d < closest_dist:
                closest_dist, closest = d, name
        return closest if closest_dist < 2.0 else None

    # ── Tool executor ──────────────────────────────────────────────────────

    def execute_physical_tool(self, tool_name: str, args: dict) -> str:

        if tool_name == "drive_to_location":
            try:
                cmd = DriveCommand(**args)
                target = cmd.location_name.lower()
                if target not in self.semantic_map:
                    available = ", ".join(sorted(self.semantic_map))
                    return f"FAILURE: '{target}' not on map. Available: {available}"
                c = self.semantic_map[target]
                ok = self.mc.move_base_to(c['x'], c['y'], math.degrees(c['yaw_rad']))
                return (f"SUCCESS: Arrived at {target}."
                        if ok else f"FAILURE: Navigation to {target} failed.")
            except ValidationError as e:
                return f"SYSTEM ERROR: {e.errors()[0]['msg']}"

        if tool_name == "pick_object":
            try:
                cmd = PickCommand(**args)
                u, v, w = self._scan_for_object(cmd.target_class)
                if u is not None:
                    pose = self.mc.get_3d_coordinates(u, v, w)
                    if pose and self.mc.execute_pick(pose, cmd.target_class):
                        loc = self._get_current_location()
                        if loc:
                            self.update_inventory(cmd.target_class, loc)
                        return f"SUCCESS: Picked up the {cmd.target_class}."
                return f"FAILURE: Could not find '{cmd.target_class}'. Are you near it?"
            except ValidationError as e:
                return f"SYSTEM ERROR: {e.errors()[0]['msg']}"

        if tool_name == "place_object":
            try:
                cmd = DriveCommand(**args)
                target = cmd.location_name.lower()
                if target not in self.semantic_map:
                    available = ", ".join(sorted(self.semantic_map))
                    return f"FAILURE: '{target}' not on map. Available: {available}"
                c = self.semantic_map[target]
                surface_height = float(args.get("surface_height", 0.75))
                ok = self.mc.execute_place(c['x'], c['y'], surface_height)
                return (f"SUCCESS: Placed object at {target}."
                        if ok else f"FAILURE: Could not place object at {target}.")
            except ValidationError as e:
                return f"SYSTEM ERROR: {e.errors()[0]['msg']}"

        if tool_name == "list_locations":
            lines = ["Available locations by room:"]
            for room, locs in sorted(self.room_groups.items()):
                lines.append(f"  {room}: {', '.join(sorted(locs))}")
            lines.append("\nKnown object locations:")
            if self.object_inventory:
                for obj, loc in sorted(self.object_inventory.items()):
                    lines.append(f"  {obj} → {loc}")
            else:
                lines.append("  (none)")
            return "\n".join(lines)

        return f"SYSTEM ERROR: unknown tool '{tool_name}'"

    # ── Main loop ──────────────────────────────────────────────────────────

    @staticmethod
    def _msg_to_dict(msg) -> dict:
        """Serialize a ChatCompletionMessage to a plain dict.

        DeepSeek thinking mode adds reasoning_content to assistant messages.
        The field must be echoed back verbatim or the next API call returns 400.
        The Pydantic object silently drops unknown fields when the SDK serialises
        it, so we build the dict ourselves.
        """
        d: dict = {"role": "assistant"}
        if msg.content is not None:
            d["content"] = msg.content
        if msg.tool_calls:
            d["tool_calls"] = [
                {
                    "id": tc.id,
                    "type": tc.type,
                    "function": {
                        "name": tc.function.name,
                        "arguments": tc.function.arguments,
                    },
                }
                for tc in msg.tool_calls
            ]
        reasoning = getattr(msg, "reasoning_content", None)
        if reasoning:
            d["reasoning_content"] = reasoning
        return d

    def give_command(self, user_prompt: str) -> str:
        rospy.loginfo(f"[HUMAN] {user_prompt}")
        self.messages.append({"role": "user", "content": user_prompt})

        while True:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=self.messages,
                tools=ROBOT_TOOLS,
                tool_choice="auto",
            )
            choice = response.choices[0]

            # No tool calls — final text answer
            if choice.finish_reason != "tool_calls":
                text = choice.message.content
                self.messages.append(self._msg_to_dict(choice.message))
                rospy.loginfo(f"[ROBOT] {text}")
                return text

            # Append the assistant's tool-call message to history
            self.messages.append(self._msg_to_dict(choice.message))

            # Execute every tool call and collect results
            for tc in choice.message.tool_calls:
                args   = json.loads(tc.function.arguments)
                rospy.loginfo(f"[AGENT] {tc.function.name}({args})")
                result = self.execute_physical_tool(tc.function.name, args)
                rospy.loginfo(f"[TOOL]  {result}")
                self.messages.append({
                    "role":         "tool",
                    "tool_call_id": tc.id,
                    "content":      result,
                })


if __name__ == '__main__':
    try:
        rospy.init_node('robot_agent', anonymous=True)
        agent = RobotAgent()
        agent.give_command("Can you grab my cup please?")
    except rospy.ROSInterruptException:
        pass
