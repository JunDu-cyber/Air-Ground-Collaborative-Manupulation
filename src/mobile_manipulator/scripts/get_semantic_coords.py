#!/usr/bin/env python3
import rospy
import yaml
import math
from gazebo_msgs.srv import GetModelState
import tf.transformations as tf_trans
# All semantic furniture from small_house.world
# (walls, floors, windows, doors, lights, portraits, robot excluded)
MODELS = [
    # bedroom
    ('Bed_01_001',              'bed'),
    ('NightStand_01_001',       'nightstand_left'),
    ('NightStand_01_002',       'nightstand_right'),
    ('Wardrobe_01_001',         'wardrobe'),

    # living room
    ('SofaC_01_001',            'sofa'),
    ('CoffeeTable_01_001',      'coffee_table'),
    ('TV_01_001',               'tv_living_room'),
    ('TVCabinet_01_001',        'tv_cabinet'),
    ('Vase_01_001',             'vase'),
    ('Carpet_01_001',           'carpet_living_room'),

    # kitchen / dining
    ('KitchenTable_01_001',     'kitchen_table'),
    ('KitchenCabinet_01_001',   'kitchen_cabinet'),
    ('CookingBench_01_001',     'cooking_bench'),
    ('Refrigerator_01_001',     'refrigerator'),
    ('Rangehood_01_001',        'rangehood'),
    ('KitchenUtensils_01_001',  'kitchen_utensils'),
    ('SeasoningBox_01_001',     'seasoning_box'),
    ('Tableware_01_001',        'tableware'),
    ('Trash_01_001',            'trash_kitchen'),

    # dining chairs
    ('ChairA_01_001',           'chair_dining_1'),
    ('ChairA_01_002',           'chair_dining_2'),
    ('ChairA_01_003',           'chair_dining_3'),
    ('ChairA_01_004',           'chair_dining_4'),

    # study / reading area
    ('ReadingDesk_01_001',      'reading_desk'),
    ('ChairD_01_001',           'chair_desk_1'),
    ('ChairD_01_002',           'chair_desk_2'),
    ('ChairD_01_003',           'chair_desk_3'),
    ('Board_01_001',            'board'),
    ('TV_02_001',               'tv_study'),
    ('Tablet_01_001',           'tablet'),

    # gym area
    ('FitnessEquipment_01_001', 'fitness_equipment'),
    ('Dumbbell_01_001',         'dumbbell'),

    # balcony / entrance
    ('BalconyTable_01_001',     'balcony_table'),
    ('ShoeRack_01_001',         'shoe_rack'),
    ('Trash_01_002',            'trash_entrance'),

    # misc
    ('Carpet_01_002',           'carpet_bedroom'),
    ('Ball_01_001',             'ball_living_room'),
    ('Ball_01_003',             'ball_bedroom'),
]

MAP_YAML = '/home/jun/learning_ws/src/mobile_manipulator/maps/small_house_map.yaml'

# Nav goal offset from furniture center (meters)
# Robot needs to stand NEXT to furniture, not on top of it
NAV_OFFSET = 1.0

# --- GLOBAL MAP ALIGNMENT ---
# Your SLAM map has an offset and slight rotation compared to the Gazebo world.
# Tune these values to perfectly align the green boxes with your map walls in RViz.
GLOBAL_OFFSET_X = -0.0  # try shifting left
GLOBAL_OFFSET_Y = -0.0   # try shifting down 
GLOBAL_YAW = -1.57         # rotation in radians (e.g. 0.05 or -0.05)

def main():
    rospy.init_node('get_semantic_coords', anonymous=True)

    with open(MAP_YAML) as f:
        map_data = yaml.safe_load(f)
    print(f"# Loaded map data from {MAP_YAML}\n")
    print("# Note: map.yaml 'origin' defines the map image bounds. It is NOT the origin of the SLAM map frame.")
    print("# Assuming the robot spawned at (0,0,0) when mapping started, Gazebo world frame == ROS map frame.\n")

    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    regions = []
    not_found = []

    for model_name, semantic_name in MODELS:
        try:
            resp = get_state(model_name, 'world')
            if not resp.success:
                not_found.append(model_name)
                continue

            gx = resp.pose.position.x
            gy = resp.pose.position.y

            # 1. Apply Translation Offset
            tx = gx + GLOBAL_OFFSET_X
            ty = gy + GLOBAL_OFFSET_Y
            
            # 2. Apply Rotation
            import math
            mx = tx * math.cos(GLOBAL_YAW) - ty * math.sin(GLOBAL_YAW)
            my = tx * math.sin(GLOBAL_YAW) + ty * math.cos(GLOBAL_YAW)

            mx = round(mx, 3)
            my = round(my, 3)

            # --- ADJUST NAV GOAL POSITION HERE ---
            # By default, use the global offset (South)
            offset_x = 0.0
            offset_y = -NAV_OFFSET
            goal_yaw = 1.57 # Face North by default

            # Example: Customize the goal per furniture item so HUSKY is in a good range for UR5
            if semantic_name == 'bed':
                offset_x = 1.2
                offset_y = 0.0
                goal_yaw = 3.14 # Face West towards the bed
            elif semantic_name == 'refrigerator':
                offset_x = 0.0
                offset_y = -1.2
                goal_yaw = 1.57
            # Add more elif statements for other specific items to customize nav positions...

            nav_x = round(mx + offset_x, 3)
            nav_y = round(my + offset_y, 3)

            # Get object rotation from Gazebo
            q = [resp.pose.orientation.x, resp.pose.orientation.y, resp.pose.orientation.z, resp.pose.orientation.w]
            _, _, obj_yaw = tf_trans.euler_from_quaternion(q)
            
            # --- CUSTOM BOUNDING BOX SIZES (Half Length, Half Width) ---
            hx, hy = 0.5, 0.5 # default 1m x 1m
            if 'bed' in semantic_name: hx, hy = 1.0, 1.1
            elif 'sofa' in semantic_name: hx, hy = 1.5, 0.6
            elif 'table' in semantic_name or 'desk' in semantic_name: hx, hy = 0.4, 0.8
            elif 'wardrobe' in semantic_name: hx, hy = 0.6, 0.3
            elif 'refrigerator' in semantic_name: hx, hy = 0.4, 0.4
            elif 'tv' in semantic_name: hx, hy = 0.1, 0.6
            elif 'trash' in semantic_name: hx, hy = 0.2, 0.2


            if 'carpet' in semantic_name: hx, hy = 2.0, 2.0
            if 'nightstand' in semantic_name : hx, hy = 0.3, 0.3
            if 'chair' in semantic_name: hx, hy = 0.3, 0.3
            if 'tableware' in semantic_name: hx, hy = 0.3, 0.15
            if 'shoe_rack' in semantic_name: hx, hy = 0.5, 0.2
            if 'ball' in semantic_name: hx, hy = 0.3, 0.3
            if 'coffee_table' in semantic_name: hx, hy = 0.8, 0.4
            if 'tablet' in semantic_name: hx, hy = 0.2, 0.2
            if 'dumbbell' in semantic_name: hx, hy = 0.3, 0.6
            if 'utensils' in semantic_name: hx, hy = 0.01, 0.2
            if 'vase' in semantic_name: hx, hy = 0.05, 0.05
            if 'board' in semantic_name: hx, hy = 0.6, 0.1
            if 'tv_cabinet' in semantic_name: hx, hy = 0.4, 1.2
            
            # Allow items to define custom 6-point L-shapes instead of just rectangles!
            corners = None
            
            if 'sofa' == semantic_name: 
                # L-shaped Sofa Custom Polygon (6 points instead of 4)
                # Drawing an L shape in the local X/Y bounding box
                corners = [
                    (-1.1, -1.0),
                    ( 1.1, -1.0),
                    ( 1.1,  1.0),
                    ( 0.3,  1.0),
                    ( 0.3, -0.2),
                    (-1.1, -0.2)
                ]
                obj_yaw += 3.14
            elif 'kitchen_cabinet' in semantic_name:
                # Custom L-shaped Kitchen Cabinet
                corners = [
                    (-1.2, -1.5),
                    ( 1.2, -1.5),
                    ( 1.2,  1.5),
                    ( 0.6,  1.5),
                    ( 0.6, -0.9),
                    (-1.2, -0.9)
                ]
                obj_yaw += 3.14

            # Calculate total rotation of the polygon (Object Yaw + Global offset)
            total_yaw = obj_yaw + GLOBAL_YAW
            
            if corners is None:
                corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
            poly = []
            for cx, cy in corners:
                rcx = cx * math.cos(total_yaw) - cy * math.sin(total_yaw)
                rcy = cx * math.sin(total_yaw) + cy * math.cos(total_yaw)
                poly.append([round(mx + rcx, 2), round(my + rcy, 2)])

            region = {
                'name': semantic_name,
                'x': nav_x,
                'y': nav_y,
                'yaw': goal_yaw,
                'polygon': poly
            }
            regions.append(region)

        except Exception as e:
            not_found.append(f"{model_name} ({e})")

    # write yaml
    output = {'regions': regions}
    output_path = '/home/jun/learning_ws/src/mobile_manipulator/config/semantic_map.yaml'
    with open(output_path, 'w') as f:
        yaml.dump(output, f, default_flow_style=False, sort_keys=False)

    print(f"# Written {len(regions)} regions to {output_path}")
    if not_found:
        print(f"\n# Not found ({len(not_found)}):")
        for n in not_found:
            print(f"#   {n}")

    print("\n# IMPORTANT: Review nav goal x,y for each region.")
    print("# The y offset (-1.0m) is a default — adjust per furniture")
    print("# so HUSKY stands where the UR5 arm can actually reach the surface.")

if __name__ == '__main__':
    main()