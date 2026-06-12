# Air-Ground Collaborative Manipulation — ROS Noetic Workspace

The UGV half of an **air-ground collaborative autonomous system**: a Clearpath Husky UGV with a Universal Robots UR5 arm and a Robotiq **2F-140** gripper. It navigates autonomously **indoors and outdoors**, fuses GPS for outdoor localization, detects objects with GPU-accelerated YOLO, and accepts natural-language commands via an LLM-powered agent. The outdoor stack is the foundation for collaboration with a partner **UAV** that provides aerial 3D mapping (UAV-guided terrain-aware navigation is on the roadmap).

![grasp](assets/grasp.gif)

---

## System Overview

```
Natural Language Command
        │
        ▼
  LLM Agent (OpenAI-compatible API)
        │  tool calls
        ▼
  MasterControl
  ├── Navigation   → move_base (AMCL / SLAM Toolbox indoors · GPS-EKF outdoors)
  ├── Manipulation → MoveIt (UR5 arm + Robotiq 2F-140 gripper)
  └── Perception   → TensorRT YOLOv8 + PointCloud2
```

### :star:Features

| Layer | Technology |
|---|---|
| **Robot Base** | Clearpath Husky UGV — differential drive, 4-wheel |
| **Manipulator** | Universal Robots UR5 — 6-DOF, 850 mm reach |
| **Gripper** | Robotiq **2F-140** — parallel-jaw, 140 mm stroke (for larger outdoor objects) |
| **State Estimation** | robot_localization — local EKF fusing wheel odometry + IMU |
| **Outdoor Localization** | Dual-EKF + `navsat_transform` — GPS-fused `map→odom` for outdoor navigation |
| **Indoor Localization** | SLAM Toolbox (pose-graph SLAM) |
| **Navigation** | ROS Navigation Stack — navfn global planner + DWA local planner |
| **Motion Planning** | MoveIt + OctoMap + OMPL (RRTConnect) + KDL IK solver |
| **Object Detection** | YOLOv8m — ONNX model compiled to TensorRT for GPU inference |
| **3D Perception** | RGB-D depth image → PointCloud2 — 3D object localization for grasping |
| **LLM Agent** | OpenAI-compatible API (DeepSeek / Groq / Ollama / OpenAI) with tool-use |
| **Web UI** | Flask + HTML/JS — real-time command dashboard |
| **Simulation** | Gazebo 11 — outdoor heightmap world + forest, indoor cafe/house, grasp plugin |

---

## :mortar_board:Prerequisites

- C++17 Compiler
- ROS Noetic (Ubuntu 20.04)
- Gazebo 11
- CUDA + TensorRT (for GPU-accelerated object detection)
- OpenCV
- Python 3.8+
- `catkin_tools` or `catkin_make`

ROS packages (outdoor localization + simulation):
```bash
sudo apt install ros-noetic-robot-localization ros-noetic-hector-gazebo-plugins
```

Python dependencies:
```bash
pip install openai flask pydantic numpy pillow   # numpy + pillow used by the forest generator
```

---

## :rocket:Quick Start

1. Install ROS dependencies
```bash
cd ~/learning_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
2. Install Python requirements
```bash
pip install openai flask pydantic numpy pillow
```
3. Build the workspace
```bash
cd ~/learning_ws
catkin build
source devel/setup.bash
```
4. Launch a scenario

**Outdoor air-ground world** (heightmap terrain + mountain forest, GPS-EKF localization):
```bash
roslaunch mobile_manipulator spawn_outdoor_city.launch
```

**Indoor LLM-driven manipulation** (full agent + web UI):
```bash
export OPENAI_API_KEY="your-key-here"
chmod +x bringup.sh
./bringup.sh
# Open http://localhost:5000
```

---

## :evergreen_tree:Outdoor Air-Ground System

### World — `worlds/outdoor_city.world`
A 500 m × 500 m **heightmap mountainous terrain** (up to ~100 m elevation) with a grass plane, a small town (houses, gas station, lamp posts, signs) in the central valley, and a **130-tree forest** scattered across the surrounding slopes. Tree density and gaps are tuned so both the UGV and a UAV can traverse easily.

### Forest generator — `scripts/generate_forest.py`
Reproducible, slope-aware tree placement. It seats every tree on the highest ground under its canopy footprint (so slopes never bury trunks), keeps the city/spawn clear, and enforces a minimum gap for traversability. The block it writes is idempotent (re-run to re-tune).
```bash
# tune count / spacing / how far down the foothills trees reach / seed
python3 src/mobile_manipulator/scripts/generate_forest.py --trees 130 --spacing 14 --elev-min 8 --seed 7
python3 src/mobile_manipulator/scripts/generate_forest.py --dry-run   # preview without writing
```
Key in-file constants: `SINK` (raise/lower trees — negative lifts), `MAX_SLOPE`, `TRUNK_R`, `SPAWN_CLEAR`. Tree size lives in the `<scale>` of `Tree_1.sdf` / `Tree_2.sdf`.

### Outdoor localization — dual-EKF with GPS
- **Local EKF** (`husky_control/localization.yaml`): wheel odom + IMU → `odom→base_link`
- **`navsat_transform_node`** (`config/navsat_transform.yaml`): Husky's simulated GPS (`navsat/fix`, hector plugin) → map-frame `odometry/gps`
- **Global EKF** (`config/ekf_global.yaml`): wheel odom + IMU + GPS → `map→odom`

### Roadmap
UAV-guided terrain-aware navigation: ingest the partner UAV's 3D point cloud → elevation map → traversability costmap layer for `move_base`.

---

## :package:Packages

| Package | Description |
|---|---|
| `mobile_manipulator` | Core package — navigation, manipulation, vision, LLM agent, web UI, outdoor world + forest generator |
| `husky_ur5_moveit_config` | MoveIt motion planning config for the Husky+UR5 (2F-140 gripper) |
| `robotiq` | Robotiq gripper drivers/descriptions (Hand-E, 2F-85, 2F-140); the robot uses the **2F-140** |
| `gazebo-pkgs` | Gazebo grasp/state/world simulation plugins |
| `gazebo_models_worlds_collection` | Outdoor world models (heightmap terrain, trees, buildings) — trimmed to the subset the worlds use |

> **Optional (not bundled):** the indoor `spawn_robot.launch` demo uses the [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world). It's large (~131 MB) and gitignored — clone it into `src/` only if you need the indoor house scene.

---

## Architecture

### mobile_manipulator — Key Components

#### MasterControl (`src/mobile_manipulator/master_control.py`)
Core robot control API. Exposes high-level primitives used by the agent:

| Method | Description |
|---|---|
| `move_base_to(x, y, yaw)` | Navigate to a map pose |
| `move_arm_to_pick(class_name)` | Detect and pick a COCO object (2F-140, diameter→jaw-angle mapping) |
| `move_arm_to_place(location, height)` | Place held object |
| `get_current_pose()` | TF-based localization |
| `query_object_detection()` | Call TensorRT YOLO service |

Uses TF2, MoveIt, move_base action client, and PointCloud2 for 3D localization.

#### RobotAgent (`scripts/agent.py`)
LLM-powered autonomous agent. Accepts natural language commands and decomposes them into tool calls:

- `drive_to_location(name)` — navigate to a semantic room/location
- `pick_object(class)` — pick a COCO-class object
- `place_object(name, height)` — place at a semantic location
- `list_locations()` — query the semantic map

Supports any OpenAI-compatible backend (DeepSeek, Groq, Ollama, OpenAI).
```bash
export OPENAI_API_KEY=<your-key>          # or DEEPSEEK_API_KEY, GROQ_API_KEY
rosrun mobile_manipulator agent.py
```

#### TensorRT YOLO Node (`src/trt_yolo_node.cpp`)
Real-time object detection node:
- Model: YOLOv8m (ONNX → TensorRT engine; the engine is built per-GPU at runtime, not committed)
- Input: 640×640 camera images
- Output: `/yolo/detections/image` topic + `DetectObjects` ROS service
- Classes: COCO-80, confidence threshold 0.5
```bash
rosrun mobile_manipulator trt_yolo_node
```

#### Flask Web UI (`scripts/flask_ui.py`)
Real-time web dashboard at `http://localhost:5000`:

| Endpoint | Method | Description |
|---|---|---|
| `/api/command` | POST | Send natural language command |
| `/api/map` | GET | Semantic map + robot pose |
| `/api/save_pose` | POST | Record current pose as named location |
| `/api/rooms` | GET | List all rooms |
| `/api/history` | GET | Conversation history |
| `/api/estop` | POST | Emergency stop toggle |
```bash
rosrun mobile_manipulator flask_ui.py
# Open http://localhost:5000
```

#### Semantic Map (`config/semantic_map.yaml`)
YAML database of ~50 named locations across 5 rooms (bedroom, living room, kitchen, dining room, study). Stores `(x, y, yaw)` poses and an object inventory per room. Persisted to disk when new locations are saved via the web UI.

---

## Robot Description

The URDF (`urdf/husky_ur5.urdf.xacro`) assembles:

1. **Husky UGV** — differential drive base (with hector GPS + IMU plugins for outdoor localization)
2. **SICK LMS1XX LiDAR** — mounted at 25.7 cm for obstacle detection
3. **UR5 arm** — 6-DOF, 850 mm reach, mounted on top plate
4. **Virtual ballast** — 30 kg mass to prevent tipping during arm extension
5. **Robotiq 2F-140 gripper** — 140 mm stroke, attached at UR5 `tool0`, with the `gazebo_grasp_fix` plugin bound to the inner-finger links

TF chain: `map → odom → base_link → ur5_base_link → ... → tool0 → gripper`

---

## Configuration

| File | Purpose |
|---|---|
| `config/ekf_global.yaml` | Global EKF (`map→odom`) fusing GPS for outdoor localization |
| `config/navsat_transform.yaml` | `navsat_transform_node` — GPS (NavSatFix) → map-frame odometry |
| `config/nav/dwa_local_planner.yaml` | DWA local planner tuning |
| `config/nav/amcl.yaml` | AMCL localization parameters |
| `config/nav/costmap_common.yaml` | Shared costmap settings |
| `config/laser_filter_arm.yaml` | Filters arm self-returns from LiDAR |
| `config/ur5_controllers.yaml` | UR5 + 2F-140 gripper joint trajectory controllers |
| `config/slam_toolbox/` | SLAM Toolbox mapping and localization configs |
| `config/semantic_map.yaml` | Semantic navigation location database |

---

## LLM Backend

The agent uses the OpenAI Python SDK and works with any compatible API:

| Provider | Environment Variable |
|---|---|
| OpenAI | `OPENAI_API_KEY` |
| DeepSeek | `DEEPSEEK_API_KEY` |
| Groq | `GROQ_API_KEY` |
| Ollama (local) | No key required; set base URL |

---

## :envelope:Custom ROS Messages

**`msg/Detection.msg`** — Single object detection:
```
int32 x1, y1, x2, y2   # bounding box
string class_name
float32 confidence
```

**`srv/DetectObjects.srv`** — Detection service:
```
sensor_msgs/Image image
---
Detection[] detections
sensor_msgs/Image annotated_image
float32 inference_time_ms
```

---

## Acknowledgements

- [Clearpath Robotics](https://clearpathrobotics.com/) — Husky UGV platform and `husky_description` ROS package
- [Universal Robots](https://www.universal-robots.com/) — UR5 arm and `ur_description` ROS package
- [Robotiq](https://robotiq.com/) — 2F-140 gripper and `robotiq` ROS package
- [MoveIt](https://moveit.ros.org/) — Motion planning framework for the UR5 arm
- [OctoMap](https://github.com/OctoMap/octomap) — Collision avoidance for motion planning
- [ROS Navigation Stack](https://wiki.ros.org/navigation) — `move_base`, AMCL, navfn, and DWA planner
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization) — EKF state estimation and `navsat_transform`
- [hector_gazebo_plugins](https://wiki.ros.org/hector_gazebo_plugins) — simulated GPS/IMU for outdoor localization
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) — Pose-graph SLAM and localization
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) — Object detection model
- [gazebo_models_worlds_collection](https://github.com/leonhartyao/gazebo_models_worlds_collection) — outdoor terrain, tree, and building models
- [gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs) — Gazebo grasp and state simulation plugins
- [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world) — optional indoor house environment
