# Air-Ground Collaborative Manipulation — ROS Noetic Workspace

The UGV half of an **air-ground collaborative autonomous system**: a Clearpath Husky UGV with a Universal Robots UR5 arm and a Robotiq **2F-140** gripper. It navigates autonomously **indoors and outdoors**, fuses GPS for outdoor localization, detects objects with GPU-accelerated YOLO, and accepts natural-language commands via an LLM-powered agent. The outdoor stack is the foundation for collaboration with a partner **UAV** that provides aerial 3D mapping (UAV-guided terrain-aware navigation is on the roadmap).

![grasp](assets/grasp.gif)

---

## ⚠️ Build first (clone ≠ buildable)

The UAV stack's **EGO-Planner and PX4 are NOT in this repo** (git-ignored) — they are cloned/installed by the setup script. **Run it before `catkin_make`, or the build fails with a missing `ego-planner` package:**

```bash
git clone <repo> && cd <repo>
bash setup_uav.sh        # installs PX4 + clones ego-planner + applies patches/ (first run only)
catkin_make
source devel/setup.bash
```

No machine-specific paths are hardcoded: the iris mesh and workspace are resolved from `$PX4_DIR` (default `~/PX4-Autopilot`) and the package location, so a fresh clone works on any machine.

### Run the UGV without flying the UAV first

A sample UAV-derived terrain map is bundled (`src/mobile_manipulator/maps/uav_terrain_sample.pcd`), so you can run the ground-vehicle terrain navigation **without** building a fresh map. Pass `use_sample_map:=true`:

```bash
roslaunch mobile_manipulator spawn_outdoor_city.launch                 # terminal 1: world + Husky + localization
roslaunch mobile_manipulator ugv_terrain_nav.launch localization:=false use_sample_map:=true   # terminal 2
```

Without the flag it reads your own freshly-flown map from `~/pointcloud_maps/`.

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
| **Terrain Mapping** | ANYbotics `elevation_mapping` + ETH `grid_map` — UAV aerial point cloud → robot-centric elevation + slope grid map |
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

ROS packages (outdoor localization, terrain mapping, simulation):
```bash
sudo apt install ros-noetic-robot-localization ros-noetic-hector-gazebo-plugins \
                 ros-noetic-grid-map python3-vcstool
```
The terrain-mapping stack also builds four ETH/ANYbotics source packages
(`elevation_mapping`, `kindr`, `kindr_ros`, `message_logger`) — see Quick Start.

Python dependencies:
```bash
pip install openai flask pydantic numpy pillow   # numpy + pillow used by the forest generator
```

---

## :rocket:Quick Start

1. Install ROS dependencies
```bash
cd ~/Air-Ground-Collaborative-Manupulation
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
2. Install Python requirements
```bash
pip install openai flask pydantic numpy pillow
```
3. Import the ETH/ANYbotics terrain-mapping source deps (gitignored)
```bash
cd ~/Air-Ground-Collaborative-Manupulation
vcs import src < src/elevation_mapping.repos
```
4. **(可选) 安装 UAV 仿真环境**
```bash
bash setup_uav.sh
```
5. Build the workspace
```bash
cd ~/Air-Ground-Collaborative-Manupulation
catkin build
source devel/setup.bash
```
5. Launch a scenario

**UAV 自主建图** (PX4 + MAVROS + EGO-Planner，一键起飞):
```bash
bash one_key_takeoff.sh
```

**UAV PCD → 高程图** (离线处理已保存的点云):
```bash
roslaunch mobile_manipulator pcd_to_elevation.launch
```

**Outdoor air-ground world** (heightmap terrain + mountain forest, GPS-EKF localization):
```bash
roslaunch mobile_manipulator spawn_outdoor_city.launch
```

**Outdoor world + UAV terrain mapping** (adds the aerial camera + elevation map):
```bash
roslaunch mobile_manipulator outdoor_mapping.launch
```

**Indoor LLM-driven manipulation** (full agent + web UI):
```bash
export OPENAI_API_KEY="your-key-here"
chmod +x bringup.sh
./bringup.sh
# Open http://localhost:5000
```

---

## :helicopter: UAV 自主建图（PX4 + EGO-Planner）

仓库已集成 UAV 建图管线，无需额外克隆 ego_ws。队友 clone 后一键安装即可启动 UAV 飞行建图。

### 安装

```bash
cd ~/Air-Ground-Collaborative-Manupulation
bash setup_uav.sh
```

`setup_uav.sh` 会自动完成：
- 安装 ROS 依赖（MAVROS、OctoMap、grid_map 等）
- 克隆 EGO-Planner 到 `src/ego-planner/`
- 安装 PX4-Autopilot SITL 到 `~/PX4-Autopilot/`
- 安装 MAVROS 地理数据集
- 配置 Gazebo 模型路径

### 一键 UAV 起飞建图

```bash
source devel/setup.bash
bash one_key_takeoff.sh
```

自动启动 Gazebo 世界 + PX4 仿真 + MAVROS + EGO-Planner 自主飞行。UAV 自动起飞，EGO-Planner 自主规划避障航线。飞行过程中 LiDAR 点云自动累积保存：

- `~/pointcloud_maps/uav_points_map_latest.pcd` — 始终最新
- `~/pointcloud_maps/uav_points_map_YYYYMMDD_HHMMSS.pcd` — 带时间戳

Ctrl-C 退出时自动保存最终版本。也可随时手动存一版：
```bash
rosservice call /uav_pointcloud_map_recorder/save "{}"
```

### 建图飞行高度与避障说明（重要）

`forest_uav_mapping.launch`（`one_key_takeoff.sh` 内部调用）暴露了几个关键参数。EGO-Planner 是**局部规划器**，对大体量建筑（楼）避障很弱——直线初始轨迹扎进楼体后无法弹出，规划失败、原地卡死（日志 `first_optimize_step_success=0` 死循环）。因此建图采用**飞到所有建筑之上、从上往下扫**的策略：

- **`flight_height`（默认 12 m）** — 点击目标点 / 预设航点的飞行高度。`outdoor_city` 的楼最高约 11 m，故默认 12 m 以清楚所有楼。压低（更密的低处细节）：`flight_height:=10`，但**不要低于楼高+1 m**，否则会撞楼或卡死。
- **`ground_filter_margin`（默认 10）** — EGO 避障地图忽略 z ≤ 10 m 的所有点，巡航高度时地图为空 → 不撞楼、不卡。需要低空精细避障再调回 ~0.1（但低空在楼群里 EGO 仍会卡）。
- **`map_size_x/y`（默认 200）** — EGO 规划地图以世界原点为中心、覆盖 x,y ∈ [−100, 100] m。**点击目标必须落在 ±100 m 内**，否则出界 → `terminal in obstacle` 规划失败。要扫更远：同时加大 `map_size` 并调粗 `grid_map/resolution`（`forest_param.xml`）。

> **本质约束**：楼高 ~11 m + EGO 局部规划器 ⇒ 没有“低空 + 安全避障”的中间地带，城区建图须从楼顶之上俯扫。真正的低空贴墙建图需要带全局路径搜索的规划器。

### PCD 点云 → 高程图

```bash
source devel/setup.bash
roslaunch mobile_manipulator pcd_to_elevation.launch \
    pcd_file:=~/pointcloud_maps/uav_points_map_latest.pcd
```

流程：PCD 回放 → 体素降采样 → `elevation_mapping` → GridMap 高程图层。

输出 topic：
- `/elevation_mapping/elevation_map_postprocessed` — 后处理高程图（高程 + 坡度）
- `/elevation_mapping/elevation_map_raw` — 原始融合图

### UAV 建图包结构

| 目录 | 用途 |
|------|------|
| `src/uav_truth_tracker/` | UAV 点云导出、坐标转换、PCD 保存、PX4 桥接 |
| `src/forest_avoidance_utils/` | 深度预处理、OctoMap 管理、轨迹记录 |
| `src/ego-planner/` | EGO-Planner 自主飞行规划（setup_uav.sh 自动克隆） |
| `models/` | UAV LiDAR 模型 |
| `worlds/` | Gazebo 世界文件 |
| `one_key_takeoff.sh` | 一键 UAV 起飞建图 |
| `setup_uav.sh` | 一键安装 UAV 仿真环境 |
| `px4_bridge.py` | PX4 OFFBOARD 自动起飞控制 |

### 环境变量

所有路径和参数可通过环境变量覆盖：

```bash
# 默认村庄地图，可切换为森林
GAZEBO_WORLD=~/worlds/forest_corridor.world bash one_key_takeoff.sh

# PX4 目录 / 关闭 RViz / 开启 Gazebo GUI
PX4_DIR=~/PX4-Autopilot START_RVIZ=false PX4_GAZEBO_GUI=true bash one_key_takeoff.sh
```

### 完整空-地管线

```
UAV 起飞建图               PCD 转高程图               UGV 导航
─────────────             ────────────              ────────
one_key_takeoff.sh   →   pcd_to_elevation.launch → husky_forest_amcl.launch
PX4 + EGO-Planner         elevation_mapping         AMCL + move_base
     ↓                         ↓                        ↓
 ~/pointcloud_maps/     /elevation_mapping/         /cmd_vel
 uav_points_map_        elevation_map_             Husky 自动驾驶
 latest.pcd             postprocessed
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

### Terrain mapping — UAV → elevation map (`launch/elevation_mapping.launch`)
The partner UAV's role is stood in by a **simulated downward depth camera** hovering
over the operational area (`models/uav_mapper/`, spawned at ~130 m), publishing a
dense aerial `PointCloud2` on `/uav/points`. That cloud is down-sampled and fused by
ANYbotics **`elevation_mapping`** into a robot-centric **`grid_map`** in the GPS-anchored
`map` frame, tracking the Husky's `base_link`.

- Config: `config/elevation_mapping.yaml` (map geometry, frames, `perfect` sensor model;
  pose taken from the dual-EKF TF chain, so no extra pose topic is needed).
- Post-processing (`config/elevation_postprocessor.yaml`): inpaint holes → surface
  normals → a **`slope`** layer, published on `/elevation_mapping/elevation_map_postprocessed`
  and visualized in RViz (height = elevation, colour = slope).
- When the real UAV is ready, point `input_sources` at its cloud topic and drop the
  simulated camera — nothing else changes.

Bring it up together with the world via `outdoor_mapping.launch`. Verified against the
Gazebo ground truth: ~0.2 m error on flat ground, ~0.5 m median on slopes (from 130 m).

> **Note:** the world's `grass_plane` sits at z = 0, so the heightmap's sub-zero central
> valley reads flat in the map; real relief shows on the foothills (heightmap > 0). To map
> dramatic terrain near spawn, remove/lower the grass plane or operate over the hills.

### Roadmap
Terrain-aware navigation: turn the elevation/slope grid map into a `grid_map_costmap_2d`
traversability layer for `move_base`, then drive GPS-localized terrain-aware paths.

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
- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) (ANYbotics/ETH) — robot-centric terrain elevation mapping
- [grid_map](https://github.com/ANYbotics/grid_map) (ETH) — universal grid map library for the elevation/slope layers
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) — Pose-graph SLAM and localization
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) — Object detection model
- [gazebo_models_worlds_collection](https://github.com/leonhartyao/gazebo_models_worlds_collection) — outdoor terrain, tree, and building models
- [gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs) — Gazebo grasp and state simulation plugins
- [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world) — optional indoor house environment
