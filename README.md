# Air-Ground Collaborative Landmine Survey and Response

*[中文说明](README-zh.md)*

A ROS Noetic / Gazebo Classic research workspace for a simulated **UAV–UGV collaborative mission**. A PX4 UAV surveys a mine field with a downward RGB-D camera and a YOLO11 segmentation model; confirmed locations are transformed into the UGV's egocentric frame. A Husky–UR5 UGV uses LiDAR-inertial odometry, an elevation-aware planner, and (optionally) a MoveIt/GPD grasp pipeline to visit those targets.

> **Scope.** This is a simulation and course/research project, not a real-world explosive-ordnance-disposal system. Run it only in the supplied Gazebo/PX4 environment.

## 1. :mortar_board:Project basic information

| | |
|---|---|
| Group No. | `3` |
| Assignment name | `空地协同户外操作系统` |
| Project name | Air-Ground Collaborative Landmine Survey and Response |

## 2. Function description

**Problem.** Locate simulated landmines from the air and have a ground robot visit and (optionally) grasp each confirmed target, without any pre-built global map or GPS.

**Main features:**
- UAV survey with a down-facing RGB-D camera and a YOLO11s-seg landmine detector.
- Multi-frame spatial confirmation to turn noisy per-frame detections into a stable mine map.
- Egocentric handoff of confirmed targets into the UGV's local `odom` frame (no shared global `map` frame, no GPS required for the mission itself).
- UGV autonomy: LiDAR-inertial odometry, elevation-aware path planning, ordered target visitation.
- Optional MoveIt Task Constructor / GPD grasp pipeline for the simulated mine prop.

**Method (brief — see [`docs/algorithm_research_references.md`](docs/algorithm_research_references.md) for the underlying algorithm survey, not reproduced here):** PX4 SITL + EGO-Planner for UAV flight, a TensorRT/Ultralytics YOLO11s-seg model for detection, DLIO for UGV odometry, a CMU-derived local planner with an ANYbotics `elevation_mapping` cost layer for UGV navigation, and MoveIt Task Constructor + GPD for the optional grasp.

**Inputs:** the supplied `outdoor_city` Gazebo world (mine field baked in), the shipped YOLO11s-seg weights, and runtime environment variables (§8).

**Outputs:** a confirmed mine map (`mine_detection_output/mine_map.yaml`), UGV tour/grasp status on ROS topics, and optional RViz visualization.

**Applicable scenario:** a single-workstation Gazebo Classic simulation; not applicable to real hardware without significant additional work (see Scope above).

### Mission pipeline

```text
PX4 UAV + down-facing RGB-D camera
          │
          ▼
YOLO11s-seg landmine localization
          │  /mine_detection/raw
          ▼
multi-frame association and confirmation
          │  /mine_detection/map (MineMap)
          ▼
MineMap → WorldTarget bridge
          │  /detected_targets
          ▼
UGV target tour + elevation-aware planning
          │  /ugv/goal
          ▼
optional MoveIt / GPD grasp-and-return cycle
```

The air-ground transform is established once at startup: DLIO owns `odom → base_link`, and an anchor latches `odom → uav0/map_local`. The system therefore does not require a global `map` frame or GPS during the main egocentric mission.

## 3. Directory layout

| Path | Contents |
|---|---|
| `airground_takeoff.sh` | Main integrated demo launcher. |
| `src/uav_truth_tracker/` | UAV ROS nodes, survey, mine localization/fusion, PX4 launch files, and custom messages. |
| `src/mobile_manipulator/` | Husky–UR5 simulation, DLIO/planning integration, elevation filters, target tour, and bridge node. |
| `src/grasp_mtc/` | MoveIt Task Constructor grasp pipeline. |
| `models/` | Gazebo UAV and mine-camera models. |
| `mine_seg_v2_delivery/` | Delivered YOLO segmentation weight and model manifest. |
| `docs/` | Design notes, runbooks, testing notes, and presentation material. |
| `patches/` | Project patches applied to the external EGO-Planner checkout. |

## 4. Runtime environment

| Component | Tested target |
|---|---|
| OS | Ubuntu 20.04 |
| ROS | ROS Noetic |
| Simulator | Gazebo Classic 11 |
| Flight stack | PX4 v1.14.3 SITL + MAVROS + EGO-Planner |
| Build toolchain | `catkin_make` or `catkin build`, C++17, Python 3.8 (Noetic's system Python 3) |
| Perception (Python) | `ultralytics`, `torch`, `opencv-python`, `numpy`, `PyYAML` (no pinned versions shipped — install current releases compatible with Python 3.8) |
| Optional GPU acceleration | NVIDIA driver + CUDA + TensorRT, and an exported ONNX model (only needed for `UAV_DETECT_BACKEND=tensorrt`, the script default — the CPU/Ultralytics fallback needs none of this) |
| Desktop session | A desktop OpenGL/Gazebo-capable session and `gnome-terminal` (the launcher opens one tab per subsystem — see §7) |

There are no other OS/version combinations documented or tested; if you are not on Ubuntu 20.04 + ROS Noetic, expect to adapt package names yourself.

## 5. Dependency installation

Clone the repository into a catkin workspace and run the setup from the repository root:

```bash
git clone <repository-url> learning_ws
cd learning_ws
source /opt/ros/noetic/setup.bash
bash setup_uav.sh
```

`setup_uav.sh` installs the ROS/PX4 prerequisites, clones EGO-Planner to `src/ego-planner/`, applies the project patches, installs PX4 SITL at `${PX4_DIR:-$HOME/PX4-Autopilot}`, and builds the workspace on a first run. It requires `sudo` and can take considerable time.

Install the Python perception packages if they are not already available:

```bash
python3 -m pip install --user ultralytics torch opencv-python numpy PyYAML
```

### Elevation-mapping dependencies

The ETH/ANYbotics source dependencies are intentionally not vendored. Import the pinned versions before building the elevation-mapping workflow:

```bash
sudo apt install python3-vcstool ros-noetic-grid-map ros-noetic-grid-map-visualization
vcs import src < src/elevation_mapping.repos
catkin_make
source devel/setup.bash
```

If the workspace is already initialized, rebuild after pulling changes:

```bash
catkin_make
source devel/setup.bash
```

There are no other dependencies beyond what's listed above and inside `setup_uav.sh`.

## 6. Pre-run configuration

- **Environment variables** (full table in §8) — the only one that changes *behavior* rather than just tuning is `UAV_DETECT_BACKEND`: set `UAV_DETECT_BACKEND=cpu UAV_DETECT_DEVICE=cpu` unless you have a working NVIDIA/TensorRT install, because the script defaults to `tensorrt`.
- **Model file placement.** The submission model must be present at `mine_seg_v2_delivery/weights/best.pt` (already shipped in this repo). Its manifest is [`mine_seg_v2_delivery/DEPLOYMENT.txt`](mine_seg_v2_delivery/DEPLOYMENT.txt) — task, input size (960), recommended confidence (0.7337337337 per the manifest; the launch file's own default is 0.65, see §8), and validation metrics. Use `best.pt`, not `last.pt`.
- **`PX4_DIR`.** If PX4 was installed somewhere other than `~/PX4-Autopilot`, export `PX4_DIR` before running either `setup_uav.sh` or `airground_takeoff.sh`.
- No other config file edits are required for the default run — all mission parameters are exposed as launch args / env vars (§8), not hardcoded paths.

## 7. :rocket:Complete run procedure

Start the integrated simulation from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash airground_takeoff.sh
```

On a CUDA/TensorRT machine, omit those two environment variables to use the script defaults.

**Step 1.** From the workspace root, source ROS then the workspace overlay (in that order), as shown above.
**Step 2.** Run `airground_takeoff.sh` with the environment variables above. This is a single entry point — nothing else needs to be launched by hand for the default demo.
**Step 3.** The script opens its own `gnome-terminal` tabs, in this fixed order, each internally waiting on the previous before it starts: `1_Gazebo` → `1b_Mine_Field` → `2_PX4_Spawn_UAV` → `3_UGV` → `3b_UGV_LIO` → (`3c_UGV_NAV`, `7a_MoveGroup`, `7b_Grasp` if enabled) → `4_MAVROS` → `5_AirGround_ROS` → `6_Takeoff` → `8_UAV_Detect`. You do not need to interact with any of them — the script's own `sleep`s enforce this ordering.
**Step 4.** Wait roughly 30–60 s for all tabs to come up, then proceed to the interactive steps in §10 (confirm detections, start the tour).
**Step 5.** Inspect results per §9 (Output) and §10 (Success criteria).

The launcher starts Gazebo paused, spawns the five-mine field, brings up PX4, the UGV, DLIO, planning, the air-ground TF/elevation layer, MAVROS, the UAV takeoff bridge, and mine detection.



## 8. Input specification

Environment variables consumed by `airground_takeoff.sh`:

| Variable | Default | Purpose |
|---|---:|---|
| `UAV_DETECT` | `true` | Start the UAV mine-detection stack. |
| `UAV_DETECT_BACKEND` | `tensorrt` | Set to `cpu` for the Ultralytics/PyTorch path. |
| `UAV_DETECT_DEVICE` | `cpu` | PyTorch device passed to the detector. |
| `UAV_SURVEY` | `false` | Enable the automatic UAV survey route. |
| `UGV_NAV` | `true` | Start CMU planning and the target-tour node. |
| `UGV_GRASP` | `true` | Start MoveIt/GPD and request a grasp on target arrival. |
| `FLIGHT_H` | `2.0` | UAV flight height in metres. |
| `START_RVIZ` | `true` | Start the elevation-map RViz configuration. |
| `PX4_DIR` | `~/PX4-Autopilot` | PX4 source/build directory. |

The detector launch's own `confidence` arg defaults to **0.65** (`src/uav_truth_tracker/launch/uav_mine_detection.launch`); pass `confidence:=...` only when evaluating a different operating point than the shipped default.

Runtime topic inputs the pipeline consumes:

| Topic | Type | Meaning |
|---|---|---|
| `/mine_camera/rgb/image_raw`, `.../depth`, `.../camera_info` | `sensor_msgs` | UAV down-facing RGB-D camera feed consumed by the detector. |
| `/ugv/start_tour` | `std_srvs/Trigger` (service call) | Freeze collected targets and begin the UGV tour (call manually, or auto-armed by `UAV_SURVEY=true`). |

## 9. Output specification

Each mission writes the confirmed mine map to:

```text
mine_detection_output/mine_map.yaml
```

This file is **generated output and is not source-controlled**; it is overwritten (not appended) at the start of each run, so copy it elsewhere first if you need to keep a prior result.

Live outputs are ROS topics rather than files:

| Interface | Type | Meaning |
|---|---|---|
| `/mine_detection/raw` | `uav_truth_tracker/MineDetectionArray` | Per-frame depth-localized detections. |
| `/mine_detection/map` | `uav_truth_tracker/MineMap` | Associated and confirmed mine hypotheses. |
| `/detected_targets` | `mobile_manipulator/WorldTarget` | Confirmed mine locations handed to the UGV. |
| `/ugv/tour_status` | `std_msgs/String` | Target-tour state. |
| `/ugv/goal` | `geometry_msgs/PoseStamped` | Current UGV navigation goal in `odom`. |
| `/elevation_mapping/elevation_map_postprocessed` | `grid_map_msgs/GridMap` | Fused elevation and post-processed terrain layers. |
| `/uav/pose_cov` | pose-with-covariance | UAV state covariance used by elevation mapping. |

Custom UAV messages are defined under [`src/uav_truth_tracker/msg`](src/uav_truth_tracker/msg); the UGV handoff message is [`WorldTarget.msg`](src/mobile_manipulator/msg/WorldTarget.msg).

Visualization: with `START_RVIZ=true` (the default), an RViz window opens showing the elevation map, target-tour markers, and terrain layers automatically — no extra step is needed to view it.

## 10. Success criteria

The run is working if, in order:

1. **Detections are accumulating** — confirm before starting the tour:
   ```bash
   rostopic hz /mine_camera/rgb/image_raw
   rostopic echo /mine_detection/map
   rostopic echo /detected_targets
   ```
   Use RViz **2D Nav Goal** to direct the UAV if the automatic survey is disabled (the default).
2. **The UGV tour starts and completes** — after
   ```bash
   rosservice call /ugv/start_tour "{}"
   ```
   (or letting `UAV_SURVEY=true` auto-arm it), `/ugv/tour_status` progresses and eventually reports completion; RViz's `/target_tour_markers` shows the UGV visiting each marker in turn.
3. **The elevation map is populated** — `/terrain_map` and the elevation-map RViz display show terrain rather than an empty grid.
4. (Optional grasp path) — MoveIt/GPD reports a grasp attempt per arrival. Per §11, this stage is experimental and is not a required success condition on its own.

No persistent errors/exceptions in any of the `gnome-terminal` tabs is a baseline expectation throughout.

## 11. Troubleshooting

- **`ego-planner` missing during build:** run `bash setup_uav.sh`, or clone it into `src/ego-planner/` and apply `patches/` as the setup script does.
- **No TensorRT executable / CUDA failure:** run with `UAV_DETECT_BACKEND=cpu UAV_DETECT_DEVICE=cpu`; ensure the Python dependencies above are installed.
- **No detections:** verify the RGB, depth, and camera-info topics, then inspect `/mine_camera/diagnostics` and `/mine_detection/debug_image`.
- **UGV does not move:** targets must be confirmed before `/ugv/start_tour` is called; inspect `/detected_targets`, `/ugv/tour_status`, `/state_estimation`, and `/terrain_map`.
- **Elevation map is empty:** verify the latched transform with `rosrun tf2_ros tf2_echo odom uav0/map_local` and check the UAV cloud topic `/uav0/mapping/velodyne_points_gated`.
- **Grasp stage fails or behaves inconsistently:** expected — set `UGV_GRASP=false` for a navigation-only run. The grasp stage is sensitive to final approach pose and perception of the simulated prop; treat it as an experimental integration, not a guaranteed mission outcome (see [`docs/grasp_PLAN.md`](docs/grasp_PLAN.md) for current status and known blockers).

## Acknowledgements

This workspace builds on ROS, Gazebo Classic, PX4, MAVROS, EGO-Planner, ANYbotics `elevation_mapping` / ETH `grid_map`, Clearpath Husky, Universal Robots UR5, Robotiq, MoveIt, GPD, DLIO, Ultralytics YOLO, and the Gazebo model collections included or referenced by the project.
