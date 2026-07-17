# Air-Ground Collaborative Landmine Survey and Response

A ROS Noetic / Gazebo Classic research workspace for a simulated **UAV–UGV collaborative mission**. A PX4 UAV surveys a mine field with a downward RGB-D camera and a YOLO11 segmentation model; confirmed locations are transformed into the UGV's egocentric frame. A Husky–UR5 UGV uses LiDAR-inertial odometry, an elevation-aware planner, and (optionally) a MoveIt/GPD grasp pipeline to visit those targets.

> **Scope.** This is a simulation and course/research project, not a real-world explosive-ordnance-disposal system. Run it only in the supplied Gazebo/PX4 environment.

![Husky–UR5 grasp simulation](assets/grasp.gif)

## Mission pipeline

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

## Highlights

- **UAV survey:** PX4 SITL, MAVROS, EGO-Planner, LiDAR terrain sensing, and a down-facing RGB-D mine camera.
- **Mine perception:** YOLO11s-seg instance segmentation, depth-centroid localization, TF-aware projection, diagnostics, and multi-frame spatial confirmation.
- **Shared terrain representation:** UAV and UGV clouds can be fused into an ANYbotics `elevation_mapping` map with a traversability layer for the UGV planner.
- **UGV autonomy:** Husky base, UR5 arm, DLIO LiDAR-inertial odometry, CMU planner, confirmed-target collection, deduplication, and ordered visitation.
- **Manipulation:** optional MoveIt Task Constructor / GPD pick pipeline for the simulated mine prop.

## Requirements

| Component | Tested target |
|---|---|
| OS / ROS | Ubuntu 20.04, ROS Noetic |
| Simulator | Gazebo Classic 11 |
| Flight stack | PX4 v1.14.3 SITL + MAVROS + EGO-Planner |
| Build | `catkin_make` or `catkin build`, C++17, Python 3 |
| Perception | Python packages `ultralytics`, `torch`, `opencv-python`, `numpy`, `PyYAML` |
| Optional acceleration | NVIDIA driver, CUDA, TensorRT, and an exported ONNX model |

The full simulator also needs a desktop OpenGL/Gazebo session and `gnome-terminal`; `airground_takeoff.sh` opens one tab per major subsystem. The default detection backend in that script is TensorRT, so machines without a working NVIDIA/TensorRT installation should explicitly use the CPU fallback shown below.

## Installation

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

## Run the submission demo

Start the integrated simulation from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# CPU/Ultralytics fallback (recommended when TensorRT is unavailable)
UAV_DETECT_BACKEND=cpu UAV_DETECT_DEVICE=cpu bash airground_takeoff.sh
```

On a CUDA/TensorRT machine, omit those two environment variables to use the script defaults. The launcher starts Gazebo paused, spawns the five-mine field, brings up PX4, the UGV, DLIO, planning, the air-ground TF/elevation layer, MAVROS, the UAV takeoff bridge, and mine detection.

### Expected operation

1. Wait for the UAV to arm and take off. Use RViz **2D Nav Goal** to direct the UAV if automatic survey is disabled (the default).
2. Confirm that detections are accumulating:
   ```bash
   rostopic hz /mine_camera/rgb/image_raw
   rostopic echo /mine_detection/map
   rostopic echo /detected_targets
   ```
3. After the desired targets have been confirmed, start the UGV tour:
   ```bash
   rosservice call /ugv/start_tour "{}"
   ```
4. Observe `/ugv/tour_status`, `/target_tour_markers`, `/terrain_map`, and the elevation-map RViz display.

To let the UAV execute its lawnmower survey automatically, set `UAV_SURVEY=true`. The UGV will arm its auto-start when `/mine_survey/status` becomes `COMPLETE`.

```bash
UAV_DETECT_BACKEND=cpu UAV_DETECT_DEVICE=cpu UAV_SURVEY=true bash airground_takeoff.sh
```

### Useful runtime switches

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

For a navigation-only run, use `UGV_GRASP=false`. The grasp stage remains sensitive to final approach pose and perception of the simulated prop; it should be treated as an experimental integration rather than a guaranteed mission outcome.

## Model and mine map outputs

The submission model is in `mine_seg_v2_delivery/weights/best.pt`. Its task, input size, recommended threshold, and recorded validation metrics are listed in [`mine_seg_v2_delivery/DEPLOYMENT.txt`](mine_seg_v2_delivery/DEPLOYMENT.txt). The detector launch defaults to a 0.65 confidence threshold; tune `confidence:=...` only when evaluating a different operating point.

Each mission writes the confirmed mine map to:

```text
mine_detection_output/mine_map.yaml
```

This file is generated output and is not source-controlled.

## Key ROS interfaces

| Interface | Type | Meaning |
|---|---|---|
| `/mine_detection/raw` | `uav_truth_tracker/MineDetectionArray` | Per-frame depth-localized detections. |
| `/mine_detection/map` | `uav_truth_tracker/MineMap` | Associated and confirmed mine hypotheses. |
| `/detected_targets` | `mobile_manipulator/WorldTarget` | Confirmed mine locations handed to the UGV. |
| `/ugv/start_tour` | `std_srvs/Trigger` | Freeze collected targets and begin the UGV tour. |
| `/ugv/tour_status` | `std_msgs/String` | Target-tour state. |
| `/ugv/goal` | `geometry_msgs/PoseStamped` | Current UGV navigation goal in `odom`. |
| `/elevation_mapping/elevation_map_postprocessed` | `grid_map_msgs/GridMap` | Fused elevation and post-processed terrain layers. |
| `/uav/pose_cov` | pose-with-covariance | UAV state covariance used by elevation mapping. |

Custom UAV messages are defined under [`src/uav_truth_tracker/msg`](src/uav_truth_tracker/msg); the UGV handoff message is [`WorldTarget.msg`](src/mobile_manipulator/msg/WorldTarget.msg).

## Repository guide

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

## Standalone workflows

The repository also retains earlier simulation workflows:

```bash
# PX4/EGO mapping workflow
bash one_key_takeoff.sh

# Replay a saved UAV point cloud into elevation_mapping
roslaunch mobile_manipulator pcd_to_elevation.launch \
  pcd_file:=~/pointcloud_maps/uav_points_map_latest.pcd

# Spawn the outdoor UGV world without the integrated UAV mission
roslaunch mobile_manipulator spawn_outdoor_city.launch
```

## Troubleshooting

- **`ego-planner` missing during build:** run `bash setup_uav.sh`, or clone it into `src/ego-planner/` and apply `patches/` as the setup script does.
- **No TensorRT executable / CUDA failure:** run with `UAV_DETECT_BACKEND=cpu UAV_DETECT_DEVICE=cpu`; ensure the Python dependencies above are installed.
- **No detections:** verify the RGB, depth, and camera-info topics, then inspect `/mine_camera/diagnostics` and `/mine_detection/debug_image`.
- **UGV does not move:** targets must be confirmed before `/ugv/start_tour` is called; inspect `/detected_targets`, `/ugv/tour_status`, `/state_estimation`, and `/terrain_map`.
- **Elevation map is empty:** verify the latched transform with `rosrun tf2_ros tf2_echo odom uav0/map_local` and check the UAV cloud topic `/uav0/mapping/velodyne_points_gated`.

## Acknowledgements

This workspace builds on ROS, Gazebo Classic, PX4, MAVROS, EGO-Planner, ANYbotics `elevation_mapping` / ETH `grid_map`, Clearpath Husky, Universal Robots UR5, Robotiq, MoveIt, GPD, DLIO, Ultralytics YOLO, and the Gazebo model collections included or referenced by the project.
