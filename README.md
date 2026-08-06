# Outdoor UAV–UGV Collaborative Continuous Demining System

*[中文说明](README-zh.md)*

A ROS Noetic / Gazebo Classic workspace for a simulated **UAV–UGV collaborative mine-removal mission**. A PX4 UAV surveys a mine field with a downward RGB-D camera and a YOLO11 segmentation model; confirmed locations are transformed into the UGV's egocentric frame. A Husky–UR5 UGV uses LiDAR-inertial odometry, an elevation-aware planner, and a MoveIt pipeline to execute the configured visit–grasp–return–place cycle.

> **Scope.** This is a simulation and course/research project, not a real-world explosive-ordnance-disposal system. Run it only in the supplied Gazebo/PX4 environment.

Submission material in this repository:

- Source code and configuration: the repository itself.
- Presentation: [`空地协同连续排雷系统.pptx`](空地协同连续排雷系统.pptx).
- 3 min 11 s UAV mapping and terrain-visualization excerpt (1920×1080, H.264/MP4): [`空地协同排雷演示视频.mp4`](空地协同排雷演示视频.mp4). It is supporting footage, not a substitute for the full success checks in §10.

## 1. :mortar_board:Project basic information

| | |
|---|---|
| Group No. | `3` |
| Assignment name | `空地协同户外操作系统` |
| Project name | Outdoor UAV–UGV Collaborative Continuous Demining System / 户外空地协同连续排雷系统 |
| Team members | 武天豪、赵汝堃、杜军、吴淑林 |

## 2. Function description

**Problem.** Locate simulated landmines from the air and have a ground robot visit, grasp, return, and place each confirmed target without a pre-built shared global map. The UGV handoff and ground-navigation chain is egocentric and does not require GPS; PX4 SITL may still use its simulated GNSS sensors internally.

**Main features:**
- UAV survey with a down-facing RGB-D camera and a YOLO11s-seg landmine detector.
- Multi-frame spatial confirmation to turn noisy per-frame detections into a stable mine map.
- Egocentric handoff of confirmed targets into the UGV's local `odom` frame (the UGV ground mission needs neither a shared global `map` frame nor GPS).
- UGV autonomy: LiDAR-inertial odometry, elevation-aware path planning, ordered target visitation.
- MoveIt Task Constructor grasp pipeline for the simulated mine prop. The submitted configuration uses the analytic top-down candidate server and a Gazebo fixed joint to hold the selected `landmine_*` rigid body during transport; the visible jaw motion is not claimed as a friction grasp. The GPD adapter is retained as non-acceptance research code.

**Method (brief — see [`docs/algorithm_research_references.md`](docs/algorithm_research_references.md) for the underlying algorithm survey, not reproduced here):** PX4 SITL + EGO-Planner for UAV flight, Ultralytics YOLO11s-seg for the submitted CPU detection path, DLIO for UGV odometry, a CMU-derived local planner with an ANYbotics `elevation_mapping` cost layer for UGV navigation, and MoveIt Task Constructor with analytic grasp candidates. A TensorRT adapter is retained but is not part of the tested submission configuration.

**Inputs:** the supplied `outdoor_city` Gazebo world, the five-mine field spawned by `src/uav_truth_tracker/launch/spawn_outdoor_mine_field.launch`, the shipped YOLO11s-seg weights, and runtime environment variables (§8).

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
MoveIt grasp–return–place cycle
```

The air-ground transform is established once at startup: DLIO owns `odom → base_link`, and an anchor latches `odom → uav0/map_local`. The UGV target handoff and ground mission therefore require neither a shared global `map` frame nor GPS; this statement does not disable PX4's simulated GNSS inside SITL.

## 3. Directory layout

| Path | Contents | Ownership |
|---|---|---|
| `airground_takeoff.sh` | Main integrated demo launcher and runtime defaults. | User-run; tune through environment variables rather than editing it. |
| `one_key_takeoff.sh` | Legacy UAV-only flight/mapping launcher retained for historical debugging. | Do not use it for the complete course mission; use `airground_takeoff.sh`. |
| `record_uav_map.sh` | UAV-only point-cloud recording helper retained for mapping/debug sessions. | Optional historical/debug tool; it does not start the complete course mission. |
| `stop_airground.sh` | Broadly stops the current user's ROS/PX4/Gazebo simulation stack; see the warning in §11. | User-run. |
| `setup_uav.sh` | Installs external dependencies and builds the workspace. | User-run; `PX4_DIR` may be overridden. |
| `uav_deps.repos` | Legacy EGO-Planner compatibility import manifest. | Compatibility only; the supported `setup_uav.sh` path fetches and checks the exact EGO commit listed in §5. |
| `requirements-runtime.txt` | Pinned Python packages for the verified CPU detector path. | Dependency file; do not edit for the default run. |
| `src/uav_truth_tracker/` | UAV ROS nodes, survey, mine localization/fusion, PX4 launch files, and custom messages. | Source under `scripts/`; runtime parameters under `launch/` and `config/`. |
| `src/mobile_manipulator/` | Husky–UR5 simulation, DLIO/planning integration, elevation filters, target tour, and bridge node. | Source under `scripts/`/`src/`; runtime parameters under `launch/` and `config/`; display presets under `rviz/`. |
| `src/grasp_mtc/` | MoveIt Task Constructor grasp pipeline. | Source under `scripts/`; task launch parameters under `launch/`. |
| `src/gpd_ros/` | Optional GPD messages and detector wrapper. | Messages still build without `libgpd`; the default mission does not require GPD. |
| `models/`, `worlds/` | Gazebo UAV models and shared simulation worlds. | Supplied input assets. |
| `mine_seg_v2_delivery/` | Delivered YOLO segmentation weight and model manifest. | Supplied input model; keep `weights/best.pt` in place. |
| `docs/` | Design, research, and testing notes. | Reference material. |
| `patches/` | Project patches applied to the EGO-Planner checkout. | Applied automatically by `setup_uav.sh`. |
| `build/`, `devel/` | Catkin build products. | Generated; safe to regenerate. |
| `mine_detection_output/` | Confirmed mine-map output for the current run. | Generated and overwritten at the next run. |

## 4. Runtime environment

| Component | Recorded environment or reproducible target |
|---|---|
| OS | Ubuntu 20.04.6 LTS |
| ROS | ROS Noetic |
| Simulator | Gazebo Classic 11.15.1 |
| Flight stack in the latest local runs | PX4 commit `bda25bfcc1a817f4ba559497c8ad6962f114cfd7` (`v1.17.0-alpha1-1668-gbda25bfcc1-dirty` when inspected on 6 August 2026) + MAVROS + EGO-Planner commit `bfda51284c8c1b476043255a8145ef925a3778a5` |
| Clean reproduction target installed by this repository | PX4 tag `v1.14.3`, commit `1dacb4cdef2d7145754fc788fa8dc482eed74b40`, built as `px4_sitl_default gazebo-classic` |
| Main ROS packages | MAVROS 1.20.1, Navigation 1.17.3, robot_localization 2.7.7, MoveIt 1.1.16, catkin-tools 0.9.4 |
| Build toolchain | GCC 9.4.0, CMake 3.16.3, C++17, catkin-tools 0.9.4 (`catkin build`), Python 3.8.10 |
| Verified CPU perception environment | Ultralytics 8.4.60, PyTorch 2.4.1+cpu, torchvision 0.19.1+cpu, OpenCV 4.13.0, NumPy 1.24.4, SciPy 1.10.1, PyYAML 5.3.1 |
| Optional GPU acceleration | NVIDIA driver + CUDA + TensorRT and an exported ONNX model. This is used only when explicitly selecting `UAV_DETECT_BACKEND=tensorrt`; the default CPU/Ultralytics path needs none of it. |
| Desktop session | A desktop OpenGL/Gazebo-capable session and `gnome-terminal` (the launcher opens one tab per subsystem — see §7) |
| Actual test hardware | Lenovo ThinkBook 15 G4 IAP, Intel Core i5-1240P (16 logical CPUs), 16 GB RAM, Intel integrated graphics, no NVIDIA GPU. No physical UAV/UGV is required. |
| Storage requirement | At least 10 GB free for PX4 sources/build products, ROS build products, logs, and generated maps. |

The local PX4 row records the exact checkout on the development workstation, including its local modifications; it is not portable. For a fresh evaluator machine, `setup_uav.sh` deliberately installs and checks the clean v1.14.3 reproduction target instead of pretending to reproduce a dirty checkout. There are no other OS/version combinations documented or tested; if you are not on Ubuntu 20.04 + ROS Noetic, expect to adapt package names yourself.

## 5. Dependency installation

**Prerequisite:** Ubuntu 20.04 with the ROS Noetic apt repository configured and `ros-noetic-desktop-full` installed (the course image used for testing already provides this). On another clean Ubuntu 20.04 host, install ROS Noetic first, then verify that `/opt/ros/noetic/setup.bash` exists before continuing. The project setup script intentionally stops if no ROS environment has been sourced.

Clone the repository into a catkin workspace and run the setup from the repository root:

```bash
git clone --recurse-submodules https://github.com/JunDu-cyber/Air-Ground-Collaborative-Manupulation.git learning_ws
cd learning_ws
source /opt/ros/noetic/setup.bash
bash setup_uav.sh
```

`setup_uav.sh` initializes and verifies the pinned submodules and ANYbotics helper repositories, installs the fixed Python/ROS dependencies, prepares the egocentric navigation sources, clones and patches EGO-Planner, installs or completes PX4 SITL at `${PX4_DIR:-$HOME/PX4-Autopilot}`, and builds the complete workspace. It requires `sudo` and can take considerable time. After the PX4 revision check, it records the resolved path in `${XDG_CONFIG_HOME:-$HOME/.config}/airground/env.sh` and appends one managed Gazebo setup block to `~/.bashrc`; an explicitly exported `PX4_DIR` always takes priority.

| External source | Verified revision | Installed location and handling |
|---|---|---|
| [ZJU FAST-Lab EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) | `bfda51284c8c1b476043255a8145ef925a3778a5` | `src/ego-planner/`; fetched by commit and patched automatically. No archive extraction or manual environment variable is needed. |
| [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | tag `v1.14.3`, commit `1dacb4cdef2d7145754fc788fa8dc482eed74b40` | `${PX4_DIR:-$HOME/PX4-Autopilot}`; cloned with submodules and built as `px4_sitl_default gazebo-classic`. If that directory already contains a different PX4 revision, setup stops without modifying it and tells you to choose a new `PX4_DIR`. |
| `direct_lidar_inertial_odometry` | `fc8d183f18cdcfb9bb4fc754c6d373cedc4cbd04` | Git submodule at `src/direct_lidar_inertial_odometry/`. |
| `autonomous_exploration_development_environment` | `bf0cba71365271ebff09831a05afd78578150300` | Git submodule at `src/autonomous_exploration_development_environment/`. |
| `robot_body_filter` | `b6635e9c40d0524d70e4e0059a5c6c6bb382d6f4` | Git submodule at `src/robot_body_filter/`. |
| `FAST_LIO` / `livox_ros_driver` (optional baseline) | `7cc4175de6f8ba2edf34bab02a42195b141027e9` / `3d240d5666129e1a3052e78ee8487a04b08fdda3` | Git submodules at `src/FAST_LIO/` and `src/livox_ros_driver/`; present for comparison but excluded from the default build. |
| ANYbotics `message_logger`, `kindr`, `kindr_ros` | commits recorded in `src/elevation_mapping.repos` | Imported into `src/` by `setup_uav.sh`. The project-modified `elevation_mapping` source itself is already vendored at `src/elevation_mapping/`; see its `UPSTREAM.md`. |
| YOLO11s-seg model | manifest in `mine_seg_v2_delivery/DEPLOYMENT.txt` | Already included as `mine_seg_v2_delivery/weights/best.pt`; do not move or extract it. |

The setup script installs the pinned Python perception packages from `requirements-runtime.txt`. To repair only that environment later, run:

```bash
python3 -m pip install --user -r requirements-runtime.txt
```

### Updating or repairing an existing clone

After pulling a new revision, rerun the supported setup path. It rechecks source revisions, restores dependencies and patches, runs `rosdep`, verifies/installs PX4, and rebuilds the workspace:

```bash
source /opt/ros/noetic/setup.bash
bash setup_uav.sh
source devel/setup.bash
```

ROS package dependencies declared in `package.xml` files are resolved by `rosdep` inside `setup_uav.sh`. GPD and TensorRT are retained comparison adapters but are not reproducible submission/acceptance configurations; leave `GRASP_SOURCE=analytic` and `UAV_DETECT_BACKEND=cpu` for assessment.

## 6. Pre-run configuration

- **Environment variables** (common mission switches and advanced overrides are listed in §8). The tested defaults use CPU detection, manual UAV goals, UGV navigation/grasping, elevation costs, and analytic grasp candidates. Change one subsystem at a time when diagnosing a run.
- **Model file placement.** The submission model must be present at `mine_seg_v2_delivery/weights/best.pt` (already shipped in this repo). Its manifest is [`mine_seg_v2_delivery/DEPLOYMENT.txt`](mine_seg_v2_delivery/DEPLOYMENT.txt) — task, input size (960), recommended confidence (0.7337337337 per the manifest; the launch file's own default is 0.65, see §8), and validation metrics. Use `best.pt`, not `last.pt`.
- **`PX4_DIR`.** If PX4 must be installed somewhere other than `~/PX4-Autopilot`, export `PX4_DIR` for `setup_uav.sh`. After its revision check, setup saves that directory in `${AIRGROUND_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/airground/env.sh}`; `airground_takeoff.sh` reads it automatically. An explicitly exported `PX4_DIR` still overrides the saved value.
- The default integrated entry point does not require editing personal absolute paths; its active paths are derived from the repository root or the variables in §8. Some legacy standalone launch files retain their own defaults and are not part of this procedure.

## 7. :rocket:Complete run procedure

Start the integrated simulation from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash airground_takeoff.sh
```

The default is the verified CPU detector path. `UAV_DETECT_BACKEND=tensorrt` is retained for separately configured research machines but was not tested on the acceptance hardware.

**Step 1.** From the workspace root, source ROS then the workspace overlay (in that order), as shown above.
**Step 2.** Run `airground_takeoff.sh`. This is the single entry point for the default demo; no second launch command is required.
**Step 3.** The script opens `gnome-terminal` tabs in this fixed launch order: `1_Gazebo` → `1b_Mine_Field` → `2_PX4_Spawn_UAV` → `3_UGV` → `3b_UGV_LIO` → (`3c_UGV_NAV`, `7a_MoveGroup`, `7b_Grasp` if enabled) → `4_MAVROS` → `5_AirGround_ROS` → `6_Takeoff` → `8_UAV_Detect`. Built-in delays sequence the starts; the readiness checks in Step 4, not the elapsed delay alone, determine when it is safe to operate.
**Step 4.** Wait roughly 30–60 s for the tabs to come up. `startup sequence done` means that launch commands were dispatched, not that every node is ready. In a new terminal, source both setup files and verify all of the following before sending a goal (stop each continuous `hz`/`tf2_echo` command with `Ctrl-C` after it produces valid data):

```bash
rostopic echo -n1 /mavros/state              # connected: True
rostopic echo -n1 /state_estimation           # one UGV odometry message
rosrun tf2_ros tf2_echo odom uav0/map_local   # a stable transform
rostopic hz /terrain_map                      # non-zero rate
rostopic hz /mine_camera/rgb/image_raw        # non-zero rate
missing=0
for service in /ugv/align_to_mine /grasp/execute /grasp/place /ugv/start_tour; do
  rosservice list | grep -qx "$service" || { echo "missing: $service"; missing=1; }
done
test "$missing" -eq 0
```

Then proceed to the interactive steps in §10.
**Step 5.** Inspect results per §9 (Output) and §10 (Success criteria).

The launcher starts Gazebo paused, spawns the five-mine field, brings up PX4, the UGV, DLIO, planning, the air-ground TF/elevation layer, MAVROS, the UAV takeoff bridge, and mine detection.



## 8. Input specification

Static and interactive inputs used by the submitted mission:

| Input | Format / interface | Repository path or source | Role in the default run |
|---|---|---|---|
| `outdoor_city.world` | SDFormat/XML Gazebo world | `src/mobile_manipulator/worlds/outdoor_city.world` | Default simulation scene selected by `airground_takeoff.sh`. |
| `spawn_outdoor_mine_field.launch` | ROS launch/XML | `src/uav_truth_tracker/launch/spawn_outdoor_mine_field.launch` | Spawns the supplied five-mine test field into the default world. |
| `best.pt` | PyTorch checkpoint | `mine_seg_v2_delivery/weights/best.pt` | YOLO11s-seg weights used by the default CPU detector. |
| RViz `2D Nav Goal` | Manual `geometry_msgs/PoseStamped` interaction | RViz tool → `/move_base_simple/goal` | Supplies UAV survey waypoints when `UAV_SURVEY=false` (the default). |

All required mission inputs are repository assets, simulated sensor streams, or RViz interaction. No physical UAV, UGV, LiDAR, camera, joystick, or other external peripheral is required.

Common mission switches consumed by `airground_takeoff.sh`:

| Variable | Default | Purpose |
|---|---:|---|
| `UAV_DETECT` | `true` | Start the UAV mine-detection stack. |
| `UAV_DETECT_BACKEND` | `cpu` | `cpu` uses the verified Ultralytics/PyTorch path; `tensorrt` is retained but not part of submission acceptance. |
| `UAV_DETECT_DEVICE` | `cpu` | PyTorch device passed to the detector. |
| `UAV_SURVEY` | `false` | Enable the automatic UAV survey route. |
| `UGV_NAV` | `true` | Start CMU planning and the target-tour node. |
| `UGV_GRASP` | `true` | Start MoveIt and request a grasp on target arrival. |
| `GRASP_SOURCE` | `analytic` | Grasp candidate provider. Keep `analytic` for the submitted configuration; GPD is not part of acceptance. |
| `GRASP_DETECTOR` | `color` | Wrist-camera mine detector used by the grasp pipeline. |
| `FLIGHT_H` | `2.0` | UAV flight height in metres. |
| `LOW_ALTITUDE` | `true` | Use the low-altitude EGO flight/planning configuration. |
| `START_RVIZ` | `true` | Start the elevation-map RViz configuration. |
| `UGV_COST_SOURCE` | `elevation` | UGV terrain-cost source. |
| `UGV_GLOBAL_PLANNER` | `far` | UGV global planner selection. |
| `UGV_UAV_PRIOR` | `true` | Fuse the gated UAV cloud into the UGV-centred elevation map. |
| `UGV_ELEVATION_UPDATE` | `false` | Add live UGV LiDAR updates to that elevation map when enabled. |
| `UGV_MAP_SIZE` | `120` | Elevation-map side length in metres. |
| `UGV_MAP_RES` | `0.35` | Elevation-map resolution in metres per cell. |

Path, spawn, simulation, and sequencing overrides:

| Variable | Default | Purpose |
|---|---:|---|
| `AIRGROUND_ENV_FILE` | `${XDG_CONFIG_HOME:-$HOME/.config}/airground/env.sh` | Managed file used to remember the PX4 directory verified by setup. Override only when maintaining separate installations. |
| `PX4_DIR` | `$HOME/PX4-Autopilot` | PX4 source/build directory. |
| `EGO_WS`, `UGV_WS` | repository root | Workspace overlays used by UAV and UGV launch tabs. |
| `UGV_WORLD` | `src/mobile_manipulator/worlds/outdoor_city.world` | Default shared Gazebo world. |
| `GAZEBO_WORLD` | value of `UGV_WORLD` | World passed to `gazebo_ros`. |
| `LIDAR_SDF` | `models/iris_depth_camera_lidar_terrain/model.sdf` | PX4 UAV model SDF. |
| `MAVROS_PX4_LAUNCH` | `/opt/ros/noetic/share/mavros/launch/px4.launch` | MAVROS launch file. |
| `UAV_POINTS_MAP_DIR` | `$HOME/pointcloud_maps` | Saved UAV point-cloud directory. |
| `SPAWN_X`, `SPAWN_Y`, `SPAWN_Z` | `0.0`, `-18.0`, `1.5` | UAV Gazebo spawn position in metres. |
| `SPAWN_YAW` | `1.5707963` | UAV spawn yaw in radians. |
| `MAP_LOCAL_Z` | `0.0` | Vertical offset of the UAV-local origin in `odom`; normally leave at zero. |
| `PX4_GAZEBO_GUI` | `true` | Show the Gazebo client. |
| `PX4_SIM_SPEED_FACTOR` | `1` | PX4 simulation speed factor. |
| `PHYSICS_STEP`, `PHYSICS_RATE` | `0.005`, `200.0` | Gazebo time step and maximum update rate. |
| `AG_ENABLE_GATE` | derived | Override the altitude gate; normally derived from elevation-map ownership and `UGV_UAV_PRIOR`. |
| `GAZEBO_LOAD_WAIT`, `PX4_WAIT`, `UGV_SPAWN_WAIT` | `10`, `8`, `6` | Startup delays in seconds before the next launch tab is dispatched. |
| `MAVROS_WAIT`, `ROS_WAIT` | `5`, `20` | MAVROS and ROS-layer startup delays in seconds. |

These are all environment-variable overrides read by the integrated launcher; subsystem launch files expose additional direct `roslaunch` arguments. The detector launch's own `confidence` arg is **0.65** (`src/uav_truth_tracker/launch/uav_mine_detection.launch`). Change it only in a standalone detector evaluation; the integrated default uses 0.65.

Runtime topic inputs the pipeline consumes:

| Topic | Type | Meaning |
|---|---|---|
| `/mine_camera/rgb/image_raw` | `sensor_msgs/Image` | UAV down-facing RGB image. |
| `/mine_camera/depth/image_raw` | `sensor_msgs/Image` | Time-aligned UAV depth image. |
| `/mine_camera/rgb/camera_info` | `sensor_msgs/CameraInfo` | RGB optical calibration used for 3-D projection. |
| RViz `2D Nav Goal` → `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Optional manual UAV goal when `UAV_SURVEY=false` (default). The UGV does not consume this topic. |
| `/ugv/start_tour` | `std_srvs/Trigger` (service call) | Freeze the currently collected targets and begin the UGV tour. With the five-mine field, wait for `confirmed_count: 5` first; detections received after this call are intentionally ignored. |

With the default manual-UAV mode, use RViz's fixed `odom` frame and click near the five supplied mine-field positions below. The goal bridge converts the click into the UAV-local frame and applies `FLIGHT_H`; the coordinates are operator waypoints for this supplied world, not detector ground truth used by the algorithm.

| Suggested click order | `odom` XY (m) |
|---:|---:|
| 1 | `(1.4, 0.9)` |
| 2 | `(2.3, -1.1)` |
| 3 | `(5.5, 1.4)` |
| 4 | `(8.5, -1.4)` |
| 5 | `(11.5, 1.3)` |

Wait at each area until its marker turns confirmed, then continue. Do not call `/ugv/start_tour` until the YAML reports all five confirmations.

## 9. Output specification

Generated files and their overwrite behavior:

| Path | Behavior |
|---|---|
| `logs/setup_build.log` | Latest `setup_uav.sh` catkin build log; overwritten by the next setup build and ignored by Git. |
| `mine_detection_output/mine_map.yaml` | Atomic snapshot of candidates and confirmed mines. Reset at integrated detector startup and overwritten when the map revision changes. |
| `~/pointcloud_maps/uav_points_map_latest.pcd` | Latest accumulated UAV cloud in `odom`; overwritten every 30 s and on clean shutdown. |
| `~/pointcloud_maps/uav_points_map_YYYYMMDD_HHMMSS.pcd` | Timestamped snapshot created every 30 s; files accumulate until manually removed. |
| `~/trajectory_logs/flight_trajectory_YYYYMMDD_HHMMSS.csv` | A new UAV actual/desired trajectory log for each mapping-node start. |

These are generated outputs and are not source-controlled. Copy wanted results elsewhere before another run or cleanup. The legacy GPS/UTM `uav_map_origin.yaml` sidecar is deliberately disabled in the default egocentric run because its cloud is recorded in `odom`, not the legacy UAV `map` frame.

Live outputs are ROS topics rather than files:

| Interface | Type | Meaning |
|---|---|---|
| `/mine_detection/raw` | `uav_truth_tracker/MineDetectionArray` | Per-frame depth-localized detections. |
| `/mine_detection/map` | `uav_truth_tracker/MineMap` | Associated and confirmed mine hypotheses. |
| `/detected_targets` | `mobile_manipulator/WorldTarget` | Confirmed mine locations handed to the UGV. |
| `/ugv/tour_status` | `std_msgs/String` | Target-tour state. |
| `/ugv/goal` | `geometry_msgs/PoseStamped` | Current UGV navigation goal in `odom`. |
| `/elevation_mapping/elevation_map_postprocessed` | `grid_map_msgs/GridMap` | Fused elevation and post-processed terrain layers. |
| `/terrain_map` | `sensor_msgs/PointCloud2` | Traversability/cost cloud consumed by UGV planning. |
| `/target_tour_markers` | `visualization_msgs/MarkerArray` | Pending/current/processed tour targets. |
| `/uav0/mapping/points_world` | `sensor_msgs/PointCloud2` | UAV cloud transformed into the shared `odom` frame. |
| `/mine_detection/debug_image` | `sensor_msgs/Image` | Timestamped detector visualization. |
| `/mine_camera/diagnostics` | `diagnostic_msgs/DiagnosticArray` | RGB-D rate, age, synchronization, encoding, and TF health. |
| `/uav/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | UAV state covariance used by elevation mapping. |

Custom UAV messages are defined under [`src/uav_truth_tracker/msg`](src/uav_truth_tracker/msg); the UGV handoff message is [`WorldTarget.msg`](src/mobile_manipulator/msg/WorldTarget.msg).

Visualization: with `START_RVIZ=true` (the default), RViz opens with the fused/raw elevation maps, `/terrain_map`, `/target_tour_markers`, UAV cloud, robot model, and `/mine_detection/debug_image` already configured.

## 10. Success criteria

The run is working if, in order:

1. **All five mines are confirmed before the target set is frozen** — confirm:
   ```bash
   rostopic hz /mine_camera/rgb/image_raw
   rostopic echo -n1 /mine_detection/map
   grep '^confirmed_count: 5$' mine_detection_output/mine_map.yaml
   ```
   The map message must contain five entries with `confirmed: true`, and the YAML check must print `confirmed_count: 5`. Use RViz **2D Nav Goal** to direct the UAV if automatic survey is disabled (the default). Do not start the tour early: the tour intentionally freezes its input set and ignores later detections.
2. **The UGV tour starts with all five targets** — call
   ```bash
   rosservice call /ugv/start_tour "{}"
   rostopic echo /ugv/tour_status
   ```
   The returned message and `/ugv/tour_status` must show `total: 5`; RViz's `/target_tour_markers` shows the current and pending targets.
3. **The elevation map is populated** — `/terrain_map` and the elevation-map RViz display show terrain rather than an empty grid.
4. **Every mine completes the simulated grasp cycle** — with `UGV_GRASP=true`, record one successful `ALIGN → GRASP → NAV_HOME → PLACE` sequence and a `place OK` result for each of the five targets. During GRASP, the wrist-camera target drives the approach; the jaws close visually and the executor selects the nearest matching `landmine`/`landmine_*` rigid body only for a Gazebo fixed-joint transport lock. That same model remains locked until `/grasp/place` detaches it. Only after all five successful PLACE events may terminal `DONE` be accepted. `DONE` by itself is insufficient because the tour can advance after a navigation/alignment/grasp failure. `UGV_GRASP=false` is a navigation-only diagnostic mode and must be identified as such when reporting results.

No persistent errors/exceptions in any `gnome-terminal` tab is a baseline expectation throughout. The supplied video is an UAV mapping/terrain-visualization excerpt; use the live checks above for full mission acceptance.

## 11. Stopping the program

From a new terminal in the repository root, run:

```bash
bash stop_airground.sh
```

**Do not run another ROS/PX4 session under the same user at the same time.** The launcher and stop script intentionally clear stale simulation state broadly: `stop_airground.sh` kills all nodes on the current ROS master and may terminate other Gazebo, ROS, RViz, PX4, or MAVROS processes owned by the user. Save unrelated work first.

The script requests a ROS shutdown, then terminates matching Gazebo, PX4, MAVROS, planner, detector, and grasp processes. It does not delete maps, models, source files, or build products. Shutdown is complete when Gazebo closes and the following command prints no matching process:

```bash
pgrep -af 'gzserver|gzclient|rosmaster|px4|mavros_node'
```

If a prior run ended abnormally, run `bash stop_airground.sh` once before starting again.

## 12. Troubleshooting

- **`ego-planner` missing during build:** run `bash setup_uav.sh`, or clone it into `src/ego-planner/` and apply `patches/` as the setup script does.
- **No TensorRT executable / CUDA failure:** run with `UAV_DETECT_BACKEND=cpu UAV_DETECT_DEVICE=cpu`; ensure the Python dependencies above are installed.
- **No detections:** verify the RGB, depth, and camera-info topics, then inspect `/mine_camera/diagnostics` and `/mine_detection/debug_image`.
- **UGV does not move:** targets must be confirmed before `/ugv/start_tour` is called; inspect `/detected_targets`, `/ugv/tour_status`, `/state_estimation`, and `/terrain_map`.
- **Elevation map is empty:** verify the latched transform with `rosrun tf2_ros tf2_echo odom uav0/map_local` and check the UAV cloud topic `/uav0/mapping/velodyne_points_gated`.
- **Grasp stage fails or behaves inconsistently:** set `UGV_GRASP=false` for a navigation-only diagnostic run. For a grasp-enabled run, inspect the `physical mine selected: landmine_*` and `welded ...` lines, `/gazebo/model_states`, `/grasp/execute`, and `/grasp/place`. [`docs/grasp_PLAN.md`](docs/grasp_PLAN.md) is a historical investigation log, not the current acceptance status.
- **CMake warns that `libgpd` is absent:** this is expected for the reproducible default. Keep `GRASP_SOURCE=analytic`; the package still generates its ROS messages and the workspace continues to build. GPD is an optional comparison backend, not a default dependency.

## Acknowledgements

This workspace builds on ROS, Gazebo Classic, PX4, MAVROS, EGO-Planner, ANYbotics `elevation_mapping` / ETH `grid_map`, Clearpath Husky, Universal Robots UR5, Robotiq, MoveIt, GPD, DLIO, Ultralytics YOLO, and the Gazebo model collections included or referenced by the project.
