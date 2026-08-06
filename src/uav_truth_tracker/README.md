# UAV Truth Tracker

> **Historical subsystem notes.** Most of this document records the retained
> dual-UAV tracking/interception experiments and their standalone launch files;
> it is not the run guide for the current course demining mission. For the
> submitted UAV–UGV workflow, dependencies, launch order, inputs, outputs, and
> acceptance checks, use the repository [README](../../README.md) and start only
> through [`airground_takeoff.sh`](../../airground_takeoff.sh).

This package contains the staged dual-UAV truth-tracking and intercept loop for the PX4 + MAVROS + Gazebo Classic simulation.

The prediction-focused Stage 5 path is stable: `/uav1` flies a 3D target trajectory, `/target_state_estimator_node.py` predicts an intercept point from observations only, and `/uav0` uses velocity guidance to intercept. Stage 6 adds a Gazebo Classic 32-channel Velodyne-style 3D LiDAR observation path while keeping ego_planner disabled.

The dual-UAV launch keeps `iris` as the PX4 vehicle for both UAVs. For `/uav0`, the spawned SDF is generated from this package's `iris_depth_camera_visible.sdf.jinja` template, which is the PX4 Iris model plus the official Gazebo Classic depth camera internals: the `realsense_camera/meshes/realsense.dae` visual and the standard `libgazebo_ros_openni_kinect.so` plugin settings. `/uav1` uses the standard jinja-generated `iris` model for its second MAVLink port set.

For Stage 6 LiDAR tests, `/uav0` can instead use `models/iris_depth_camera_lidar/iris_depth_camera_lidar.sdf.jinja`. This keeps the existing depth camera and adds a 32-channel 360 degree ray sensor on top of the vehicle at `velodyne_link` offset `z=0.06m`. Gazebo publishes raw `sensor_msgs/PointCloud` on `/uav0/velodyne_points_raw`; `pointcloud_to_pointcloud2_relay.py` republishes the required `sensor_msgs/PointCloud2` topic `/uav0/velodyne_points`.

## Current Progress

- Stage 1 truth tracking is available: `/truth_tracker_node` predicts `/uav1` future pose and publishes `/uav0/tracker/goal`.
- Stage 2 position-setpoint chase is available for baseline tests, but it can lag moving targets because `/uav0` follows a moving pose goal through bounded position setpoints.
- Stage 3 velocity guidance remains available: `/uav0_velocity_guidance_node.py` publishes `/uav0/mavros/setpoint_velocity/cmd_vel` and can still run its legacy internal prediction with `target_input_mode:=internal`.
- Stage 5 3D target tests are working with `intercept_velocity_chase.launch`: the default target is `circle_z_sine`, prediction uses observed `/uav1` odometry, and capture is checked in 3D.
- The current prediction-focused path is `intercept_estimator_velocity_chase.launch`: `/target_state_estimator_node.py` owns the online target prediction, and `/uav0_velocity_guidance_node.py` consumes `/target_estimator/intercept_point`.
- The latest intercept fix computes the time-to-go by scanning reachable intercept times with separate `/uav0` XY and Z speed assumptions, then commands Z velocity from both `z_to_intercept / t_go` and current target-height error. This prevents `/uav0` from arriving horizontally while still below `/uav1`.
- The estimator velocity chase uses observation-only prediction by default: `estimator_model:=kalman_cv`, fitted/smoothed velocity from recent observations, `uav1_speed:=0.6` as the conservative launch default, `assumed_chaser_speed_xy:=1.5`, `max_prediction_time:=2.5`, `velocity_fit_window:=0.5`, `velocity_smoothing_alpha:=0.65`, `kf_velocity_blend:=0.55`, `kf_process_noise_vel:=1.0`, `face_guidance_before_move:=true`, `yaw_gate_xy_scale:=0.4`, `yaw_rate_max:=1.2`, `yaw_align_threshold:=0.35`, `yaw_align_min_distance:=1.2`, `allow_vertical_during_yaw_align:=true`, `Kp_z:=0.75`, `vxy_max:=2.2`, `vz_max:=0.8`, `axy_max:=1.8`, `az_max:=1.0`, `capture_distance:=0.35`, `hard_capture_distance:=0.25`, `hold_time:=0.15`, `capture_slowdown_distance:=0.35`, and `stop_on_capture_success:=true`.
- Evaluator CSV logging is enabled by default in `intercept_estimator_velocity_chase.launch` with `start_evaluator:=true`; logs are written to `~/uav_intercept_logs`.
- Latest high-speed reference tests use `target_mode:=circle_z_sine`, `uav1_speed:=1.0`, `z_amplitude:=0.5`, and formal models only. Both `kalman_cv` and `kalman_ca` can complete capture under the estimator velocity chase launch. The default `max_prediction_time:=2.5` is a compromise between circle lead distance and figure-8 turn overshoot; sharper figure-8 tests may still override `max_prediction_time:=2.0`.
- CV remains the default formal model because it is steadier across repeated runs. CA is retained as a formal comparison model with conservative acceleration use through `ca_accel_prediction_horizon:=1.0`.
- The latest figure-8 CA logs show repeated 0.5m close passes but short 0.35m windows because Z error remains around `0.17m` to `0.30m` at the closest samples. The main launch now keeps horizontal and prediction parameters unchanged but raises Z correction to `Kp_z:=0.75` and `az_max:=1.0`.
- The latest capture-gate tuning issue was not estimator failure: the pursuer reached about `0.28m` at an earlier pass but the previous capture gate was too tight. The estimator launch now uses `capture_distance:=0.35`, `hard_capture_distance:=0.25`, `hold_time:=0.15`, and `capture_slowdown_distance:=0.35`; the capture node treats `distance <= capture_distance` as inside the normal gate and `distance <= hard_capture_distance` as immediate success.

## Project Status Summary

- Simulation stack: dual PX4/MAVROS/Gazebo Classic UAV setup is in place, with `/uav0` as the pursuer and `/uav1` as the moving target.
- Control path: the active path is estimator-driven velocity chase, not position-goal following. `/uav0_velocity_guidance_node.py` publishes velocity setpoints to `/uav0/mavros/setpoint_velocity/cmd_vel`.
- Prediction path: `/target_state_estimator_node.py` publishes `/target_estimator/state`, `/target_estimator/intercept_point`, and `/target_estimator/t_go`. Formal prediction models must use only current and past observations; target trajectory parameters are not allowed in the online estimator.
- Active default model: `kalman_cv`, with retuned observation velocity fitting and Kalman velocity blending to reduce long straight-line extrapolation during target turns.
- Logging and analysis: `chase_evaluator_node.py` starts by default in the estimator launch and writes CSV logs for `analyze_prediction_log.py`.
- Current performance: recent runs can intercept `circle_z_sine` at `uav1_speed:=1.0`, and `figure8_3d` works with the conservative `kalman_ca` setup after the final Z-guidance correction. The tuning stage is considered converged; do not keep changing parameters unless repeated logs show the same failure pattern.
- Stage 6 LiDAR detection now uses the standard LiDAR path: `/uav0/velodyne_points -> target_lidar_detector_node.py -> /target_observation/pose -> target_state_estimator_node.py`. `/uav1` odom is not used by the online detector.
- Current Stage 6 LiDAR checkpoint: `/uav0` uses the 32-channel sensor mounted at `z=0.06m`, the detector publishes `/target_observation/pose`, the estimator reaches `TRACKING`, and `/uav0_velocity_guidance_node.py` publishes nonzero velocity commands. LiDAR chase now keeps `stop_on_capture_success=false` and `latch_capture_success=false`; this avoids the previous `reason_if_zero_velocity=capture_success_stop` lock that stopped `/uav0` immediately after evaluator capture.

## Files

- `scripts/truth_tracker_node.py`: subscribes to target odometry and publishes the predicted intercept point.
- `scripts/mavros_hover_node.py`: optional namespaced setpoint publisher for OFFBOARD/arm hover checks.
- `scripts/uav1_motion_node.py`: publishes `/uav1/mavros/setpoint_position/local` circle or line target motion setpoints.
- `scripts/uav0_goal_follower_node.py`: follows `/uav0/tracker/goal` through bounded `/uav0/mavros/setpoint_position/local` steps.
- `scripts/uav0_velocity_guidance_node.py`: follows the observed target with velocity guidance on `/uav0/mavros/setpoint_velocity/cmd_vel`.
- `scripts/target_state_estimator_node.py`: estimates target state and predicted intercept point from observation history.
- `scripts/chase_evaluator_node.py`: records online chase and estimator data to CSV.
- `scripts/target_depth_detector_node.py`: detects the target from `/iris0/camera/depth/points` or depth image and publishes `/target_visual/pose`.
- `scripts/target_lidar_detector_node.py`: Stage 6 32-channel LiDAR detector; clusters `/uav0/velodyne_points` and publishes `/target_observation/pose` without `/uav1` odom.
- `scripts/target_sensor_fusion_node.py`: publishes priority-fused `/target_fused/pose` with visual primary and LiDAR fallback.
- `scripts/collect_uav_yolo_dataset_node.py`: offline-only RGB collector and YOLO label generator; `/uav1` odom is used only to create training labels.
- `scripts/collect_uav_yolo_multiworld.sh`: runs repeated offline truth-follow collection sessions across multiple Gazebo Classic worlds.
- `scripts/target_yolo_detector_node.py`: online one-class UAV detector; combines RGB detections with aligned depth and publishes `/target_semantic/pose` without `/uav1` odom.
- `scripts/train_uav_yolo.py`: trains the one-class UAV detector from a collected YOLO-format dataset.
- `scripts/target_reacquire_manager_node.py`: classifies visual-loss states and publishes `/target_reacquire/yaw_target` for optional yaw reacquisition.
- `scripts/pointcloud_to_pointcloud2_relay.py`: converts Gazebo block-laser `PointCloud` output to the Stage 6 `PointCloud2` topic.
- `scripts/analyze_prediction_log.py`: computes offline prediction and detection error plots from evaluator CSV logs.
- `scripts/run_estimator_model_tests.sh`: runs the estimator model comparison sequence and writes per-model CSV logs.
- `scripts/run_kalman_tuning_sweep.sh`: runs a small Kalman CV/CA reachability tuning sweep and prints candidate Kalman parameter commands.
- `scripts/summarize_prediction_results.py`: summarizes a log directory into `prediction_model_summary.csv`.
- `scripts/capture_decision_node.py`: checks `/uav0` to `/uav1` distance and publishes capture success plus a marker.
- `launch/intercept_truth_tracking.launch`: starts only the tracker node.
- `launch/intercept_truth_chase.launch`: starts the Stage 2 truth chase loop without ego_planner or LiDAR.
- `launch/intercept_velocity_chase.launch`: starts the Stage 3/5 velocity-guided intercept loop.
- `launch/intercept_estimator_velocity_chase.launch`: starts the prediction-focused estimator plus velocity-guided intercept loop.
- `launch/target_lidar_detector_debug.launch`: starts only the LiDAR detector for point-cloud detection debug.
- `launch/target_fusion_debug.launch`: starts depth detector, LiDAR detector, and priority fusion only; it does not start estimator or velocity guidance.
- `launch/target_depth_detector_debug.launch`: starts only the depth-camera detector path for visual target detection debug.
- `launch/intercept_depth_estimator_chase.launch`: uses `/target_visual/pose` as the estimator observation for depth-only intercept tests.
- `launch/intercept_lidar_estimator_chase.launch`: starts the Stage 6 LiDAR observation, estimator, velocity guidance, capture, and evaluator chain.
- `launch/intercept_depth_lidar_fusion_chase.launch`: starts visual-primary/LiDAR-fallback fusion and feeds `/target_fused/pose` into the estimator.
- `launch/collect_uav_yolo_lidar_follow.launch`: follows `/uav1` with the LiDAR baseline while collecting RGB images and offline training labels.
- `launch/collect_uav_yolo_truth_follow.launch`: offline-only collection launch that uses `/uav1` truth so `/uav0` can keep the target in view while generating labels.
- `launch/target_yolo_detector_debug.launch`: starts the online semantic UAV detector for topic and debug-image checks.
- `launch/intercept_yolo_lidar_fusion_chase.launch`: starts YOLO-confirmed LiDAR fusion and prevents unconfirmed obstacle clusters from becoming chase targets.
- `launch/stage6_lidar_intercept_onekey.launch`: starts dual-UAV SITL with the `/uav0` LiDAR model plus the full Stage 6 LiDAR intercept chain.
- `launch/dual_hover_check.launch`: starts hover setpoint streams for `/uav0` and `/uav1`.
- `launch/dual_uav_mavros_sitl.launch`: starts two namespaced PX4/MAVROS vehicles, `/uav0` and `/uav1`.
- `models/iris_depth_camera_visible`: local Iris template that embeds the official depth camera mesh and plugin settings into `/uav0`.
- `models/iris_depth_camera_lidar`: local Iris template that keeps the depth camera and adds the 32-channel Velodyne-style 3D LiDAR.

## Build

From `$HOME/ego_ws`:

```bash
catkin_make
source devel/setup.bash
```

## Start Dual-UAV SITL

In a new terminal:

```bash
source /opt/ros/noetic/setup.bash
source $HOME/ego_ws/devel/setup.bash
cd $HOME/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch uav_truth_tracker dual_uav_mavros_sitl.launch
```

Optional world override:

```bash
roslaunch uav_truth_tracker dual_uav_mavros_sitl.launch world:=$HOME/.gazebo/worlds/rugged_mountain.world
```

## Check MAVROS Odometry

```bash
rostopic list | grep '/uav[01]/mavros/local_position/odom'
rostopic echo -n 1 /uav0/mavros/local_position/odom
rostopic echo -n 1 /uav1/mavros/local_position/odom
```

Expected topics:

- `/uav0/mavros/local_position/odom`
- `/uav1/mavros/local_position/odom`

The official depth camera keeps its default camera name and frame. In this dual-UAV launch the Gazebo model name for `/uav0` is `iris0`, so Gazebo ROS publishes the camera topics under `/iris0/camera`:

```bash
rostopic list | grep -E '^/iris0/camera|depth|points'
rostopic echo -n 1 /iris0/camera/depth/image_raw/header
rostopic echo -n 1 /iris0/camera/depth/points/header
```

Expected official frame id: `camera_link`.

This launch also publishes `camera_link` TF from `/uav0/mavros/local_position/odom`, using the same camera offset as the SDF (`x=0.1, y=0, z=0.0`). In RViz, set `Fixed Frame` to `map`, then add:

- `Image`: `/iris0/camera/depth/image_raw`
- `PointCloud2`: `/iris0/camera/depth/points`

If you only want to inspect raw camera data without TF, `rqt_image_view /iris0/camera/depth/image_raw` is the simplest check.

## Stage 6A: Depth Camera Target Detection

LiDAR returns on a small UAV can still be sparse even with 32 vertical channels, so the recommended sensing path uses the front depth camera as the primary detector. The first depth detector is intentionally simple: it reads `/iris0/camera/depth/points` by default, transforms points to `map`, filters by range and altitude, clusters target-sized components, and publishes only position observations:

```text
/iris0/camera/depth/points
        -> target_depth_detector_node.py
        -> /target_visual/pose
        -> /target_visual/pose_cov
        -> /target_visual/confidence
        -> /target_visual/valid
```

It also supports `detector_input_mode:=depth_image`, using `/iris0/camera/depth/image_raw` plus `/iris0/camera/depth/camera_info`, but `depth_points` is the default because it avoids hand-written projection and optical-frame mistakes. The detector does not subscribe to `/uav1/mavros/local_position/odom`.

Check actual camera topics after SITL is running:

```bash
rostopic list | grep -E 'iris0/camera|camera|depth|rgb'
rostopic hz /iris0/camera/depth/image_raw
rostopic hz /iris0/camera/depth/points
rostopic echo -n 1 /iris0/camera/depth/image_raw/header
rostopic echo -n 1 /iris0/camera/depth/points/header
rostopic echo -n 1 /iris0/camera/depth/camera_info/header
```

Run detector-only debug:

```bash
roslaunch uav_truth_tracker target_depth_detector_debug.launch
rostopic hz /target_visual/pose
rostopic echo -n 1 /target_visual/pose
rostopic echo -n 1 /target_visual/confidence
rostopic echo -n 1 /target_visual/valid
```

RViz displays:

- `MarkerArray`: `/uav_models/markers`
- `PointCloud2`: `/iris0/camera/depth/points`
- `MarkerArray`: `/target_visual/marker`
- `Pose`: `/target_visual/pose`

Acceptance: when UAV1 is in the front camera view, `/target_visual/pose` should be stable and close to the orange UAV1 marker. When UAV1 leaves the camera view, `/target_visual/valid` should become `false`. Rear targets are not expected to be detected by the depth camera.

## Stage 6B: Depth-Only Intercept

Use depth-camera observations directly as the estimator input before enabling fusion:

```bash
roslaunch uav_truth_tracker intercept_depth_estimator_chase.launch \
  target_mode:=circle_z_sine \
  uav1_speed:=0.6 \
  z_amplitude:=0.5 \
  estimator_model:=kalman_cv
```

Data flow:

```text
/target_visual/pose
        -> target_state_estimator_node.py
        -> /target_estimator/state
        -> /target_estimator/intercept_point
        -> uav0_velocity_guidance_node.py
```

If this path is not stable while the target is visible, fix the depth detector first instead of tuning Kalman CV/CA or adding fusion complexity.

## Stage 6C: Depth + LiDAR Fusion Intercept

This path is the current depth-camera + LiDAR fusion experiment. It keeps the
Stage 5 Kalman CV/CA estimator and `/uav0_velocity_guidance_node.py`; only the
target observation input is replaced by `/target_fused/pose`.

Data flow:

```text
/iris0/camera/depth/points
        -> target_depth_detector_node.py
        -> /target_visual/pose, /target_visual/valid, /target_visual/confidence

/uav0/velodyne_points
        -> target_lidar_detector_node.py
        -> /target_observation/pose, /target_observation/valid, /target_observation/confidence

/target_visual/*
/target_observation/*
        -> target_sensor_fusion_node.py
        -> /target_fused/pose, /target_fused/valid, /target_fused/confidence, /target_fused/source
        -> target_state_estimator_node.py
        -> /target_estimator/state
        -> /target_estimator/intercept_point
        -> uav0_velocity_guidance_node.py
        -> /uav0/mavros/setpoint_velocity/cmd_vel
```

The fusion node does not estimate target velocity. It only publishes a current position observation and quality fields. The target estimated odom is still `/target_estimator/state`, produced by `target_state_estimator_node.py` using Kalman CV/CA. The future guidance point is still `/target_estimator/intercept_point`.

Current full-fusion launch:

```bash
roslaunch uav_truth_tracker intercept_depth_lidar_fusion_chase.launch
```

Current default highlights in `intercept_depth_lidar_fusion_chase.launch`:

- Target motion: `target_mode:=circle_z_sine`, `uav1_speed:=1.2`, `z_amplitude:=0.5`.
- Depth detector: `/iris0/camera/depth/points`, `depth_z_min:=0.3`, `depth_z_max:=5.0`, `depth_min_cluster_size:=8`.
- LiDAR detector: `/uav0/velodyne_points`, `lidar_z_min:=2.0`, `lidar_z_max:=5.0`, `lidar_min_cluster_size:=5`, `lidar_use_estimator_gating:=false`.
- Fusion: `fusion_mode:=weighted`, `prefer_visual:=true`, `require_consistency_for_visual:=true`, `visual_min_confidence:=0.40`, `lidar_min_confidence:=0.25`, `max_observation_age:=0.8`.
- Fusion smoothing/current clamp settings: `fusion_output_smoothing_alpha:=0.92`, `fusion_clamp_output_motion:=false`, `fusion_clamp_output_z_bounds:=false`.
- Estimator: `observation_source:=target_pose`, `target_pose_topic:=/target_fused/pose`, `estimator_model:=kalman_cv`.
- Guidance: `lost_target_behavior:=continue_predict`, `Kp:=2.5`, `max_closing_speed:=3.5`, `terminal_guidance_enable:=true`.
- Altitude guard: guidance clamps with `guidance_z_min:=1.2`, `guidance_z_max:=5.5`, `altitude_guard_min_z:=1.5`, `altitude_guard_max_z:=5.5`.
- Capture/evaluator truth uses spawn offsets, but online detector/fusion/estimator do not use `/uav1/mavros/local_position/odom`.

Fusion behavior:

- `target_sensor_fusion_node.py` subscribes to visual and LiDAR pose, covariance, valid, and confidence topics.
- It publishes `/target_fused/pose`, `/target_fused/pose_cov`, `/target_fused/valid`, `/target_fused/confidence`, `/target_fused/source`, and `/target_fused/marker`.
- In `priority` mode, visual is selected first when it is valid and fresh enough; LiDAR is fallback.
- In the current full-fusion launch, `weighted` mode can blend visual and LiDAR when both are usable. If one side is stale or invalid, fusion falls back to the usable source. If neither source is fresh enough, it publishes `source=none` and `valid=false`.
- `/target_fused/source` is the quickest live diagnostic: expected values include `visual`, `lidar`, `weighted`, `visual_hold`, `lidar_hold`, or `none`.

Coordinate policy:

- `/target_visual/pose`, `/target_observation/pose`, and `/target_fused/pose` are treated as `map`/common-frame observations.
- In the fusion chase include, `target_state_estimator_node.py` and `uav0_velocity_guidance_node.py` use `use_spawn_offsets:=false` because `/target_fused/pose` is already a common-frame target observation.
- `capture_decision_node.py`, `chase_evaluator_node.py`, and `target_truth_compare_node.py` use `/uav1/mavros/local_position/odom` only as debug/evaluator truth and apply spawn offsets for truth comparison.
- Do not feed `/uav1/mavros/local_position/odom` into detector, fusion, or estimator online inputs.

Run fusion debug first:

```bash
roslaunch uav_truth_tracker target_fusion_debug.launch
rostopic hz /target_visual/pose
rostopic hz /target_observation/pose
rostopic hz /target_fused/pose
rostopic echo /target_fused/source
rostopic echo /target_fused/valid
```

Useful debug checks:

```bash
rostopic echo /target_fused/source
rostopic echo -n 1 /target_fused/pose
rostopic echo /target_estimator/tracking_state
rostopic echo -n 1 /target_estimator/intercept_point
rostopic echo -n 1 /uav0/mavros/setpoint_velocity/cmd_vel
```

RViz displays:

- `MarkerArray`: `/uav_models/markers`
- `PointCloud2`: `/iris0/camera/depth/points`
- `PointCloud2`: `/uav0/velodyne_points`
- `MarkerArray`: `/target_visual/marker`
- `MarkerArray`: `/target_observation/marker`
- `MarkerArray`: `/target_fused/marker`
- `Pose`: `/target_fused/pose`
- `Marker`: `/target_estimator/debug_marker`
- `Marker`: `/uav0/guidance/marker`

Run the full fusion chase directly:

```bash
roslaunch uav_truth_tracker intercept_depth_lidar_fusion_chase.launch \
  target_mode:=circle_z_sine \
  uav1_speed:=1.2 \
  z_amplitude:=0.5 \
  estimator_model:=kalman_cv
```

One-key fusion startup from `$HOME/ego_ws`:

```bash
./one_key_intercept.sh fusion
```

This starts dual UAV SITL with the `/uav0` LiDAR model, uses temporary hover for takeoff, and then starts `intercept_depth_lidar_fusion_chase.launch`. `fusion_debug` starts only the detectors and fusion chain after hover; it does not start estimator or velocity guidance.

Current known failure signatures:

- `source=none` for a long time: both visual and LiDAR observations are stale or invalid; check detector logs before tuning Kalman.
- `lidar_valid=True` but `lidar_ok=False`: LiDAR observation exists but is older than `max_observation_age`.
- `tracking_state=LOST` with old estimator/intercept values: estimator is no longer receiving valid `/target_fused/pose`.
- `/target_fused/pose` far from `/uav1` marker: inspect `/target_visual/marker` and `/target_observation/marker` separately to find which detector is wrong.
- `/uav0` hovers while `tracking_state=LOST`: guidance is operating on stale or missing estimator output; fix `/target_fused/valid` first.
- High z prediction or climb: inspect `/target_fused/pose.z`, `/target_estimator/intercept_point.z`, and guidance `ZLimit`/`AltitudeGuard` logs.

## Stage 6D: Visual Lost and Reacquisition

When the target moves out of the front camera FOV:

- visual valid: visual dominates fusion.
- visual lost briefly: the estimator continues Kalman predict-only.
- LiDAR valid: fusion publishes LiDAR with larger covariance and `source=lidar`.
- both lost: fusion publishes `source=none`, and the estimator naturally enters predict-only/LOST according to observation age.

`target_reacquire_manager_node.py` publishes:

- `/target_reacquire/state`: `VISUAL_TRACK`, `VISUAL_LOST_PREDICT`, `LIDAR_REACQUIRE`, or `FULL_LOST`
- `/target_reacquire/yaw_target`: yaw angle for optional guidance yaw reacquisition

The combined depth/LiDAR launch is now documented as an experimental fusion intercept path. Keep the LiDAR-only path as the baseline reference, and use `target_fusion_debug.launch` before running the full fusion chase whenever detection quality is uncertain.

Analyzer output now includes visual/LiDAR/fused detection rates, mean 3D errors, source ratios, lost durations, and these additional figures:

- `fused_source_timeline.png`
- `visual_lidar_fused_error_vs_time.png`
- `detection_source_vs_time.png`
- `visual_valid_vs_time.png`
- `lidar_valid_vs_time.png`
- `fused_valid_vs_time.png`

The geometric depth/LiDAR path remains useful for sensor diagnostics, but it
cannot reliably distinguish a UAV from obstacle clusters. The recommended
clutter-world main line is now:

```text
RGB YOLO UAV confirmation + LiDAR position fallback + Kalman CV/CA estimator
```

Future ego_planner integration can keep this perception layer and use
`/target_fused/object_cloud` or a future `bbox_3d` output to remove the target
UAV from obstacle clouds.

## Stage 6E: UAV Semantic YOLO Confirmation

Geometry-only LiDAR and depth clustering can select trees, poles, walls, or
terrain fragments even when their cluster dimensions look UAV-like. Stage 6E
adds a one-class RGB UAV detector without modifying the original `/uav1`
model:

```text
/iris0/camera/rgb/image_raw
        -> target_yolo_detector_node.py
        -> /target_semantic/pose

/uav0/velodyne_points
        -> target_lidar_detector_node.py
        -> /target_observation/pose

/target_semantic/pose + /target_observation/pose
        -> target_sensor_fusion_node.py
        -> /target_fused/pose
        -> target_state_estimator_node.py
        -> /target_estimator/intercept_point
        -> uav0_velocity_guidance_node.py
```

`target_yolo_detector_node.py` uses an RGB bounding box plus the aligned depth
image to publish a current 3D target position. It does not estimate velocity.
In the semantic fusion launch, LiDAR-only fallback is accepted only near a
recent YOLO-confirmed UAV position. This prevents an unconfirmed obstacle
cluster from becoming a chase target.

The data collector is intentionally separate from the online chain:

```text
LiDAR chase -> RGB image capture
/uav1/mavros/local_position/odom -> offline YOLO label generation only
```

`collect_uav_yolo_dataset_node.py` may use `/uav1` truth to create training
labels. `target_yolo_detector_node.py`, `target_lidar_detector_node.py`,
`target_sensor_fusion_node.py`, and the target-pose estimator path must not use
`/uav1` odom online.

Collect a first dataset while `/uav0` follows `/uav1` using the LiDAR baseline:

```bash
cd $HOME/ego_ws
YOLO_MAX_IMAGES=3000 TARGET_MODE=figure8_3d UAV1_SPEED=0.8 \
  ./one_key_intercept.sh yolo_collect_light
```

The default dataset directory is `~/uav_yolo_dataset`. It contains YOLO-format
images, labels, positive samples, and background negative samples. Repeated
collection sessions continue numbering instead of overwriting existing data.

When the first model has low confidence or poor recall, collect more positive
samples with the offline truth-follow mode. This mode uses
`/uav1/mavros/local_position/odom` only to keep `/uav0` near `/uav1` and to
generate labels. It is a dataset-generation tool, not a valid online chase
evaluation:

```bash
cd $HOME/ego_ws
WORLD=$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/warehouse.world \
PX4_GAZEBO_GUI=false \
YOLO_MAX_IMAGES=0 \
YOLO_MAX_NEW_IMAGES=1000 \
YOLO_DATASET_TAG=warehouse \
TARGET_MODE=figure8_3d \
UAV1_SPEED=0.8 \
  ./one_key_intercept.sh yolo_collect_truth
```

`YOLO_MAX_NEW_IMAGES` limits only the current session. `YOLO_DATASET_TAG` is
included in each image filename, so samples from different worlds can be
identified and repeated sessions do not overwrite older files.

Run the default multi-world collection sequence:

```bash
cd $HOME/ego_ws
IMAGES_PER_WORLD=1000 PX4_GAZEBO_GUI=false \
  rosrun uav_truth_tracker collect_uav_yolo_multiworld.sh
```

Or provide an explicit world list:

```bash
rosrun uav_truth_tracker collect_uav_yolo_multiworld.sh \
  $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world \
  $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/warehouse.world \
  $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/outdoor_village.world
```

Truth-follow collection mostly produces positive UAV samples. Keep collecting
background negative images with LiDAR-follow or hover sessions where `/uav1`
is outside the camera view or occluded. Those negatives are required to reduce
false UAV detections on trees, poles, walls, and terrain.

Train the one-class UAV detector:

```bash
rosrun uav_truth_tracker train_uav_yolo.py ~/uav_yolo_dataset \
  --model yolo11n.pt \
  --epochs 80 \
  --imgsz 416 \
  --batch 8 \
  --device cpu \
  --project ~/uav_yolo_runs \
  --name uav_detector
```

The original expected weight file is
`~/uav_yolo_runs/uav_detector/weights/best.pt`.

After multi-world collection, the current default semantic model path is:

```text
~/uav_yolo_runs/uav_detector_v2/weights/best.pt
```

`one_key_intercept.sh` automatically prefers the v2 weight if it exists, and
falls back to the original `uav_detector/weights/best.pt` otherwise.

Run semantic detector debug before enabling chase:

```bash
./one_key_intercept.sh yolo_debug
rostopic hz /target_semantic/pose
rostopic echo /target_semantic/valid
rostopic echo -n 1 /target_semantic/pose
```

Then run semantic-confirmed LiDAR fusion chase:

```bash
./one_key_intercept.sh yolo_fusion_light
rostopic echo /target_fused/source
rostopic echo /target_fused/valid
```

Interpretation:

- `source=visual`: YOLO has identified a UAV and depth produced a usable 3D position.
- `source=lidar`: LiDAR is temporarily filling a gap near a recent semantic UAV confirmation.
- `source=none`: there is no semantically confirmed target; `/uav0` must not chase an arbitrary obstacle cluster.

Current tested v2 behavior in `lidar_clutter_light.world`:

- Hover/target-in-view YOLO debug: `/target_semantic/pose` publishes at about
  6 Hz, valid is continuous, and mean semantic 3D error is about 0.18 m against
  offline `/uav1` truth.
- Dynamic YOLO+LiDAR fusion chase: YOLO semantic detections become sparse when
  `/uav1` leaves the camera view, but accepted semantic detections are accurate.
  The fusion launch keeps LiDAR fallback enabled for up to 8 s after a recent
  semantic confirmation, with a 6 m spatial gate.
- A 30 s dynamic v2 test produced mean semantic error about 0.22 m on valid
  semantic frames and mean fused error about 0.28 m on valid fused frames.

## Stage 6: 32-Channel Velodyne-Style LiDAR Target Detection and Intercept

Stage 6 replaces the online target observation with simulated 3D LiDAR detection while keeping the same Kalman CV/CA estimator and velocity guidance:

```text
/uav0/velodyne_points
        -> target_lidar_detector_node.py
        -> /target_observation/pose
        -> target_state_estimator_node.py
        -> /target_estimator/intercept_point
        -> uav0_velocity_guidance_node.py
        -> /uav0/mavros/setpoint_velocity/cmd_vel
```

The online input is `/uav0/velodyne_points -> /target_observation/pose`. Offline evaluation truth is `/uav1/mavros/local_position/odom`. The LiDAR detector does not subscribe to `/uav1` odom, and when `observation_source:=target_pose`, the estimator does not use `/uav1` odom as target observation.

Current stable checkpoint:

- `/uav0` LiDAR model: `models/iris_depth_camera_lidar`, with `velodyne_link` placed on top of the Iris at `z=0.06m`.
- Point-cloud path: Gazebo `/uav0/velodyne_points_raw` -> `pointcloud_to_pointcloud2_relay.py` -> `/uav0/velodyne_points`.
- Detector path: `target_lidar_detector_node.py` publishes `/target_observation/pose`, `/target_observation/valid`, `/target_observation/confidence`, `/target_observation/point_count`, and `/target_observation/marker`.
- Chase path: `intercept_lidar_estimator_chase.launch` runs detector, Kalman CV/CA estimator, velocity guidance, capture decision, and evaluator.
- Continuous chase: LiDAR mode defaults to `stop_on_capture_success=false` and `latch_capture_success=false`, so evaluator capture events are logged but do not command `/uav0` to hold zero velocity.

### 32-Channel Detector Baseline

The current Stage 6 mainline uses `target_lidar_detector_node.py`; older legacy/simple/raw/basic detector launch modes are not part of this path.

The Stage 6 baseline subscribes to `/uav0/velodyne_points`, `/uav0/mavros/local_position/odom` for range filtering, and optional `/target_estimator/state` or `/target_estimator/intercept_point` for association. It never subscribes to `/uav1/mavros/local_position/odom`; `/uav1` odom remains evaluator/offline truth only.

The detector does finite point filtering, TF transform with source-frame fallback, range/z filtering, voxel downsample, Euclidean clustering, target-size filtering, and best-cluster publication. Obstacle rejection uses current-cluster geometry: horizontal extent, vertical extent, overall size, and aspect ratio. If an association reference is available, distance and geometry are scored together so a nearby wall fragment or pole does not win only because it is close to the previous detection. Confidence is quality metadata and is recorded by the evaluator. If no cluster survives, the node publishes `valid=false` and the once-per-second log shows whether the break happened at `raw_points=0`, `filtered_points=0`, `raw_cluster_count=0`, or after size/shape rejection. The LiDAR launch starts `pointcloud_to_pointcloud2_relay.py` by default to convert Gazebo's `/uav0/velodyne_points_raw` `PointCloud` stream into `/uav0/velodyne_points` `PointCloud2`.

The first target is a working LiDAR observation stream. If the detector publishes `/target_observation/pose` but the chase lags, LiDAR observation is available and later work belongs in estimator tuning or depth fusion. If the detector cannot publish a pose in the two-UAV world, debug the LiDAR point cloud, TF, model geometry, and target observability before touching Kalman or guidance.

Test order:

```bash
roslaunch uav_truth_tracker target_lidar_detector_debug.launch
rostopic info /target_observation/pose
rostopic hz /target_observation/pose
rostopic echo -n 1 /target_observation/pose
rostopic echo -n 1 /target_observation/valid
rostopic echo -n 1 /target_observation/confidence
rostopic echo -n 1 /target_observation/point_count
```

`rostopic info /target_observation/pose` should show exactly one publisher:

```text
/target_lidar_detector_node
```

Then run the LiDAR chase:

```bash
roslaunch uav_truth_tracker intercept_lidar_estimator_chase.launch
rostopic hz /target_observation/pose
rostopic echo /target_estimator/tracking_state
rostopic hz /target_estimator/intercept_point
rostopic echo /uav0/mavros/setpoint_velocity/cmd_vel
```

Interpretation:

- If `/target_observation/pose` has no frequency in the LiDAR detector debug launch, the problem is LiDAR visibility, topic availability, TF, or the Gazebo model, not estimator/guidance.
- If pose has frequency but `/target_estimator/tracking_state` is `LOST`, check estimator confidence threshold, `/target_observation/valid`, and covariance/noise settings.
- If `/target_estimator/intercept_point` has frequency but `/uav0/mavros/setpoint_velocity/cmd_vel` is zero, check guidance diagnostics for `no_estimator_state`, `no_intercept_point`, `target_lost_hold`, `yaw_gate`, `stop_distance`, `capture_success_stop`, or `offboard_not_ready`.

Do not tune detector, estimator, and guidance at the same time. First make `/target_observation/pose` stable with the LiDAR detector. If the LiDAR detector works but prediction lags, then tune estimator or add depth camera fusion later. If the LiDAR detector still cannot see a cluster in a two-UAV world, debug LiDAR observability and the vehicle model before tuning any Kalman or guidance parameter.

The LiDAR detector logs once per second with:

```text
[TargetLidarDetector] raw_points=... finite_points=... tf_ok=True/False filtered_points=... voxel_points=... raw_cluster_count=... cluster_count=... size_rejected=... shape_rejected=... selected_points=... selected_range=... selected_z=... selected_bbox_diag=... published_pose=True/False confidence=... frame_id=... selection=...
```

The LiDAR detector does not contain detector state machines, temporal cloud accumulation, MHT, target trajectory script prediction, or online `/uav1` odom assistance.

The local Stage 6 model is `models/iris_depth_camera_lidar`. It uses a Gazebo Classic 32-channel 360 degree ray sensor with 360 horizontal samples, approximately 30 degree vertical field of view, `range_min=0.3m`, `range_max=30m`, and `update_rate=10Hz`. Each scan contains 11520 points. Because this system has `velodyne_description` meshes but not `velodyne_gazebo_plugins`, the model uses `libgazebo_ros_block_laser.so` and the relay node republishes `/uav0/velodyne_points` as `sensor_msgs/PointCloud2`. This is a Velodyne-style simulation path, not a millimeter-wave radar model. The 32 vertical samples improve the number of returns on a small UAV while preserving the existing topic and frame contract.

Start dual UAV SITL with the LiDAR model on `/uav0`:

```bash
roslaunch uav_truth_tracker dual_uav_mavros_sitl.launch \
  uav0_sdf_jinja:=$(rospack find uav_truth_tracker)/models/iris_depth_camera_lidar/iris_depth_camera_lidar.sdf.jinja
```

One-key Stage 6 startup from `$HOME/ego_ws`:

```bash
./one_key lidar
```

For obstacle-interference testing without the cost of a warehouse, mountain, or forest world, use the local lightweight clutter world. It contains only the ground plane and three simple static obstacles:

```bash
./one_key lidar_light
```

The same lightweight world is available for the depth visual primary + LiDAR fallback path:

```bash
./one_key fusion_light
```

These modes are intended for fast detector regression testing. Use `lidar` in the empty world first to confirm the basic observation chain, then use `lidar_light` to check whether static obstacles are rejected.

Use detector-only debug mode first when you only want to inspect the LiDAR point cloud and target cluster in RViz:

```bash
./one_key lidar_debug
```

Optional overrides:

```bash
TARGET_MODE=figure8_3d UAV1_SPEED=0.8 Z_AMPLITUDE=0.5 ESTIMATOR_MODEL=kalman_ca ./one_key lidar
```

`./one_key lidar_debug` starts Gazebo/PX4/MAVROS, keeps both vehicles hovering, and runs `target_lidar_detector_node.py`. It does not start the estimator or `/uav0` velocity guidance, so `/uav0` will not chase.

After `lidar_debug` is publishing `/target_observation/pose`, start the chase manually in a new terminal:

```bash
roslaunch uav_truth_tracker intercept_lidar_estimator_chase.launch \
  target_mode:=circle_z_sine \
  uav1_speed:=0.6 \
  z_amplitude:=0.5 \
  estimator_model:=kalman_cv
```

This launch starts the LiDAR detector, estimator, and guidance chain and does not subscribe to `/uav1/mavros/local_position/odom` as the online detector or estimator input. Once `/uav0/mavros/setpoint_velocity/cmd_vel` is publishing steadily, stop the `LiDAR_Debug_Hover` terminal to avoid a continuous position setpoint publisher fighting velocity guidance.

LiDAR Stage 6 now defaults to continuous intercept behavior: `stop_on_capture_success=false`, `latch_capture_success=false`, and `stop_distance=0.3`. A capture event is still computed by `capture_decision_node.py` from evaluator truth for logging/evaluation, but it no longer commands `/uav0` to stop in the LiDAR chase. The previous symptom was `reason_if_zero_velocity=capture_success_stop`; if that appears again during LiDAR mode, an old launch or explicit override is still enabling capture-stop behavior. The online detector and estimator still must not use `/uav1/mavros/local_position/odom`.

The LiDAR chase commands yaw toward the guidance direction but does not block translation by default: `yaw_mode=face_guidance`, `face_guidance_before_move=false`, `yaw_gate_xy_scale=1.0`, and `allow_vertical_during_yaw_align=true`. If you see `reason_if_zero_velocity=yaw_gate`, an old launch or explicit override is still using yaw gating.

`./one_key lidar` starts Gazebo/PX4/MAVROS dual UAV simulation, spawns `/uav0` with the LiDAR model, runs `dual_hover_check` only as a temporary takeoff step, then starts `intercept_lidar_estimator_chase.launch`: LiDAR detector, estimator, velocity guidance, capture decision, and evaluator. It does not start ego_planner. The temporary hover publisher exits after `STAGE6_INITIAL_HOVER` seconds, so the chase phase is controlled by `/uav0/mavros/setpoint_velocity/cmd_vel` only. If `/uav0` is not yet armed/OFFBOARD, switch it after velocity setpoints are publishing:

```bash
rosservice call /uav0/mavros/set_mode "custom_mode: 'OFFBOARD'"
rosservice call /uav0/mavros/cmd/arming "value: true"
```

Step 1, check LiDAR point cloud:

```bash
rostopic list | grep -E 'velodyne|lidar|points'
rostopic hz /uav0/velodyne_points
rostopic echo -n 1 /uav0/velodyne_points/header
```

The older depth camera point cloud remains available as `/iris0/camera/depth/points`; it is a depth camera point cloud and is not equivalent to the 32-channel Velodyne-style LiDAR path.

Step 2, validate only the detector:

```bash
roslaunch uav_truth_tracker target_lidar_detector_debug.launch
rostopic info /target_observation/pose
rostopic echo /target_observation/pose
rostopic hz /target_observation/pose
rostopic echo -n 1 /target_observation/marker
rostopic echo /target_observation/confidence
rostopic echo /target_observation/point_count
rostopic echo /target_observation/valid
```

RViz displays for detector debug:

- `MarkerArray`: `/uav_models/markers`
- `PointCloud2`: `/uav0/velodyne_points`
- `PointCloud2`: `/target_observation/debug_cloud`
- `MarkerArray`: `/target_observation/marker`
- `Pose`: `/target_observation/pose`

Step 3, run full LiDAR intercept:

```bash
roslaunch uav_truth_tracker intercept_lidar_estimator_chase.launch \
  target_mode:=circle_z_sine \
  uav1_speed:=0.6 \
  z_amplitude:=0.5 \
  estimator_model:=kalman_cv
```

Step 4, analyze logs:

```bash
rosrun uav_truth_tracker analyze_prediction_log.py \
  ~/uav_intercept_logs/<log>.csv \
  --vxy-max 2.2 \
  --vz-max 0.8 \
  --terminal-window 3.0
```

The evaluator records both online detection and offline truth metrics in LiDAR mode:

- `observation_source`
- `lidar_cloud_topic`
- `target_observation_x/y/z`
- `detection_valid`
- `detection_confidence`
- `detection_point_count`
- `detection_age`
- `tracking_state`
- `observation_age`
- `detector_state`
- `candidate_count`
- `range_valid_cluster_count`
- `selected_cluster_id`
- `selected_cluster_point_count`
- `selected_cluster_score`
- `second_best_score`
- `association_margin`
- `hit_count`
- `miss_count`
- `reacquire_count`
- `time_since_last_detection`
- `gate_radius`
- `use_estimator_filtering`
- `selected_distance_to_gate`
- `reject_reason_summary`
- `target_lost`
- `predict_only`
- `detection_error_3d`
- `detection_error_xy`
- `detection_error_z`

### LiDAR Observation Robustness

Even with 32 vertical channels, a single frame may only hit a few UAV surface points at long range, so Stage 6 treats the detector output as a quality-rated observation stream instead of a perfect odometry replacement.

The LiDAR detector publishes `/target_observation/confidence`, `/target_observation/valid`, `/target_observation/point_count`, and `/target_observation/pose_cov` next to the pose stream. These topics are diagnostic inputs for the estimator and evaluator; confidence does not block pose publication.

`target_state_estimator_node.py` uses those quality topics when `observation_source:=target_pose`. Valid observations with confidence at or above `0.25` update Kalman CV/CA; confidence from `0.25` to `0.55` is accepted with larger measurement noise instead of being dropped. Short detection gaps enter `PREDICT_ONLY`, where the Kalman state is propagated for up to `max_predict_only_time:=0.8`. Longer gaps enter `LOST`, where `freeze_prediction_after_lost:=true` prevents long-running CV/CA drift. The online estimator still does not subscribe to `/uav1/mavros/local_position/odom`.

`uav0_velocity_guidance_node.py` distinguishes `TRACKING`, `PREDICT_ONLY`, and `LOST`. In `PREDICT_ONLY` it keeps chasing the predicted intercept point with `predict_only_speed_scale:=0.7`. In Stage 6 recovery chase tests the default is `lost_target_behavior:=continue_predict` with `lost_target_speed_scale:=0.3`, so short detector outages do not make `/uav0` stop immediately. Keep `capture_slowdown_distance` greater than `stop_distance`; the node now warns and repairs the parameter if that interval is invalid.

Recommended interpretation:

- Low `detection_rate`: check raw cloud frequency, TF, range/z limits, cluster tolerance, or LiDAR/target observability.
- Low `mean_detection_point_count`: reduce voxel size or consider more LiDAR vertical samples.
- Many false detections: inspect RViz markers and tighten size/range limits after the LiDAR detector is publishing.
- High `predict_only_ratio` with successful capture: predict-only handling is doing useful work.
- Long `target_lost_duration_total` or `longest_detection_gap`: fix the observation stream before tuning Kalman.

LiDAR detector parameters:

- `lidar_cloud_topic`
- `range_min`, `range_max`
- `z_min`, `z_max`
- `self_exclusion_enable`, `self_exclusion_radius`
- `enable_voxel_filter`
- `voxel_leaf_size`
- `cluster_tolerance`
- `min_cluster_size`, `max_cluster_size`
- `target_size_min`, `target_size_max`
- `expected_target_size`
- `enable_shape_filter`
- `target_xy_size_min`, `target_xy_size_max`
- `target_z_size_min`, `target_z_size_max`
- `target_aspect_ratio_max`
- `expected_target_xy_size`, `expected_target_z_size`
- `reference_distance_weight`
- `always_publish_best_cluster`
- `publish_debug_cloud`

### Detector Debug Notes

The 32-channel LiDAR can still produce sparse clusters on a small UAV. The detector uses current-frame geometry filtering and a short association reference, but it does not use `/uav1` truth, temporal cloud accumulation, or MHT. In clutter, inspect `size_rejected` and `shape_rejected` together with RViz candidate boxes before changing estimator or guidance parameters.

Association debug topics:

- `/target_observation/pose`
- `/target_observation/pose_cov`
- `/target_observation/valid`
- `/target_observation/confidence`
- `/target_observation/point_count`
- `/target_observation/marker`
- `/target_observation/debug_cloud`

RViz normal displays:

- `MarkerArray`: `/uav_models/markers`
- `PointCloud2`: `/uav0/velodyne_points`
- `PointCloud2`: `/target_observation/debug_cloud`
- `MarkerArray`: `/target_observation/marker`
- `Pose`: `/target_observation/pose`
- `Marker`: `/target_estimator/debug_marker`

By default `/target_observation/marker` shows the accepted clusters and selected pose marker. The selected cluster is green/yellow; non-selected clusters are blue.

Recommended association debug flow:

```bash
./one_key lidar_debug
```

Then inspect in RViz whether the real UAV1 cluster exists and whether `cluster_count` is nonzero in the LiDAR detector log.

Run the full LiDAR chase after detector debug:

```bash
./one_key lidar
```

Analyze chase metrics:

```bash
rosrun uav_truth_tracker analyze_prediction_log.py \
  ~/uav_intercept_logs/<log>.csv \
  --vxy-max 2.2 \
  --vz-max 0.8 \
  --terminal-window 3.0
```

Important diagnosis:

- `raw_points=0`: the LiDAR topic or Gazebo sensor path is not publishing usable cloud data.
- `filtered_points=0`: check range/z limits, TF frame choice, and whether the target is within the LiDAR scan.
- `raw_cluster_count=0`: check cluster tolerance, voxel leaf size, and target surface returns.
- `raw_cluster_count>0` with `cluster_count=0`: check `size_rejected`, `shape_rejected`, and the target geometry limits.
- `published_pose=True` but large `detection_error_3d`: the LiDAR detector is publishing, but the selected cluster is wrong or the TF frame is wrong.
- Long-term `detection_point_count < 3`: LiDAR observability is insufficient; consider a larger collision-only reflector or more vertical samples.

### Stage 6 Troubleshooting

Use this split before tuning Kalman CV/CA or velocity guidance:

- If the LiDAR detector has no pose frequency, the problem is LiDAR/target observability, topic availability, TF, or the vehicle model.
- If pose has frequency but `/target_estimator/tracking_state` stays `LOST`, check `/target_observation/valid`, pose timestamps, covariance/noise settings, and estimator observation age.
- If `/target_estimator/intercept_point` has frequency but velocity command stays near zero, check guidance diagnostics for `no_estimator_state`, `no_intercept_point`, `target_lost_hold`, `yaw_gate`, `stop_distance`, `capture_success_stop`, or `offboard_not_ready`.
- If UAV0 suddenly flies away, stop the full chase and run only `target_lidar_detector_debug.launch` to verify the selected marker before touching estimator or guidance.
- For continuous intercept tests, the default command is now simply `./one_key lidar`. The LiDAR detector log should show cluster counts and whether pose publication is happening.
- `/uav1/mavros/local_position/odom` is evaluator truth only. It must not be fed into the online detector or estimator in Stage 6 LiDAR mode.

For observability experiments, keep the default model topics and frames unchanged. `models/iris_lidar_target` is an optional enhanced target model with multiple collision-only LiDAR reflectors; it is not used by the default launches.

Debug-only alignment check:

```bash
rosrun uav_truth_tracker check_lidar_detection_alignment.py
```

This node reads `/uav1/mavros/local_position/odom` only for offline validation and publishes `/target_observation/alignment/*` distances. Do not feed these debug outputs back into detector or estimator.

First-stage acceptance criteria:

- `/uav0/velodyne_points` publishes a `sensor_msgs/PointCloud2` stream near 10Hz.
- RViz shows the LiDAR cloud and `/target_observation/marker`.
- `/target_observation/pose` is close to the orange `/uav1` model marker.
- `target_state_estimator_node.py` runs with `observation_source:=target_pose` and publishes `/target_estimator/intercept_point`.
- `/uav0` can complete a simple `circle_z_sine` intercept from LiDAR observations.
- Logs contain detection error and prediction error fields.

The two MAVROS odometry topics are local to each PX4 instance, so `/uav0` and `/uav1` may appear at the same origin if displayed directly as Odometry in RViz. For common-frame visualization, this launch publishes Iris mesh markers on:

```bash
rostopic echo -n 1 /uav_models/markers
```

In RViz, add `MarkerArray` with topic `/uav_models/markers`. The marker publisher applies the Gazebo spawn offsets:

- `uav0`: `(0, 0, 0)`
- `uav1`: `(2, 0, 0)`

## Arm, OFFBOARD, and Hover Check

Each UAV must receive setpoints before switching to OFFBOARD. Start namespaced hover setpoint streams:

```bash
source $HOME/ego_ws/devel/setup.bash
roslaunch uav_truth_tracker dual_hover_check.launch
```

Then call the namespaced MAVROS services:

```bash
rosservice call /uav0/mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
rosservice call /uav0/mavros/cmd/arming "value: true"

rosservice call /uav1/mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
rosservice call /uav1/mavros/cmd/arming "value: true"
```

Do not use the old global `/mavros/...` services in dual-UAV mode.

For a fully automatic hover check, launch with:

```bash
roslaunch uav_truth_tracker dual_hover_check.launch auto_offboard:=true auto_arm:=true
```

## Start Truth Tracker

In another terminal:

```bash
source $HOME/ego_ws/devel/setup.bash
roslaunch uav_truth_tracker intercept_truth_tracking.launch prediction_time:=1.5
```

The node publishes:

- `/uav0/tracker/goal` as `geometry_msgs/PoseStamped`
- `/uav0/tracker/marker` as `visualization_msgs/Marker`

## RViz

Start RViz:

```bash
rviz
```

Set `Fixed Frame` to `map` if MAVROS odometry uses `map`.

Add these displays:

- `MarkerArray`: topic `/uav_models/markers`
- `Image`: topic `/iris0/camera/depth/image_raw`
- `PointCloud2`: topic `/iris0/camera/depth/points`
- `Pose`: topic `/uav0/tracker/goal`
- `Marker`: topic `/uav0/tracker/marker`

If RViz reports a missing transform, check the `frame_id` in `/uav1/mavros/local_position/odom` and launch the tracker with the same frame:

```bash
roslaunch uav_truth_tracker intercept_truth_tracking.launch frame_id:=map
```

## Stage 2: Truth Chase Loop

This stage still does not start ego_planner, does not use LiDAR, and does not modify the PX4 or dual-UAV launch files. It closes the loop with MAVROS local position setpoints only:

- `/uav1_motion_node` publishes `/uav1/mavros/setpoint_position/local`.
- `/truth_tracker_node` reads `/uav1/mavros/local_position/odom` and publishes `/uav0/tracker/goal`.
- `/uav0_goal_follower_node` reads `/uav0/tracker/goal` and `/uav0/mavros/local_position/odom`, then publishes bounded steps to `/uav0/mavros/setpoint_position/local`.
- `/capture_decision_node` reads both odometry topics and publishes `/uav0/capture/success` plus `/uav0/capture/marker`.

### Why Goals Are Not Published at Odom Rate

`/uav1/mavros/local_position/odom` can update much faster than `/uav0` can physically fly to a setpoint. If every target odom message immediately becomes a new `/uav0/tracker/goal`, the goal moves around the circle before `/uav0` reaches the previous one. The follower then keeps turning toward a moving point instead of building stable closing motion. The current chase launch publishes tracker goals at `5 Hz`, which is fast enough for continuous pursuit but still lower than raw odom.

Fixed lead prediction has a second problem: when `/uav0` is far from `/uav1`, a fixed `prediction_time` may not look far enough ahead, so `/uav0` arrives after `/uav1` has already left. When `/uav0` is close, the same fixed lead may be too aggressive and push the goal too far in front of the target.

Stage 2 therefore separates estimation from command updates:

- `truth_tracker_node.py` receives `/uav1` odom at full rate, estimates target velocity, then publishes `/uav0/tracker/goal` only at `goal_update_rate` Hz.
- The published goal is predictive and can use `fixed`, `adaptive`, or `intercept` prediction.
- `smooth_prediction` filters the predicted time-to-go and rate-limits the published goal so the intercept point moves continuously instead of jumping around the circle.
- `uav0_goal_follower_node.py` smooths pursuit with `max_speed`, `max_accel`, `pursuit_slow_radius`, and a bounded virtual setpoint before publishing `/uav0/mavros/setpoint_position/local`.

### Prediction Modes

- `prediction_mode:=fixed`: keeps the original rule, `p_goal = p_target + v_target * prediction_time`.
- `prediction_mode:=adaptive`: estimates time-to-go as `distance(/uav0,/uav1) / assumed_chaser_speed`, clamps it to `[min_prediction_time, max_prediction_time]`, then predicts linearly.
- `prediction_mode:=intercept`: iterates time-to-go and future target position so the predicted point is closer to a reachable intercept point. This is the default in `intercept_truth_chase.launch`.

`intercept` supports configured target models:

- `prediction_model:=auto`: selects `circle` when `target_mode:=circle`, `line` when `target_mode:=line`, otherwise falls back to `linear`. This is the chase launch default.
- `prediction_model:=linear`: future target position is `p_target + v_target * t_go`.
- `prediction_model:=circle`: future target position is constrained to the configured circle using `target_center_x`, `target_center_y`, `target_radius`, `target_direction`, and `uav1_speed`.
- `prediction_model:=line`: future target position is constrained to the same back-and-forth line segment used by `uav1_motion_node.py`.

The tracker defaults to position-difference velocity (`velocity_source:=diff`) because MAVROS odometry twist can be noisy or expressed differently across setups. Use `velocity_source:=twist` only if you have confirmed `/uav1/mavros/local_position/odom.twist.twist.linear` points in the same local frame as position.

### One-Key Hover Bringup

From `$HOME/ego_ws`:

```bash
./one_key_intercept.sh
```

This starts `dual_uav_mavros_sitl.launch`, then starts `dual_hover_check.launch auto_offboard:=true auto_arm:=true`. It should bring both `/uav0` and `/uav1` into OFFBOARD hover without manual `rosservice call`.

Check:

```bash
rostopic echo -n 1 /uav0/mavros/state
rostopic echo -n 1 /uav1/mavros/state
```

Both should show `armed: True` and `mode: "OFFBOARD"`.

After both UAVs are hovering, start the chase loop manually:

```bash
source $HOME/ego_ws/devel/setup.bash
roslaunch uav_truth_tracker intercept_truth_chase.launch
```

Recommended first intercept test:

```bash
roslaunch uav_truth_tracker intercept_truth_chase.launch \
  prediction_mode:=intercept \
  prediction_model:=circle \
  command_yaw:=false \
  face_goal_before_move:=false
```

After `intercept_truth_chase.launch` is publishing steadily, stop the `Dual_Hover_Arm` terminal opened by `one_key_intercept.sh`, so only the chase nodes publish to the two MAVROS setpoint topics.

### Launch Order

1. Start the dual-UAV PX4 + MAVROS simulation:

```bash
source /opt/ros/noetic/setup.bash
source $HOME/ego_ws/devel/setup.bash
cd $HOME/PX4-Autopilot
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch uav_truth_tracker dual_uav_mavros_sitl.launch
```

2. Start hover setpoint streams, switch both vehicles to OFFBOARD, and arm them:

```bash
source $HOME/ego_ws/devel/setup.bash
roslaunch uav_truth_tracker dual_hover_check.launch
```

In another terminal:

```bash
rosservice call /uav0/mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
rosservice call /uav0/mavros/cmd/arming "value: true"

rosservice call /uav1/mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
rosservice call /uav1/mavros/cmd/arming "value: true"
```

3. Start the chase loop:

```bash
source $HOME/ego_ws/devel/setup.bash
roslaunch uav_truth_tracker intercept_truth_chase.launch
```

After `intercept_truth_chase.launch` is publishing steadily, stop `dual_hover_check.launch` so only the chase nodes publish to:

- `/uav0/mavros/setpoint_position/local`
- `/uav1/mavros/setpoint_position/local`

A short handoff overlap is fine, but do not leave hover and chase setpoint publishers running together for normal tests.

You can also let the chase launch request OFFBOARD and arm after its initial setpoint burst:

```bash
roslaunch uav_truth_tracker intercept_truth_chase.launch auto_offboard:=true auto_arm:=true
```

### Trajectory Modes

Circle is the default:

```bash
roslaunch uav_truth_tracker intercept_truth_chase.launch target_mode:=circle
```

Line mode moves `/uav1` back and forth along x around `target_center_x,target_center_y`:

```bash
roslaunch uav_truth_tracker intercept_truth_chase.launch target_mode:=line target_radius:=3.0 uav1_speed:=0.6
```

Default target parameters:

- `frame_id:=map`
- `target_height:=3.0`
- `target_radius:=3.0`
- `uav1_speed:=0.6`
- `target_center_x:=2.0`
- `target_center_y:=0.0`
- `target_direction:=1.0`

### Check Topics

```bash
rostopic hz /uav1/mavros/setpoint_position/local
rostopic hz /uav0/tracker/goal
rostopic echo /uav0/tracker/goal
rostopic hz /uav0/mavros/setpoint_position/local
rostopic echo -n 1 /uav0/mavros/setpoint_position/local
rostopic echo /uav0/capture/success
rostopic echo -n 1 /uav0/capture/marker
```

Expected Stage 2 topics:

- `/uav1/mavros/setpoint_position/local`
- `/uav0/tracker/goal`
- `/uav0/tracker/marker`
- `/uav0/mavros/setpoint_position/local`
- `/uav0/capture/success`
- `/uav0/capture/marker`

### RViz Displays

Set `Fixed Frame` to `map`, then add:

- `MarkerArray`: `/uav_models/markers`
- `Pose`: `/uav0/tracker/goal`
- `Marker`: `/uav0/tracker/marker`
- `Marker`: `/uav0/capture/marker`

The capture marker is a line between `/uav0` and `/uav1`; it turns yellow inside `capture_distance` and green after the hold-time capture condition is met.

### Tuning

Useful chase parameters:

- `uav1_speed`: target speed in m/s. `0.6` is the default.
- `prediction_mode`: `fixed`, `adaptive`, or `intercept`. `intercept` is the chase launch default.
- `prediction_model`: `auto`, `linear`, `circle`, or `line`. `auto` is the chase launch default and resolves from `target_mode`.
- `prediction_time`: prediction horizon in seconds. `1.5` is the default.
- `assumed_chaser_speed`: effective chase speed used by adaptive/intercept time-to-go prediction. `1.2` is the chase launch default.
- `min_prediction_time`: lower bound for adaptive/intercept prediction time. `0.5` is the default.
- `max_prediction_time`: upper bound for adaptive/intercept prediction time. `4.0` is the default.
- `intercept_iterations`: iteration count for intercept mode. `5` is the default.
- `velocity_source`: `diff`, `twist`, or `auto`; `diff` is the chase launch default.
- `goal_update_rate`: `/uav0/tracker/goal` publish rate in Hz. `5.0` is the chase launch default.
- `smooth_prediction`: filters the tracker prediction before publishing. `true` is the chase launch default.
- `t_go_smoothing_alpha`: low-pass factor for prediction time. Lower is smoother; `0.25` is the chase launch default.
- `goal_smoothing_alpha`: low-pass factor for the published goal. Lower is smoother; `0.35` is the chase launch default.
- `max_t_go_rate`: maximum allowed prediction-time change in seconds per second. `0.8` is the chase launch default.
- `max_goal_speed`: maximum speed of the published `/uav0/tracker/goal`. `1.0` is the chase launch default.
- `keep_goal_on_circle`: keeps smoothed circle-mode goals on the configured circle. `true` is the chase launch default.
- `uav0_max_speed`: follower speed limit in m/s. `1.2` is the chase launch default.
- `uav0_max_accel`: follower acceleration limit in m/s^2. `0.7` is the chase launch default.
- `accept_radius`: goal acceptance radius in meters. `0.5` is the default.
- `stop_at_goal`: whether `/uav0` should brake inside `accept_radius`. It is `false` by default for moving-target pursuit.
- `min_pursuit_speed`: minimum virtual-setpoint speed when `stop_at_goal:=false`. `0.15` is the chase launch default.
- `pursuit_slow_radius`: distance under which the follower reduces speed near the moving goal. `1.2` is the chase launch default.
- `max_setpoint_lead`: maximum distance that the virtual setpoint may lead `/uav0`. `0.9` is the chase launch default.
- `keep_setpoint_between_uav_and_goal`: prevents the virtual setpoint from overshooting beyond the current tracker goal. `true` is the chase launch default.
- `command_yaw`: whether the follower commands yaw toward the goal. `false` is the default for chase testing.
- `face_goal_before_move`: when `true`, `/uav0` turns to face the goal before advancing the setpoint. `false` is the default for chase testing.
- `yaw_align_threshold`: yaw error threshold in radians before translation starts. `0.35` is the default.
- `max_goal_jump`: clamps sudden tracker-goal jumps. The chase launch uses `0.0`, which disables this clamp so a valid far-ahead intercept point is not dragged back toward an old goal.
- `max_tracking_distance`: clamps very distant tracker goals. `20.0` is the default.
- `hold_height_from_goal`: when `true`, `/uav0` follows the predicted goal height.
- `capture_distance`: success radius in meters. `0.35` is the estimator velocity launch default.
- `hold_time`: required continuous time inside the capture radius. `0.15` is the estimator velocity launch default; this is short enough to count high-speed close passes without accepting large miss distances.
- `latch_capture_success`: when `true`, `/uav0/capture/success` stays true after the first valid capture so logs and guidance agree about the capture state.
- `stop_on_capture_success`: when `true`, `/uav0_velocity_guidance_node.py` latches the first `/uav0/capture/success` and commands zero velocity so `/uav0` does not keep chasing through `/uav1` after a successful intercept.

Recommended tuning order:

1. First lower `uav1_speed` until `/uav0` can visibly close distance.
2. Keep `prediction_mode:=intercept prediction_model:=circle` for the circular target.
3. If the red tracker marker jumps forward/back around the circle, lower `max_t_go_rate` or `goal_smoothing_alpha`.
4. If `/uav0` still swings past the intercept point, lower `max_setpoint_lead` or `uav0_max_accel`.
5. Increase `uav0_max_speed` only after the tracker marker and setpoint motion are stable.

Intercept tuning:

- If `/uav0` is always behind `/uav1`, decrease `assumed_chaser_speed` or increase `max_prediction_time`. Lower assumed speed means a longer predicted time-to-go and a farther-ahead intercept point.
- If `/uav0` cuts too far in front, increase `assumed_chaser_speed` or lower `max_prediction_time`.
- If circular-target prediction visibly leaves the circle, use `prediction_model:=circle` or the default `prediction_model:=auto`, then verify `target_radius`, `target_center_x`, `target_center_y`, and `target_direction`.
- If the predicted point appears behind the target, keep `velocity_source:=diff` and check that `target_direction` matches the actual circle direction.
- If `/uav0` flies a little, stops, then flies again, keep `stop_at_goal:=false` and `max_goal_jump:=0.0`; otherwise the moving intercept point is treated like a sequence of static waypoints.
- If `/uav0` oscillates around the path, keep `smooth_prediction:=true`, reduce `max_setpoint_lead` to `0.6`, and reduce `uav0_max_accel` to `0.5`.
- If `/uav0` follows smoothly but always arrives late, increase `max_goal_speed` slightly, for example `1.3`, before increasing `uav0_max_speed`.

If `/uav0` should face the target before pursuit, use `command_yaw:=true face_goal_before_move:=true`. If it rotates but never starts translating, increase `yaw_align_threshold` slightly, for example `0.5`, and check that `/uav0/mavros/setpoint_position/local` moves away from the current `/uav0/mavros/local_position/odom` after yaw alignment. For pure translation debugging, use `command_yaw:=false`.

Example:

```bash
roslaunch uav_truth_tracker intercept_truth_chase.launch \
  target_mode:=circle \
  prediction_mode:=intercept \
  prediction_model:=circle \
  target_radius:=2.0 \
  uav1_speed:=0.5 \
  assumed_chaser_speed:=1.2 \
  max_prediction_time:=4.0 \
  goal_update_rate:=5.0 \
  max_goal_jump:=0.0 \
  smooth_prediction:=true \
  max_t_go_rate:=0.8 \
  max_goal_speed:=1.0 \
  uav0_max_speed:=1.2 \
  uav0_max_accel:=0.7 \
  accept_radius:=0.5 \
  stop_at_goal:=false \
  min_pursuit_speed:=0.15 \
  pursuit_slow_radius:=1.2 \
  max_setpoint_lead:=0.9 \
  command_yaw:=false \
  face_goal_before_move:=false \
  capture_distance:=0.8
```

## Stage 3: Velocity Guidance Chase

Stage 3 keeps the same dual-UAV truth simulation, but changes `/uav0` from position-goal pursuit to velocity guidance. It still does not start ego_planner, does not use LiDAR, and does not modify PX4 launch files.

Stage 3 publishes `/uav1/mavros/setpoint_position/local` from `uav1_motion_node.py`, `/uav0/mavros/setpoint_velocity/cmd_vel` from `uav0_velocity_guidance_node.py`, and capture status from `capture_decision_node.py`. Do not run `intercept_truth_chase.launch`, `uav0_goal_follower_node.py`, or `dual_hover_check` setpoint publishers for `/uav0` at the same time.

### Target Motion vs Prediction

`target_mode` is only the target aircraft motion command:

- `target_mode:=circle` means `/uav1` flies its own circular path.
- `target_mode:=line` means `/uav1` flies its own line path.
- This must not mean `/uav0` knows the target path.

`prediction_model` is the `/uav0` prediction algorithm:

- `current`: no prediction. The guidance point is the current observed target point.
- `linear_observation` or `linear`: estimates target velocity from observed `/uav1` odom history and predicts `p_target + v_target * t_go`. This is the default.
- `learned_circle`: fits a circle from recent observed `/uav1` positions, estimates angular speed from timestamps, and falls back to `linear_observation` if the fit is not stable.
- `circle_configured`: debug/known-trajectory mode. It uses configured `target_center_x`, `target_center_y`, `target_radius`, `target_direction`, and `uav1_speed`, so it is not a formal tracking algorithm.

`auto` no longer switches to configured circle just because `target_mode:=circle`. It uses `linear_observation` unless `learned_circle_enable:=true` and the observed circle fit is stable.

### Common Frame

The two MAVROS odometry streams can be expressed in each vehicle's local origin, while `/uav_models/markers` applies Gazebo spawn offsets for RViz display. Stage 3 therefore computes all guidance in a common `map` frame:

```text
p_uav0_common = p_uav0_odom + uav0_spawn_offset
p_uav1_common = p_uav1_odom + uav1_spawn_offset
```

Defaults match the dual spawn:

```text
use_spawn_offsets:=true
uav0_spawn_x/y/z:=0.0/0.0/0.0
uav1_spawn_x/y/z:=2.0/0.0/0.0
target_center_frame:=uav1_local
```

All relative vectors, velocity differencing, intercept prediction, intercept pose, and RViz guidance markers use this common frame. In RViz, the blue target sphere should overlap the orange `/uav1` model or be very close to it. If it does not, check the spawn offset parameters first.

### Check Topics

```bash
rostopic info /uav0/mavros/setpoint_position/local
rostopic info /uav0/mavros/setpoint_velocity/cmd_vel
rosnode list | grep -E 'hover|goal|motion|velocity|chase|tracker|capture'
rostopic echo -n 1 /uav0/mavros/local_position/odom/pose/pose/position
rostopic echo -n 1 /uav1/mavros/local_position/odom/pose/pose/position
rostopic echo -n 1 /uav0/guidance/intercept_point
```

During Stage 3, `/uav0/mavros/setpoint_position/local` should not have a continuous `/uav0` publisher. `/uav1_motion_node` publishing `/uav1/mavros/setpoint_position/local` is expected.

### RViz Markers

Add the `Marker` topic `/uav0/guidance/marker`:

- blue sphere: current observed target `p_uav1_common`
- red sphere: predicted guidance/intercept point in common frame
- green line: `p_uav0_common` to the red point
- orange line: `p_uav1_common` to the red point
- text: distance, time-to-go, command velocity, yaw error, yaw alignment state, guidance dimension, and prediction model
- purple circle: learned circle when `prediction_model:=learned_circle`
- gray circle: configured circle only when `prediction_model:=circle_configured`

If the blue sphere and orange `/uav1` model are separated by about the spawn offset, the guidance node is using the wrong frame parameters. If `/uav0` still flies a circle in the direct current-point test below, check for setpoint publisher conflicts or MAVROS velocity-frame issues before tuning prediction.

### Recommended Tests

Current-point closing test:

```bash
roslaunch uav_truth_tracker intercept_velocity_chase.launch \
  target_mode:=circle \
  uav1_speed:=0.3 \
  guidance_mode:=los \
  guidance_target:=current \
  prediction_model:=current \
  feedforward_scale:=0.0 \
  yaw_mode:=none
```

Observation-linear prediction:

```bash
roslaunch uav_truth_tracker intercept_velocity_chase.launch \
  target_mode:=circle \
  uav1_speed:=0.3 \
  guidance_mode:=los \
  guidance_target:=intercept \
  prediction_model:=linear_observation \
  feedforward_scale:=0.0 \
  assumed_chaser_speed:=1.2 \
  yaw_mode:=none
```

Learned circle prediction:

```bash
roslaunch uav_truth_tracker intercept_velocity_chase.launch \
  target_mode:=circle \
  uav1_speed:=0.3 \
  guidance_mode:=los \
  guidance_target:=intercept \
  prediction_model:=learned_circle \
  learned_circle_enable:=true \
  feedforward_scale:=0.0 \
  yaw_mode:=none
```

Configured circle should only be used as a debug comparison:

```bash
roslaunch uav_truth_tracker intercept_velocity_chase.launch \
  target_mode:=circle \
  guidance_target:=intercept \
  prediction_model:=circle_configured
```

Do not treat `circle_configured` as a formal tracking predictor. It tells `/uav0` the target's trajectory parameters in advance. When replacing truth odom with LiDAR later, use observation-based models such as `current`, `linear_observation`, or `learned_circle`.

## Stage 5: Three-Dimensional Target Motion Test

Stage 5 extends the Stage 3 velocity chase from a flat circular target to a target that also moves in height. This checks whether `/uav0` can predict and intercept from `/uav1/mavros/local_position/odom` history only, without connecting ego_planner, LiDAR, or PX4 original launch files. `/uav0_velocity_guidance_node.py` must not use the target trajectory parameters for prediction; keep `prediction_model:=linear_observation` for the formal 3D test.

### 3D Target Modes

- `target_mode:=circle_z_sine`: `/uav1` flies the existing XY circle while z follows `target_height + z_amplitude * sin(omega_z * t)`, clamped to `[z_min, z_max]`. This is the safest first 3D test because XY motion stays familiar.
- `target_mode:=figure8_3d`: `/uav1` follows a figure-8 in XY and the same bounded sinusoidal z motion. This tests sharper changing curvature.
- `target_mode:=random_waypoint_3d`: `/uav1` samples random XYZ waypoints inside `x_min/x_max`, `y_min/y_max`, and `z_min/z_max`, then switches after it reaches `waypoint_accept_radius` and holds for `waypoint_hold_time`.

### Guidance Dimension

`guidance_dimension:=split_3d` is the default because multirotor horizontal pursuit and vertical tracking have different limits. XY still uses LOS closing speed toward the predicted guidance point, while z is scheduled to meet the same intercept time:

```text
v_cmd.xy = feedforward_scale * v_target.xy + closing_speed_xy * u_los_xy
v_cmd.z  = z_to_intercept / t_go + Kp_z * z_current_error + Kd_z * vz_error
```

The intercept time `t_go` is solved by scanning the prediction window and checking whether `/uav0` can reach each candidate future target point using separate `assumed_chaser_speed_xy` and `assumed_chaser_speed_z` limits. This keeps the horizontal and vertical arrival times coupled, so `/uav0` does not pass underneath `/uav1` when the target height changes.

Use `guidance_dimension:=xy_only` to compare against the old flat chase behavior. Use `guidance_dimension:=los_3d` only as an experiment; it points along the full 3D LOS vector but still limits XY and z velocity separately.

### Camera-Forward Yaw Alignment

Stage 5 internal velocity launch can run without yaw gating, but the estimator velocity launch defaults to `yaw_mode:=face_guidance` and `face_guidance_before_move:=true`. `/uav0` first turns toward the red predicted guidance point, then starts horizontal translation after yaw alignment. Vertical correction remains active during yaw alignment so `/uav0` does not wait below the target while turning. To keep the gate from making pursuit sluggish, the estimator launch uses `yaw_rate_max:=1.2`, `yaw_align_threshold:=0.35`, `yaw_align_hold_time:=0.1`, and `yaw_align_min_distance:=1.2`.

Relevant parameters:

- `yaw_mode:=face_guidance`: face the predicted guidance point. Use `face_target` to face the current `/uav1` observation instead.
- `face_guidance_before_move:=true`: block linear velocity until yaw is aligned. Set it to `false` for pure pursuit debugging.
- `yaw_align_threshold:=0.35`: yaw error threshold in radians.
- `yaw_align_hold_time:=0.1`: required stable alignment time before translation.
- `yaw_align_min_distance:=1.2`: skip yaw gating when the guidance point is already close enough for final capture.
- `yaw_gate_xy_scale:=0.4`: keep 40% of the horizontal chase command while yaw is not aligned, instead of freezing XY motion completely. This prevents high-speed targets from escaping after a close pass.
- `allow_vertical_during_yaw_align:=true`: keep commanding z velocity while horizontal motion is blocked by yaw alignment.

### Recommended Order

1. `circle_z_sine` with `z_amplitude:=0.5`
2. `circle_z_sine` with `z_amplitude:=1.0`
3. `figure8_3d`
4. `random_waypoint_3d`

Start the prediction-focused chase with:

```bash
roslaunch uav_truth_tracker intercept_estimator_velocity_chase.launch
```

The estimator launch defaults already select `circle_z_sine`, `split_3d`, `kalman_cv`, `uav1_speed:=0.6`, `assumed_chaser_speed_xy:=1.5`, `max_prediction_time:=2.5`, `Kp_z:=0.75`, `vxy_max:=2.2`, `vz_max:=0.8`, `axy_max:=1.8`, `az_max:=1.0`, and yaw alignment before horizontal translation. The current reference evaluation overrides `uav1_speed:=1.0 z_amplitude:=0.5`. Override only the parameter you are testing, for example `z_amplitude:=1.0` or `target_mode:=figure8_3d`.

### Tuning Notes

- If XY catches up but z does not, increase `vz_max`, `az_max`, or `assumed_chaser_speed_z`.
- If `/uav0` passes underneath while yaw is still aligning, keep `allow_vertical_during_yaw_align:=true` and increase `yaw_align_min_distance` so close-range capture is not frozen by yaw gating.
- If height oscillates, lower `az_max`, lower `vz_max`, or increase `Kd_z`.
- If the overall chase cannot catch the target, first lower `uav1_speed` or `z_amplitude`.
- If the predicted point overshoots in z, lower `max_prediction_time`, `assumed_chaser_speed_z`, or `vz_max`.
- If 3D capture keeps failing while XY is close, inspect `distance_z`, `distance_3d`, and `capture_distance` in `/uav0/guidance/marker` and `/uav0/capture/marker`.
- If `/uav0` rotates but does not start moving, increase `yaw_align_threshold` slightly, for example `0.45`, or temporarily set `face_guidance_before_move:=false` for pure pursuit debugging.

## Formal Prediction Model Evaluation

In the real illegal-UAV capture problem, `/uav0` cannot know `/uav1`'s future script trajectory. `target_mode`, `target_radius`, `target_center_x`, `target_direction`, and `uav1_speed` are only inputs to the `/uav1` motion script. They must not be used by formal online prediction models.

The prediction-focused launch is:

```bash
roslaunch uav_truth_tracker intercept_estimator_velocity_chase.launch
```

Online data flow:

```text
/uav1/mavros/local_position/odom
        |
target_state_estimator_node.py
        |
/target_estimator/state
/target_estimator/intercept_point
/target_estimator/t_go
/target_estimator/debug_marker
        |
uav0_velocity_guidance_node.py
        |
/uav0/mavros/setpoint_velocity/cmd_vel
```

The estimator also subscribes to `/uav0/mavros/local_position/odom` so it can choose a reachable time-to-go for the current pursuer state. All estimator positions and markers use the common `map` frame with spawn offsets:

```text
p_uav0_common = p_uav0_odom + uav0_spawn_offset
p_uav1_common = p_uav1_odom + uav1_spawn_offset
```

Formal online models:

- `kalman_cv`: constant-velocity Kalman filter.
- `kalman_ca`: constant-acceleration Kalman filter.

Legacy/debug online models:

- `current`: legacy baseline, no future prediction.
- `linear_observation`: legacy baseline, velocity from observed position differencing only.
- `learned_circle`: legacy periodic baseline, fits a circle from historical observed positions only.
- `circle_configured_debug`: known-trajectory upper-bound comparison. It may read configured target trajectory parameters and must not be used as a formal prediction result.

Target speed is unknown online. The estimator therefore fits velocity from a short observation window instead of trusting a single odom difference. Current estimator launch defaults are tuned for the CV model:

- `velocity_fit_window:=0.5`: seconds of recent observations used for velocity regression.
- `min_velocity_fit_points:=4`: minimum samples for the velocity fit.
- `velocity_smoothing_alpha:=0.35`: smoothing for sparse LiDAR fitted velocity in recovery mode.
- `max_target_speed_xy:=2.5`, `max_target_speed_z:=1.5`: outlier clamps for estimated target velocity.
- `kf_velocity_blend:=0.0` in `intercept_lidar_recovery_chase.launch`, and no more than `0.05` by default elsewhere. Do not use high `kf_velocity_blend` in LiDAR mode because raw velocity comes from sparse point-cloud centroid differencing and is very noisy.
- `kf_process_noise_pos:=0.08`, `kf_process_noise_vel:=1.0`: lets the constant-velocity filter adapt faster to curved target motion.
- `kf_process_noise_acc:=0.05`, `kf_accel_blend:=0.02`: keeps the CA model conservative so acceleration does not dominate terminal prediction.
- `max_target_acc_xy:=0.7`, `max_target_acc_z:=0.4`: clamps fitted acceleration outliers for CA.
- `ca_accel_prediction_horizon:=1.0`: applies the CA acceleration term only over the first second of a long prediction horizon; beyond that the model behaves closer to CV so noisy acceleration does not dominate 3-second intercept predictions.
- `kf_measurement_noise:=0.06`: trusts the observed target position slightly more than the earlier default.

Online prediction may use only current and past observations. Offline evaluation may use future truth from the recorded CSV to measure prediction error:

```text
e_pred(t) = ||p_pred(t) - p_uav1_true(t + t_go)||
```

Recommended model test command:

```bash
roslaunch uav_truth_tracker intercept_estimator_velocity_chase.launch \
  target_mode:=circle_z_sine \
  uav1_speed:=1.0 \
  z_amplitude:=0.5 \
  estimator_model:=kalman_cv
```

The evaluator now starts by default and writes CSV logs under `~/uav_intercept_logs`. Analyze one log with:

```bash
rosrun uav_truth_tracker analyze_prediction_log.py \
  ~/uav_intercept_logs/circle_z_sine_kalman_cv_<timestamp>.csv \
  --vxy-max 2.0 \
  --vz-max 0.8 \
  --terminal-window 3.0
```

The analyzer creates a sibling output folder named `<csv_file>_analysis` and writes:

- `summary.txt`
- `distance_3d_vs_time.png`
- `prediction_error_3d_vs_time.png`
- `prediction_error_xy_z_vs_time.png`
- `t_go_vs_time.png`
- `z_error_vs_time.png`
- `trajectory_3d.png`
- `terminal_distance_zoom.png`
- `reachable_margin_vs_time.png`
- `v_cmd_z_vs_z_error.png`
- `prediction_error_histogram.png`

Older recorded result before current CV retuning:

```text
log: ~/uav_intercept_logs/circle_z_sine_kalman_cv_20260523_151035.csv
target_mode: circle_z_sine
estimator_model: kalman_cv
uav1_speed: 0.8
capture_success: True
capture_time: 8.796s
min_distance_3d: 0.121m
mean_distance_3d: 2.144m
mean_prediction_error_3d: 1.268m
max_prediction_error_3d: 5.891m
mean_prediction_error_xy: 1.189m
mean_prediction_error_z: 0.345m
mean_reachable_margin: -0.065s
percent_reachable_margin_positive: 74.5%
```

Offline replay on `circle_z_sine_kalman_cv_20260524_141017.csv` showed that the current CV tuning improves prediction error versus the earlier CV defaults:

```text
old kalman_cv defaults: mean_error_3d=1.856m, p90_error_3d=4.661m, max_error_3d=16.188m
current kalman_cv tuning: mean_error_3d=0.924m, p90_error_3d=2.038m, max_error_3d=11.969m
```

Current reference results after the latest tuning pass:

```text
log: ~/uav_intercept_logs/circle_z_sine_kalman_cv_20260525_205057.csv
target_mode: circle_z_sine
estimator_model: kalman_cv
uav1_speed: 1.0
z_amplitude: 0.5
capture_success: True
capture_time: 11.876s
min_distance_3d: 0.166m
terminal_mean_abs_z_error: 0.091m
terminal_mean_prediction_error_3d: 0.915m
p90_prediction_error_3d: 1.973m
max_prediction_error_3d: 6.450m
```

```text
log: ~/uav_intercept_logs/circle_z_sine_kalman_ca_20260525_210936.csv
target_mode: circle_z_sine
estimator_model: kalman_ca
uav1_speed: 1.0
z_amplitude: 0.5
capture_success: True
capture_time: 25.804s
min_distance_3d: 0.237m
terminal_mean_abs_z_error: 0.019m
terminal_mean_prediction_error_3d: 1.114m
terminal_mean_reachable_margin: 0.433s
```

Interpretation: the current setup can intercept in manual tests at `uav1_speed:=1.0`. CV is still the default because its prediction is less aggressive. CA is usable after conservative acceleration limiting, but it should remain a comparison model until repeated-run results show it is more reliable than CV.

## Latest Tuning Log

The recent tuning sequence converged on estimator-focused prediction plus a slightly more robust capture gate:

1. CV became the default formal model. Its velocity estimate uses a short observation fit, smoothing, Kalman velocity blending, and speed clamps. This reduced long extrapolation errors during turns without using `/uav1` trajectory script parameters.
2. CA was kept as a formal model but made conservative. `kf_process_noise_acc:=0.05`, `kf_accel_blend:=0.02`, acceleration clamps, and `ca_accel_prediction_horizon:=1.0` prevent acceleration noise from dominating a 3-second prediction.
3. Vertical terminal behavior was improved in guidance. Z control now uses the target-height error term, vertical correction remains active while yaw alignment is happening, and the main estimator launch uses `Kp_z:=0.75 az_max:=1.0` after figure-8 CA logs showed close XY passes limited by Z error.
4. Yaw gating was softened with `yaw_gate_xy_scale:=0.4` and skipped near the guidance point so `/uav0` does not freeze horizontally during close-range pursuit.
5. The capture gate was adjusted after logs showed close passes near `0.28m` and figure-8 contacts near `0.19m` that could still wait for hold time before stopping. The estimator launch now uses `capture_distance:=0.35`, `hard_capture_distance:=0.25`, `hold_time:=0.15`, and `capture_slowdown_distance:=0.35`; `capture_decision_node.py` uses `<=` for the normal boundary and immediately latches success inside the hard boundary. Slowdown now starts at the normal capture boundary instead of outside it, so `/uav0` does not bleed speed while still just outside capture range.

Current tuning conclusion: no further tuning is needed for the current prediction-focused checkpoint. The last useful figure-8 CA run reached `capture_success=True`, `min_distance_3d=0.235m`, `terminal_mean_abs_z_error=0.024m`, and a valid `0.35m` capture window. Keep these parameters fixed and use future tests for validation, not continuous retuning.

## Next Step

The project is paused at a stable prediction-focused checkpoint. Do not tune parameters again unless repeated validation logs show a consistent failure mode. Recommended next work:

1. Keep `kalman_cv` as the default formal model.
2. Use `kalman_ca` for sharper turn scenarios such as `figure8_3d`.
3. Save new tests and summarize `~/uav_intercept_logs` with `summarize_prediction_results.py`.
4. Compare `capture_success`, `capture_time`, `terminal_min_distance_3d`, `terminal_mean_abs_z_error`, and `terminal_mean_prediction_error_3d`.

Only tune again if a repeated pattern appears across multiple valid logs:

- Large `terminal_mean_prediction_error_3d`: tune estimator parameters.
- Negative `terminal_mean_reachable_margin`: tune time-to-go or reachable speed assumptions.
- Large signed Z error with weak `v_cmd_z`: tune vertical guidance.
- Close pass under `capture_distance` without success: tune capture gate or hold time.
- Good prediction but large miss distance: tune guidance gains or slowdown behavior.

## Terminal Bias Analysis

The current system can intercept, and the latest CA run reduced terminal height error to the centimeter scale. If a future run shows terminal bias again, do not solve it by giving the estimator the `/uav1` motion-script parameters. Use the last terminal window first.

`analyze_prediction_log.py` uses `--terminal-window 3.0` by default:

- If `capture_success=True`, it analyzes the 3 seconds before `capture_time`.
- If `capture_success=False`, it analyzes the final 3 seconds of the log.

Key terminal metrics:

- `terminal_mean_prediction_error_3d`: large values point to estimator prediction error.
- `terminal_mean_reachable_margin`: negative values mean the intercept point is too aggressive or unreachable.
- `terminal_mean_distance_z_signed`: `uav1_z - uav0_z`; positive means `/uav0` is below `/uav1`, negative means `/uav0` is above.
- `terminal_mean_abs_z_error`: large values with small `terminal_mean_v_cmd_z` suggest vertical correction is weak or gated.
- `terminal_mean_distance_xy`: small XY error with large Z error means vertical tracking is the limiting factor.
- `close_xy_below_ratio` and `close_xy_max_underpass_z`: measure whether `/uav0` repeatedly passes below `/uav1` inside the terminal window when `distance_xy < 0.5m`.
- yaw fields are summarized only when the CSV contains yaw columns.

The analyzer writes a `[Diagnosis]` section with simple rule-based hints:

- prediction error dominated
- unreachable/aggressive intercept point
- weak or gated vertical correction
- horizontal capture good but vertical tracking limiting
- guidance/control terminal behavior limiting capture

Recommended workflow:

1. Run the same scene with `kalman_cv` and `kalman_ca`.
2. Analyze each CSV with `analyze_prediction_log.py`.
3. Use terminal window metrics to identify the bias source.
4. Tune only the relevant small parameter set.
5. Run `run_kalman_tuning_sweep.sh` for a controlled follow-up sweep.

Recommended single-run commands:

```bash
roslaunch uav_truth_tracker intercept_estimator_velocity_chase.launch \
  target_mode:=circle_z_sine \
  uav1_speed:=1.0 \
  z_amplitude:=0.5 \
  estimator_model:=kalman_cv
```

```bash
roslaunch uav_truth_tracker intercept_estimator_velocity_chase.launch \
  target_mode:=circle_z_sine \
  uav1_speed:=1.0 \
  z_amplitude:=0.5 \
  estimator_model:=kalman_ca
```

```bash
rosrun uav_truth_tracker analyze_prediction_log.py \
  ~/uav_intercept_logs/<log>.csv \
  --vxy-max 2.0 \
  --vz-max 0.8 \
  --terminal-window 3.0
```

Batch comparison:

```bash
rosrun uav_truth_tracker run_estimator_model_tests.sh
```

Default batch settings:

- `target_mode:=circle_z_sine`
- `uav1_speed:=1.0`
- `z_amplitude:=0.5`
- formal models: `kalman_cv`, `kalman_ca`
- optional debug models with `--include-debug`: `current`, `linear_observation`, `learned_circle`, `circle_configured_debug`

Recommended experiment table:

| estimator_model | target_mode | success_rate | capture_time | mean_prediction_error_3d | max_prediction_error_3d | min_distance_3d |
| --- | --- | --- | --- | --- | --- | --- |
| kalman_cv | circle_z_sine | 1/1 reference run | 11.876s | 1.539m | 6.450m | 0.166m |
| kalman_ca | circle_z_sine | 1/1 reference run | 25.804s | 1.719m | 6.266m | 0.237m |

Debug/legacy table, not formal results:

| estimator_model | target_mode | role | capture_time | mean_prediction_error_3d | min_distance_3d |
| --- | --- | --- | --- | --- | --- |
| current | circle_z_sine | legacy baseline | | | |
| linear_observation | circle_z_sine | legacy baseline | | | |
| learned_circle | circle_z_sine | legacy periodic baseline | | | |
| circle_configured_debug | circle_z_sine | cheating upper-bound only | | | |

Summarize a log directory:

```bash
rosrun uav_truth_tracker summarize_prediction_results.py ~/uav_intercept_logs
```

Run the first-stage Kalman reachability sweep:

```bash
rosrun uav_truth_tracker run_kalman_tuning_sweep.sh
```

Print second-stage Kalman parameter candidate commands:

```bash
rosrun uav_truth_tracker run_kalman_tuning_sweep.sh --print-kalman-grid
```
