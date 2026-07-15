#!/bin/bash
# ============================================================================
# airground_takeoff.sh - ONE Gazebo with BOTH the real PX4+EGO UAV (flying &
# scanning) and the UGV (at init), feeding an EGOCENTRIC online elevation map
# centered on the UGV's odom (live UAV LiDAR + real MAVROS pose covariance).
#
# FULL-EGOCENTRIC framework: the UGV runs DLIO LiDAR-inertial odometry (no GPS /
# no global EKF / no `map` frame); `odom` is the sole authoritative frame. The UAV
# map is tied to the UGV via a capture-once-then-latch anchor (odom->uav0/map_local
# from a one-shot ground-truth snapshot) instead of a static map->uav0/map_local
# TF, and the UAV pose covariance is expressed in odom.
#
# Sequence (single unpause for BOTH robots):
#   Gazebo paused -> set physics 0.005/200Hz -> spawn PX4 (paused) -> wait for
#   PX4 link -> spawn UGV egocentric (paused, -J arm-stow HELD) -> unpause ->
#   UGV DLIO -> MAVROS -> airground_egocentric.launch egocentric:=true (UAV
#   mapping + anchor + egocentric map).
# Standalone one_key_takeoff.sh is left untouched.
# ============================================================================

echo "[airground] cleaning stale processes ..."
killall -9 gzserver gzclient rosmaster roscore px4 mavros 2>/dev/null
pkill -9 -f "mavros_node" 2>/dev/null || true
pkill -9 -f "px4_bridge.py" 2>/dev/null || true
pkill -9 -f "roslaunch .*airground_egocentric.launch" 2>/dev/null || true
pkill -9 -f "roslaunch .*lidar_odometry.launch" 2>/dev/null || true
pkill -9 -f "roslaunch .*cmu_planner.launch" 2>/dev/null || true
pkill -9 -f "dlio_odom_node" 2>/dev/null || true
pkill -9 -f "ugv_target_tour" 2>/dev/null
pkill -9 -f "roslaunch .*move_group.launch" 2>/dev/null
pkill -9 -f "roslaunch .*manipulation.launch" 2>/dev/null
pkill -9 -f "roslaunch .*husky_ur5_gpd.launch" 2>/dev/null
pkill -9 -f "grasp_task.py|gpd_grasp_server.py|landmine_detector.py|detect_grasps|mine_align.py" 2>/dev/null || true
pkill -9 -f "airground_anchor_latch" 2>/dev/null || true
pkill -9 -f "roslaunch .*spawn_outdoor_city.launch" 2>/dev/null || true
pkill -9 -f "roslaunch .*forest_uav_mapping.launch" 2>/dev/null || true
pkill -9 -f "roslaunch .*px4_spawn_existing_gazebo.launch" 2>/dev/null || true
pkill -9 -f "roslaunch .*mavros.*/px4.launch" 2>/dev/null || true
pkill -9 -f "roslaunch .*gazebo_ros .*empty_world.launch" 2>/dev/null || true
sleep 3

if ! command -v gnome-terminal &>/dev/null; then
    echo "[airground] need gnome-terminal: sudo apt-get install gnome-terminal"; exit 1
fi

echo "[airground] starting air-ground co-sim (UAV mapping + static UGV, UGV-centric map) ..."

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
EGO_WS=${EGO_WS:-$SCRIPT_DIR}
UGV_WS=${UGV_WS:-$SCRIPT_DIR}
PX4_GAZEBO_GUI=${PX4_GAZEBO_GUI:-true}
PX4_SIM_SPEED_FACTOR=${PX4_SIM_SPEED_FACTOR:-1}
START_RVIZ=${START_RVIZ:-true}
UGV_WORLD=${UGV_WORLD:-$SCRIPT_DIR/src/mobile_manipulator/worlds/outdoor_city.world}
GAZEBO_WORLD=${GAZEBO_WORLD:-$UGV_WORLD}
# Terrain-mapping LiDAR: velodyne FOV widened DOWNWARD to -90deg so it sees the
# ground directly below from altitude (the stock +/-30deg light model can't reach
# flat ground above ~8 m -> holes at the 12 m flight height). Standalone forest
# keeps the ..._mapping_light model.
LIDAR_SDF=${LIDAR_SDF:-$SCRIPT_DIR/models/iris_depth_camera_lidar_terrain/model.sdf}
MAVROS_PX4_LAUNCH=${MAVROS_PX4_LAUNCH:-/opt/ros/noetic/share/mavros/launch/px4.launch}
UGV_ROS_PACKAGE_PATH=$UGV_WS/src:$UGV_WS/src/robotiq
GAZEBO_LOAD_WAIT=${GAZEBO_LOAD_WAIT:-10}
PX4_WAIT=${PX4_WAIT:-8}
UGV_SPAWN_WAIT=${UGV_SPAWN_WAIT:-6}
MAVROS_WAIT=${MAVROS_WAIT:-5}
ROS_WAIT=${ROS_WAIT:-20}
UAV_POINTS_MAP_DIR=${UAV_POINTS_MAP_DIR:-$HOME/pointcloud_maps}
# UAV spawn pose in Gazebo. SPAWN_Z is just the drop-in height; the UAV settles to
# the ground and PX4 zeroes MAVROS-local z there.
SPAWN_X=${SPAWN_X:-0.0}
SPAWN_Y=${SPAWN_Y:--18.0}
SPAWN_Z=${SPAWN_Z:-1.5}
SPAWN_YAW=${SPAWN_YAW:-1.5707963}
# map -> uav0/map_local Z offset (DISTINCT from SPAWN_Z): where MAVROS-local z=0
# sits in odom. PX4's local-z origin is the GROUND (= UGV ground plane, odom 0), so
# this is 0, NOT the spawn altitude. A nonzero value floats the terrain above the UGV.
MAP_LOCAL_Z=${MAP_LOCAL_Z:-0.0}
FLIGHT_H=${FLIGHT_H:-12.0}
PHYSICS_STEP=${PHYSICS_STEP:-0.005}
PHYSICS_RATE=${PHYSICS_RATE:-200.0}

if [[ ! -f "$MAVROS_PX4_LAUNCH" ]]; then
  echo "[airground] MAVROS PX4 launch not found: $MAVROS_PX4_LAUNCH"
  echo "            sudo apt install ros-noetic-mavros ros-noetic-mavros-extras"; exit 1
fi
if [[ "$GAZEBO_WORLD" = /* && ! -f "$GAZEBO_WORLD" ]]; then
  echo "[airground] Gazebo world not found: $GAZEBO_WORLD"; exit 1
fi
if [[ "$LIDAR_SDF" = /* && ! -f "$LIDAR_SDF" ]]; then
  echo "[airground] UAV SDF not found: $LIDAR_SDF"; exit 1
fi
mkdir -p "$UAV_POINTS_MAP_DIR"

GZ_MODEL_PATH="$EGO_WS/models:$EGO_WS/src/uav_truth_tracker/models:$UGV_WS/src/gazebo_models_worlds_collection/models:$UGV_WS/src/mobile_manipulator/gazebo_models:$UGV_WS/src/mobile_manipulator/models:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$HOME/.gazebo/gazebo_models:$HOME/.gazebo/models"

# 1. Gazebo (paused), with the WORKSPACE sourced so the UGV's package:// meshes
#    (husky/ur5/robotiq) resolve in the shared server.
gnome-terminal --tab --title="1_Gazebo" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
source /usr/share/gazebo/setup.sh && \
export PX4_SIM_SPEED_FACTOR='$PX4_SIM_SPEED_FACTOR' && \
export GAZEBO_MODEL_DATABASE_URI='' && \
export GAZEBO_PLUGIN_PATH=\"$PX4_DIR/build/px4_sitl_default/build_gazebo-classic:\$GAZEBO_PLUGIN_PATH\" && \
export GAZEBO_MODEL_PATH=\"$GZ_MODEL_PATH:\$GAZEBO_MODEL_PATH\" && \
export GAZEBO_RESOURCE_PATH=\"$EGO_WS/worlds:$UGV_WS/src/mobile_manipulator/worlds:\$GAZEBO_RESOURCE_PATH\" && \
roslaunch gazebo_ros empty_world.launch world_name:='$GAZEBO_WORLD' paused:=true use_sim_time:=true gui:='${PX4_GAZEBO_GUI}' 2>&1 | grep -v \"parser.cc\"; exec bash"

echo "[airground] waiting ${GAZEBO_LOAD_WAIT}s for Gazebo to load ..."
sleep "$GAZEBO_LOAD_WAIT"

# 1.4 Physics step for non-lockstep SITL EKF convergence (RTF~1.0). Set BEFORE PX4.
echo "[airground] setting physics step=$PHYSICS_STEP rate=$PHYSICS_RATE (non-lockstep SITL) ..."
( source /opt/ros/noetic/setup.bash && \
  rosservice call --wait /gazebo/set_physics_properties "
time_step: $PHYSICS_STEP
max_update_rate: $PHYSICS_RATE
gravity: {x: 0.0, y: 0.0, z: -9.8}
ode_config:
  auto_disable_bodies: false
  sor_pgs_precon_iters: 0
  sor_pgs_iters: 50
  sor_pgs_w: 1.3
  sor_pgs_rms_error_tol: 0.0
  contact_surface_layer: 0.001
  contact_max_correcting_vel: 100.0
  cfm: 0.0
  erp: 0.2
  max_contacts: 20
" ) 2>/dev/null && echo "[airground] physics set" || echo "[airground] set_physics_properties failed"

# 1.5 Spawn PX4 + UAV into the running (still paused) Gazebo.
gnome-terminal --tab --title="2_PX4_Spawn_UAV" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
cd '$PX4_DIR' && \
export GAZEBO_MODEL_DATABASE_URI='' && \
source Tools/simulation/gazebo-classic/setup_gazebo.bash \$(pwd) \$(pwd)/build/px4_sitl_default && \
export GAZEBO_MODEL_PATH=\"$GZ_MODEL_PATH:\$GAZEBO_MODEL_PATH\" && \
export GAZEBO_RESOURCE_PATH=\"$EGO_WS/worlds:$UGV_WS/src/mobile_manipulator/worlds:\$GAZEBO_RESOURCE_PATH\" && \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd):\$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic && \
export PX4_SIM_SPEED_FACTOR='$PX4_SIM_SPEED_FACTOR' && \
roslaunch uav_truth_tracker px4_spawn_existing_gazebo.launch vehicle:=iris_depth_camera sdf:='$LIDAR_SDF' x:='$SPAWN_X' y:='$SPAWN_Y' z:='$SPAWN_Z' Y:='$SPAWN_YAW' sim_speed_factor:='$PX4_SIM_SPEED_FACTOR' 2>&1 | grep -v \"parser.cc\"; exec bash"

echo "[airground] waiting ${PX4_WAIT}s for PX4 to link Gazebo plugin (TCP 4560) ..."
sleep "$PX4_WAIT"

# 1.6 Spawn the UGV WHILE STILL PAUSED so the -J arm-stow pose is held (mirrors
#     outdoor_mapping.launch). spawn_unpause:=false defers the unpause to step 2
#     so both the UGV controllers and the PX4 lockstep handshake resume together.
#     egocentric:=true drops the GPS/global-EKF chain (DLIO owns odom->base_link).
echo "[airground] spawning egocentric UGV (paused, arm-stow held) ..."
gnome-terminal --tab --title="3_UGV" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
export ROS_PACKAGE_PATH='$UGV_ROS_PACKAGE_PATH':\$ROS_PACKAGE_PATH && \
export GAZEBO_MODEL_PATH=\"$GZ_MODEL_PATH:\$GAZEBO_MODEL_PATH\" && \
roslaunch mobile_manipulator spawn_outdoor_city.launch start_gazebo:=false spawn_unpause:=false egocentric:=true; exec bash"

echo "[airground] waiting ${UGV_SPAWN_WAIT}s for the UGV model to load ..."
sleep "$UGV_SPAWN_WAIT"

# 2. Single unpause -> UGV controllers start (hold the arm) + PX4 lockstep handshake.
echo "[airground] unpausing Gazebo (UGV controllers + PX4 handshake) ..."
( source /opt/ros/noetic/setup.bash && \
  rosservice call --wait /gazebo/unpause_physics "{}" ) 2>/dev/null \
  && echo "[airground] unpaused" || echo "[airground] unpause failed: run rosservice call /gazebo/unpause_physics"
sleep 2

# 2.5 UGV LiDAR-inertial odometry (DLIO). Owns odom->base_link and publishes
#     /state_estimation, which the egocentric anchor (odom->uav0/map_local) and the
#     elevation map (tracks base_link) both need. Needs the UGV Velodyne, so it
#     runs after unpause; the GPU Velodyne sensor needs a GL context (GUI/DISPLAY).
echo "[airground] starting UGV DLIO odometry ..."
gnome-terminal --tab --title="3b_UGV_LIO" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
export ROS_PACKAGE_PATH='$UGV_ROS_PACKAGE_PATH':\$ROS_PACKAGE_PATH && \
roslaunch mobile_manipulator lidar_odometry.launch odom_source:=dlio self_filter:=true; exec bash"
echo "[airground] waiting 8s for DLIO to initialize (odom->base_link) ..."
sleep 8

# 2.6 UGV CMU local planner + target tour, matching egocentric_nav.launch's Phase-B
#     stack: nav=cmu, cost_source=elevation, global_planner=far.
#
#     This node OWNS the one and only elevation_mapping (see start_elevation:=false
#     passed to airground_egocentric.launch below). It fuses TWO input sources into a
#     single map: the UGV's own self-filtered VLP-16, and — with uav_prior:=true — the
#     UAV's aerial LiDAR, each with its own covariance lever arm. The UGV only sees
#     line of sight; the UAV at 12 m sees over the hill, which is the entire point of
#     the air-ground stack.
#
#     uav_prior subscribes to /uav0/mapping/velodyne_points_gated, so airground_
#     egocentric.launch MUST run with enable_gate:=true or that topic never exists and
#     the aerial source stays silent. It also needs /uav/pose_cov, still published there.
#
#     The 40 m default map cannot even reach the hill at y=30, so the aerial prior would
#     have nowhere to land: enlarge to 120 m and coarsen to 0.35 m (118k cells).
#
#     FAR (global_planner:=far) owns /way_point and needs cost_source:=elevation, whose
#     postprocessor derives the traversability layer feeding /terrain_map_ext. The launch
#     already refuses to start terrain_analysis_ext alongside it (they would both publish
#     /terrain_map_ext and fight).
#
#     The UGV stays PUT until you call /ugv/start_tour (planner idle with no goal,
#     target_tour waits for the trigger), so it is safe to start during the UAV mapping
#     epoch. Detected targets arrive on /detected_targets (the UAV-detector seam).
UGV_NAV=${UGV_NAV:-true}
UGV_COST_SOURCE=${UGV_COST_SOURCE:-elevation}
UGV_GLOBAL_PLANNER=${UGV_GLOBAL_PLANNER:-far}
UGV_UAV_PRIOR=${UGV_UAV_PRIOR:-true}
UGV_MAP_SIZE=${UGV_MAP_SIZE:-120}
UGV_MAP_RES=${UGV_MAP_RES:-0.35}
# GRASP: on arrival at a detected mine, the tour calls /grasp/execute (which does its own
# look + detect + GPD) and advances after. This is the NO-ALIGNMENT path -- it grasps from
# wherever nav parks, so it only lands the pick when the mine falls in the arm's reachable band
# (x in [0.60, 1.05] from base_link). The visual fine-alignment that would guarantee that is a
# separate, not-yet-working stage; this wires the whole air-ground pipeline end to end anyway.
UGV_GRASP=${UGV_GRASP:-true}
GRASP_SOURCE=${GRASP_SOURCE:-gpd}
GRASP_DETECTOR=${GRASP_DETECTOR:-color}
if [ "$UGV_NAV" = "true" ]; then
  echo "[airground] starting UGV CMU planner (cost=$UGV_COST_SOURCE global=$UGV_GLOBAL_PLANNER uav_prior=$UGV_UAV_PRIOR) + target tour ..."
  gnome-terminal --tab --title="3c_UGV_NAV" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
export ROS_PACKAGE_PATH='$UGV_ROS_PACKAGE_PATH':\$ROS_PACKAGE_PATH && \
roslaunch mobile_manipulator cmu_planner.launch \
  cost_source:='$UGV_COST_SOURCE' \
  global_planner:='$UGV_GLOBAL_PLANNER' \
  uav_prior:='$UGV_UAV_PRIOR' \
  map_size:='$UGV_MAP_SIZE' \
  map_resolution:='$UGV_MAP_RES' \
  grasp_on_arrival:='$UGV_GRASP' \
  maxSpeed:=1.0; exec bash"
  echo "[airground] UGV nav up (idle until: rosservice call /ugv/start_tour)"
fi

# 2.7 Derive who owns elevation_mapping, so the two launches cannot both start one.
#     cmu_planner starts a node named `elevation_mapping` iff cost_source=elevation;
#     airground_egocentric.launch starts one named `elevation_mapping` too. Exactly one
#     may exist. Derive rather than hard-code, so UGV_NAV=false or
#     UGV_COST_SOURCE=terrain_analysis still leaves someone owning a map.
if [ "$UGV_NAV" = "true" ] && [ "$UGV_COST_SOURCE" = "elevation" ]; then
  AG_START_ELEVATION=false      # cmu_planner owns the (UGV + UAV fused) map
else
  AG_START_ELEVATION=true       # nobody else would: fall back to the UAV-only map
fi
# The gate is what publishes the topic uav_prior subscribes to, so it must be on
# whenever cmu_planner is fusing the aerial source.
if [ "$AG_START_ELEVATION" = "false" ] && [ "$UGV_UAV_PRIOR" = "true" ]; then
  AG_ENABLE_GATE=${AG_ENABLE_GATE:-true}
else
  AG_ENABLE_GATE=${AG_ENABLE_GATE:-false}
fi
echo "[airground] elevation map owner: $([ "$AG_START_ELEVATION" = "false" ] && echo 'cmu_planner (UGV+UAV fused)' || echo 'airground_egocentric (UAV only)') ; gate=$AG_ENABLE_GATE"

# 2.8 MANIPULATION: MoveIt move_group + the grasp pipeline (perception -> GPD -> MTC pick).
#     Started here because the pick needs the spawned robot + its ros_control controllers (up
#     since the unpause) and DLIO's odom frame (the lift goes along odom +Z). move_group loads
#     NO robot_description of its own -- the spawner already owns it; a second one could diverge.
#     grasp_source:=gpd runs GPD behind the /get_grasps seam; detector:=color because
#     landmine.onnx cannot see this flat-shaded prop (a domain gap, not a bug).
#     The tour drives /grasp/execute on arrival (grasp_on_arrival above).
if [ "$UGV_GRASP" = "true" ]; then
  echo "[airground] starting MoveIt move_group ..."
  gnome-terminal --tab --title="7a_MoveGroup" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
export ROS_PACKAGE_PATH='$UGV_ROS_PACKAGE_PATH':\$ROS_PACKAGE_PATH && \
roslaunch husky_ur5_moveit_config move_group.launch \
  load_robot_description:=false \
  allow_trajectory_execution:=true \
  moveit_controller_manager:=simple \
  publish_monitored_planning_scene:=true; exec bash"
  echo "[airground] waiting 12s for move_group ..."
  sleep 12

  echo "[airground] starting grasp pipeline (grasp_source=$GRASP_SOURCE detector=$GRASP_DETECTOR) ..."
  gnome-terminal --tab --title="7b_Grasp" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
export ROS_PACKAGE_PATH='$UGV_ROS_PACKAGE_PATH':\$ROS_PACKAGE_PATH && \
roslaunch grasp_mtc manipulation.launch \
  grasp_source:='$GRASP_SOURCE' \
  detector:='$GRASP_DETECTOR' \
  planning_frame:=base_link; exec bash"
  echo "[airground] grasp pipeline up (tour will call /grasp/execute on arrival)"
fi

# 3. MAVROS.
gnome-terminal --tab --title="4_MAVROS" -- bash -c "source /opt/ros/noetic/setup.bash && roslaunch '$MAVROS_PX4_LAUNCH' fcu_url:=\"udp://:14540@127.0.0.1:14580\"; exec bash"
echo "[airground] waiting ${MAVROS_WAIT}s for MAVROS ..."
sleep "$MAVROS_WAIT"

# 4. ROS layer: UAV mapping (prefixed) + the anchor + the aerial cloud feed.
#    start_elevation:=false — cmu_planner (step 2.6) owns the single elevation_mapping
#    node. Starting one here too is a NAME COLLISION and roslaunch silently kills one of
#    the two maps. What still runs here and is still load-bearing: the odom->uav0/map_local
#    anchor, the altitude gate, and uav_pose_cov_publisher (/uav/pose_cov), which supplies
#    the per-source covariance lever arm for the UAV input in cmu_planner's map.
#
#    enable_gate:=true — REQUIRED by uav_prior, whose config subscribes specifically to
#    /uav0/mapping/velodyne_points_gated. With the gate off that topic never exists and the
#    aerial prior silently contributes nothing. (This launch defaults the gate OFF for its
#    own standalone use; the air-ground path needs it ON.)
echo "[airground] starting airground_egocentric.launch (UAV mapping + anchor; cmu_planner owns the map) ..."
gnome-terminal --tab --title="5_AirGround_ROS" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
export ROS_PACKAGE_PATH='$UGV_ROS_PACKAGE_PATH':\$ROS_PACKAGE_PATH && \
export PYTHONPATH='$EGO_WS':\$PYTHONPATH && \
roslaunch mobile_manipulator airground_egocentric.launch \
  egocentric:=true \
  rviz:='$START_RVIZ' \
  start_elevation:='$AG_START_ELEVATION' \
  enable_gate:='$AG_ENABLE_GATE' \
  uav_spawn_x:='$SPAWN_X' uav_spawn_y:='$SPAWN_Y' uav_spawn_z:='$MAP_LOCAL_Z' \
  flight_height:='$FLIGHT_H' \
  ; exec bash"

echo "[airground] waiting ${ROS_WAIT}s for ROS nodes ..."
sleep "$ROS_WAIT"

# 5. Takeoff bridge - arm + OFFBOARD so EGO can fly the UAV.
gnome-terminal --tab --title="6_Takeoff" -- bash -c "source /opt/ros/noetic/setup.bash && source '$EGO_WS/devel/setup.bash' && python3 -u '$EGO_WS/px4_bridge.py' _require_depth_before_takeoff:=false; exec bash"

echo "[airground] startup sequence done (EGOCENTRIC framework)."
echo "  UAV spawn=($SPAWN_X,$SPAWN_Y,$SPAWN_Z) flight_height=${FLIGHT_H}m ; UGV egocentric (DLIO odom)"
echo "  egocentric: NO map frame; DLIO owns odom->base_link; anchor latches odom->uav0/map_local"
echo "  elevation map: /elevation_mapping/elevation_map_postprocessed (frame=odom)"
echo "  UAV pose error: /uav/pose_cov (real MAVROS covariance -> Sigma_{odom->uav}, world_frame=odom)"
echo "  UGV nav: detected targets -> /detected_targets (mobile_manipulator/WorldTarget)"
echo "           after the UAV finishes: rosservice call /ugv/start_tour  (UGV tours the targets)"
echo "  UGV grasp on arrival: $UGV_GRASP  (grasp_source=$GRASP_SOURCE, detector=$GRASP_DETECTOR)"
echo "           the tour calls /grasp/execute at each mine; NO fine-alignment yet, so the pick"
echo "           only lands when nav parks the mine inside x in [0.60, 1.05] of base_link"
echo "  checks: rostopic echo -n1 /mavros/state ; rosrun rqt_tf_tree rqt_tf_tree"
echo "          rosrun tf2_ros tf2_echo odom uav0/map_local   # anchor latched"
echo "          rosrun tf2_ros tf2_echo odom base_link        # DLIO (no map frame in tree)"
echo "  NOTE: RViz '2D Nav Goal' flies the UAV ONLY; the UGV goal is /ugv/goal."
