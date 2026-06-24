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
pkill -9 -f "dlio_odom_node" 2>/dev/null || true
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

# 3. MAVROS.
gnome-terminal --tab --title="4_MAVROS" -- bash -c "source /opt/ros/noetic/setup.bash && roslaunch '$MAVROS_PX4_LAUNCH' fcu_url:=\"udp://:14540@127.0.0.1:14580\"; exec bash"
echo "[airground] waiting ${MAVROS_WAIT}s for MAVROS ..."
sleep "$MAVROS_WAIT"

# 4. ROS layer: UAV mapping (prefixed) + egocentric elevation map (UGV already up).
echo "[airground] starting airground_egocentric.launch (UAV mapping + UGV-centric elevation map) ..."
gnome-terminal --tab --title="5_AirGround_ROS" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
export ROS_PACKAGE_PATH='$UGV_ROS_PACKAGE_PATH':\$ROS_PACKAGE_PATH && \
export PYTHONPATH='$EGO_WS':\$PYTHONPATH && \
roslaunch mobile_manipulator airground_egocentric.launch \
  egocentric:=true \
  rviz:='$START_RVIZ' \
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
echo "  checks: rostopic echo -n1 /mavros/state ; rosrun rqt_tf_tree rqt_tf_tree"
echo "          rosrun tf2_ros tf2_echo odom uav0/map_local   # anchor latched"
echo "          rosrun tf2_ros tf2_echo odom base_link        # DLIO (no map frame in tree)"
