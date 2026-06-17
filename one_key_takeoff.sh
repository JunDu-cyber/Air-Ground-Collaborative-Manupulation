#!/bin/bash

echo "🧹 正在清理僵尸进程，为起飞腾出跑道..."
killall -9 gzserver gzclient rosmaster roscore px4 mavros 2>/dev/null
pkill -9 -f "mavros_node" 2>/dev/null || true
pkill -9 -f "px4_bridge.py" 2>/dev/null || true
pkill -9 -f "roslaunch .*forest_uav_mapping.launch" 2>/dev/null || true
pkill -9 -f "roslaunch .*px4_spawn_existing_gazebo.launch" 2>/dev/null || true
pkill -9 -f "roslaunch .*mavros.*/px4.launch" 2>/dev/null || true
pkill -9 -f "roslaunch .*gazebo_ros .*empty_world.launch" 2>/dev/null || true
sleep 3

echo "🚀 开始一键召唤仿真环境..."

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
# EGO_WS 指向本仓库（UAV 包已集成）
EGO_WS=${EGO_WS:-$SCRIPT_DIR}
UGV_WS=${UGV_WS:-$SCRIPT_DIR}
PX4_GAZEBO_GUI=${PX4_GAZEBO_GUI:-false}
START_RVIZ=${START_RVIZ:-true}
# 默认使用村庄地图
FOREST_WORLD=${FOREST_WORLD:-$SCRIPT_DIR/worlds/forest_corridor.world}
UGV_WORLD=${UGV_WORLD:-$SCRIPT_DIR/src/mobile_manipulator/worlds/outdoor_city.world}
PX4_OUTDOOR_WORLD=${PX4_OUTDOOR_WORLD:-$SCRIPT_DIR/worlds/outdoor_city_px4.world}
GAZEBO_WORLD=${GAZEBO_WORLD:-$UGV_WORLD}
LIDAR_SDF=${LIDAR_SDF:-$SCRIPT_DIR/models/iris_depth_camera_lidar_mapping_light/model.sdf}
TF2_STATIC_PUBLISHER=${TF2_STATIC_PUBLISHER:-/opt/ros/noetic/lib/tf2_ros/static_transform_publisher}
MAVROS_PX4_LAUNCH=${MAVROS_PX4_LAUNCH:-/opt/ros/noetic/share/mavros/launch/px4.launch}
START_MAVROS=${START_MAVROS:-auto}
REQUIRE_MAVROS_FOR_EGO_MAPPING=${REQUIRE_MAVROS_FOR_EGO_MAPPING:-true}
# Gazebo odom bridge: /gazebo/model_states → /mavros/local_position/odom
# Mapping-only fallback. Default is false because UAV flight requires MAVROS
# OFFBOARD control; Gazebo odom alone can plan/map but cannot move PX4.
USE_GAZEBO_ODOM_BRIDGE=${USE_GAZEBO_ODOM_BRIDGE:-false}
START_UGV_MARKER=${START_UGV_MARKER:-true}
UGV_URDF=${UGV_URDF:-$UGV_WS/src/husky_ur5_moveit_config/config/gazebo_husky_ur5.urdf}
UGV_ROS_PACKAGE_PATH=$UGV_WS/src:$UGV_WS/src/robotiq
START_DEPTH_PREPROCESS=${START_DEPTH_PREPROCESS:-true}
GAZEBO_LOAD_WAIT=${GAZEBO_LOAD_WAIT:-10}
# ENABLE_OCTOMAP default is set later, after HAS_ODOMETRY is known
UAV_POINTS_TARGET_FRAME=${UAV_POINTS_TARGET_FRAME:-map}
UAV_POINTS_RAW_INPUT_TOPIC=${UAV_POINTS_RAW_INPUT_TOPIC:-/uav0/velodyne_points_raw}
UAV_POINTS_INTERMEDIATE_TOPIC=${UAV_POINTS_INTERMEDIATE_TOPIC:-/uav0/mapping/velodyne_points_sensor}
UAV_POINTS_OUTPUT_TOPIC=${UAV_POINTS_OUTPUT_TOPIC:-/uav0/mapping/points_world}
UAV_POINTS_RATE=${UAV_POINTS_RATE:-2.0}
UAV_POINTS_MIN_RANGE=${UAV_POINTS_MIN_RANGE:-0.35}
UAV_POINTS_MAX_RANGE=${UAV_POINTS_MAX_RANGE:-12.0}
UAV_POINTS_SENSOR_MAX_RANGE=${UAV_POINTS_SENSOR_MAX_RANGE:-16.0}
UAV_POINTS_MAX_RANGE_MARGIN=${UAV_POINTS_MAX_RANGE_MARGIN:-0.5}
UAV_POINTS_DROP_FAR_BOUNDARY=${UAV_POINTS_DROP_FAR_BOUNDARY:-true}
UAV_POINTS_RANGE_EDGE_MARGIN=${UAV_POINTS_RANGE_EDGE_MARGIN:-1.5}
UAV_POINTS_LEAF_SIZE=${UAV_POINTS_LEAF_SIZE:-0.1}
UAV_POINTS_RELAY_LEAF_SIZE=${UAV_POINTS_RELAY_LEAF_SIZE:-0.08}
UAV_POINTS_NEIGHBOR_VOXEL_SIZE=${UAV_POINTS_NEIGHBOR_VOXEL_SIZE:-0.18}
UAV_POINTS_MIN_NEIGHBORS=${UAV_POINTS_MIN_NEIGHBORS:-2}
UAV_POINTS_MAX_ANGULAR_VELOCITY=${UAV_POINTS_MAX_ANGULAR_VELOCITY:-0.60}
UAV_POINTS_MAX_LINEAR_VELOCITY=${UAV_POINTS_MAX_LINEAR_VELOCITY:-2.50}
UAV_POINTS_MAX_ODOM_ANGULAR_VELOCITY=${UAV_POINTS_MAX_ODOM_ANGULAR_VELOCITY:-0.60}
UAV_POINTS_MAX_IMU_AGE=${UAV_POINTS_MAX_IMU_AGE:-0.25}
UAV_POINTS_MAX_ODOM_AGE=${UAV_POINTS_MAX_ODOM_AGE:-0.25}
UAV_POINTS_GROUND_FILTER_ENABLED=${UAV_POINTS_GROUND_FILTER_ENABLED:-false}
UAV_POINTS_GROUND_KEEP_ABOVE=${UAV_POINTS_GROUND_KEEP_ABOVE:-0.10}
START_UAV_POINTS_RECORDER=${START_UAV_POINTS_RECORDER:-true}
UAV_POINTS_ACCUMULATED_TOPIC=${UAV_POINTS_ACCUMULATED_TOPIC:-/uav0/mapping/points_accumulated}
UAV_POINTS_MAP_DIR=${UAV_POINTS_MAP_DIR:-$HOME/pointcloud_maps}
UAV_POINTS_MAP_NAME=${UAV_POINTS_MAP_NAME:-uav_points_map}
UAV_POINTS_MAP_LEAF_SIZE=${UAV_POINTS_MAP_LEAF_SIZE:-0.15}
UAV_POINTS_MAP_MIN_OBSERVATIONS=${UAV_POINTS_MAP_MIN_OBSERVATIONS:-3}
UAV_POINTS_MAP_MIN_Z=${UAV_POINTS_MAP_MIN_Z:--999.0}
UAV_POINTS_MAP_MAX_Z=${UAV_POINTS_MAP_MAX_Z:-999.0}
UAV_POINTS_MAP_DROP_FLAT_GROUND=${UAV_POINTS_MAP_DROP_FLAT_GROUND:-false}
UAV_POINTS_MAP_GROUND_FILTER_MODE=${UAV_POINTS_MAP_GROUND_FILTER_MODE:-none}
UAV_POINTS_MAP_GROUND_Z=${UAV_POINTS_MAP_GROUND_Z:-0.0}
UAV_POINTS_MAP_GROUND_CLEARANCE=${UAV_POINTS_MAP_GROUND_CLEARANCE:-0.25}
UAV_POINTS_MAP_OBSTACLE_HEIGHT_ABOVE_GROUND=${UAV_POINTS_MAP_OBSTACLE_HEIGHT_ABOVE_GROUND:-0.30}
UAV_POINTS_MAP_GROUND_XY_CELL_SIZE=${UAV_POINTS_MAP_GROUND_XY_CELL_SIZE:-0.35}
UAV_POINTS_MAP_KEEP_TERRAIN=${UAV_POINTS_MAP_KEEP_TERRAIN:-false}
UAV_POINTS_MAP_TERRAIN_CELL_SIZE=${UAV_POINTS_MAP_TERRAIN_CELL_SIZE:-0.80}
UAV_POINTS_MAP_SAVE_INTERVAL=${UAV_POINTS_MAP_SAVE_INTERVAL:-30.0}
SPAWN_X=${SPAWN_X:-0.0}
SPAWN_Y=${SPAWN_Y:--18.0}
SPAWN_Z=${SPAWN_Z:-1.5}
SPAWN_YAW=${SPAWN_YAW:-1.5707963}
UGV_MARKER_X=${UGV_MARKER_X:-2.5}
UGV_MARKER_Y=${UGV_MARKER_Y:--18.0}
UGV_MARKER_Z=${UGV_MARKER_Z:-0.25}
UGV_MARKER_YAW=${UGV_MARKER_YAW:-0.0}

MAVROS_AVAILABLE=false
if [[ -f "$MAVROS_PX4_LAUNCH" ]]; then
  MAVROS_AVAILABLE=true
fi

if [[ "$START_MAVROS" = "auto" ]]; then
  START_MAVROS=$MAVROS_AVAILABLE
fi

if [[ "$START_MAVROS" = "true" && "$MAVROS_AVAILABLE" != "true" ]]; then
  echo "❌ 找不到 MAVROS PX4 launch: $MAVROS_PX4_LAUNCH"
  echo "   这套一键启动需要 MAVROS OFFBOARD 控制 UAV 起飞和响应 RViz 目标点。"
  echo "   请安装: sudo apt install ros-noetic-mavros ros-noetic-mavros-extras"
  echo "   只做静态建图调试时才手动设置: START_MAVROS=false USE_GAZEBO_ODOM_BRIDGE=true REQUIRE_MAVROS_FOR_EGO_MAPPING=false"
  exit 1
fi

if [[ -z "${HAS_ODOMETRY+x}" ]]; then
  HAS_ODOMETRY=$START_MAVROS
fi

if [[ "$START_MAVROS" != "true" && "$USE_GAZEBO_ODOM_BRIDGE" = "true" ]]; then
  echo "💡 Gazebo Odom Bridge 仅用于建图调试：有位姿但没有 MAVROS OFFBOARD 飞行控制"
  HAS_ODOMETRY=true
fi

if [[ "$START_MAVROS" != "true" && "$REQUIRE_MAVROS_FOR_EGO_MAPPING" = "true" && "$HAS_ODOMETRY" != "true" ]]; then
  echo "❌ START_MAVROS=$START_MAVROS，无里程计来源，不能进行 EGO 自动建图。"
  echo "   请选择: MAVROS 或 Gazebo Odom Bridge 提供位姿。"
  exit 1
fi

if [[ "$START_MAVROS" != "true" && "$HAS_ODOMETRY" = "true" ]]; then
  echo "🔌 Gazebo Odom Bridge 模式: 从 /gazebo/model_states 提供位姿，替代 MAVROS"
fi

# 有里程计就默认开启 OctoMap 建图 (用户可通过 ENABLE_OCTOMAP=false 强制关闭)
if [[ "$HAS_ODOMETRY" = "true" ]]; then
  ENABLE_OCTOMAP=${ENABLE_OCTOMAP:-true}
fi

START_TF_BRIDGE=${START_TF_BRIDGE:-$START_MAVROS}
START_EGO_PLANNER=${START_EGO_PLANNER:-$HAS_ODOMETRY}
START_TAKEOFF=${START_TAKEOFF:-$START_MAVROS}
START_UAV_POINTS_TO_UGV=${START_UAV_POINTS_TO_UGV:-$HAS_ODOMETRY}

if [[ "$START_MAVROS" = "true" ]]; then
  UAV_POINTS_IMU_TOPIC=${UAV_POINTS_IMU_TOPIC:-/mavros/imu/data}
  UAV_POINTS_ODOM_TOPIC=${UAV_POINTS_ODOM_TOPIC:-/mavros/local_position/odom}
  UAV_POINTS_STAMP_WITH_ODOM=${UAV_POINTS_STAMP_WITH_ODOM:-true}
elif [[ "$HAS_ODOMETRY" = "true" ]]; then
  # Gazebo Odom Bridge provides these same topics
  UAV_POINTS_IMU_TOPIC=${UAV_POINTS_IMU_TOPIC:-/mavros/imu/data}
  UAV_POINTS_ODOM_TOPIC=${UAV_POINTS_ODOM_TOPIC:-/mavros/local_position/odom}
  UAV_POINTS_STAMP_WITH_ODOM=${UAV_POINTS_STAMP_WITH_ODOM:-true}
else
  UAV_POINTS_IMU_TOPIC=${UAV_POINTS_IMU_TOPIC:-}
  UAV_POINTS_ODOM_TOPIC=${UAV_POINTS_ODOM_TOPIC:-}
  UAV_POINTS_STAMP_WITH_ODOM=false
  UAV_POINTS_MAX_ANGULAR_VELOCITY=0.0
  UAV_POINTS_MAX_LINEAR_VELOCITY=0.0
  UAV_POINTS_MAX_ODOM_ANGULAR_VELOCITY=0.0
fi

if [[ "$GAZEBO_WORLD" = /* && ! -f "$GAZEBO_WORLD" ]]; then
  echo "❌ 找不到 Gazebo world: $GAZEBO_WORLD"
  echo "   如果 UGV 工程路径不同，请这样指定：UGV_WS=/path/to/Air-Ground-Collaborative-Manupulation ./one_key_takeoff.sh"
  exit 1
fi

if [[ "$LIDAR_SDF" = /* && ! -f "$LIDAR_SDF" ]]; then
  echo "❌ 找不到 UAV SDF: $LIDAR_SDF"
  exit 1
fi

if [[ "$START_UGV_MARKER" = "true" && ! -f "$UGV_URDF" ]]; then
  echo "⚠️  找不到 UGV URDF: $UGV_URDF"
  echo "   将退回单个 Husky mesh marker；如路径不同请设置 UGV_WS 或 UGV_URDF。"
fi

if [[ ! -x "$TF2_STATIC_PUBLISHER" ]]; then
  echo "❌ 找不到 tf2_ros static_transform_publisher: $TF2_STATIC_PUBLISHER"
  echo "   请检查 ROS Noetic 是否安装完整。"
  exit 1
fi

# 1. 先单独启动 Gazebo + 轻量 forest 世界
# Workspace models must be first. models/depth_camera deliberately overrides
# ~/.gazebo/gazebo_models/depth_camera, which has no ROS image plugin.
gnome-terminal --tab --title="1_Gazebo_Forest_Headless" -- bash -c "
source /opt/ros/noetic/setup.bash && \
export GAZEBO_MODEL_DATABASE_URI='' && \
export GAZEBO_PLUGIN_PATH=\"$PX4_DIR/build/px4_sitl_default/build_gazebo-classic:\$GAZEBO_PLUGIN_PATH\" && \
export GAZEBO_MODEL_PATH=\"$EGO_WS/models:$EGO_WS/src/uav_truth_tracker/models:$UGV_WS/src/gazebo_models_worlds_collection/models:$UGV_WS/src/mobile_manipulator/gazebo_models:$UGV_WS/src/mobile_manipulator/models:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$HOME/.gazebo/gazebo_models:$HOME/.gazebo/models:\$GAZEBO_MODEL_PATH\" && \
export GAZEBO_RESOURCE_PATH=\"$EGO_WS/worlds:$UGV_WS/src/mobile_manipulator/worlds:\$GAZEBO_RESOURCE_PATH\" && \
roslaunch gazebo_ros empty_world.launch world_name:='$GAZEBO_WORLD' paused:=false use_sim_time:=true gui:='${PX4_GAZEBO_GUI}' 2>&1 | grep -v \"parser.cc\"; exec bash"

echo "⏳ 给 Gazebo $GAZEBO_LOAD_WAIT 秒加载轻量 forest 世界..."
sleep "$GAZEBO_LOAD_WAIT"

# 1.5 Gazebo 已经起来后，再启动 PX4 并把 UAV spawn 进去，避免重 world 导致 PX4 等仿真器超时
gnome-terminal --tab --title="1.5_PX4_Spawn_UAV" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
cd '$PX4_DIR' && \
export GAZEBO_MODEL_DATABASE_URI='' && \
source Tools/simulation/gazebo-classic/setup_gazebo.bash \$(pwd) \$(pwd)/build/px4_sitl_default && \
export GAZEBO_MODEL_PATH=\"$EGO_WS/models:$EGO_WS/src/uav_truth_tracker/models:$UGV_WS/src/gazebo_models_worlds_collection/models:$UGV_WS/src/mobile_manipulator/gazebo_models:$UGV_WS/src/mobile_manipulator/models:\$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$HOME/.gazebo/gazebo_models:$HOME/.gazebo/models:\$GAZEBO_MODEL_PATH\" && \
export GAZEBO_RESOURCE_PATH=\"$EGO_WS/worlds:$UGV_WS/src/mobile_manipulator/worlds:\$GAZEBO_RESOURCE_PATH\" && \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd):\$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic && \
roslaunch uav_truth_tracker px4_spawn_existing_gazebo.launch vehicle:=iris_depth_camera sdf:='$LIDAR_SDF' x:='$SPAWN_X' y:='$SPAWN_Y' z:='$SPAWN_Z' Y:='$SPAWN_YAW' 2>&1 | grep -v \"parser.cc\"; exec bash"

echo "⏳ 等待 8 秒，让 PX4 与 Gazebo 完成连接..."
sleep 8

# 2. 仓库加载好后，独立连接 MAVROS
if [ "$START_MAVROS" = "true" ]; then
  gnome-terminal --tab --title="2_MAVROS" -- bash -c "source /opt/ros/noetic/setup.bash && roslaunch '$MAVROS_PX4_LAUNCH' fcu_url:=\"udp://:14540@127.0.0.1:14557\"; exec bash"
  echo "⏳ 等待 5 秒，让 MAVROS 连上飞控..."
  sleep 5
else
  echo "⏭️  START_MAVROS=false，跳过 MAVROS。"
fi

# 3. 一站式启动: TF + marker + UGV模型 + 深度预处理 + UAV→UGV点云 + EGO-Planner + OctoMap + RViz
#    所有 ROS 层组件合并在一个 roslaunch 中，大幅减少终端窗口数量
echo "🚀 启动 forest_uav_mapping.launch (TF / Marker / UGV / Depth / UAV→UGV / EGO / OctoMap / RViz)..."
mkdir -p "$UAV_POINTS_MAP_DIR"
gnome-terminal --tab --title="3_All_ROS_Nodes" -- bash -c "
source /opt/ros/noetic/setup.bash && \
source '$EGO_WS/devel/setup.bash' && \
export ROS_PACKAGE_PATH='$UGV_ROS_PACKAGE_PATH':\$ROS_PACKAGE_PATH && \
export PYTHONPATH='$EGO_WS':\$PYTHONPATH && \
roslaunch uav_truth_tracker forest_uav_mapping.launch \
  use_mavros:='$START_MAVROS' \
  enable_octomap:='$ENABLE_OCTOMAP' \
  start_rviz:='$START_RVIZ' \
  ; exec bash"

echo "⏳ 等待 18 秒，让所有 ROS 节点启动就绪..."
sleep 18

# 4. 启动翻译官，解锁起飞！
if [ "$START_TAKEOFF" = "true" ]; then
  gnome-terminal --tab --title="6_Takeoff" -- bash -c "source '$EGO_WS/devel/setup.bash' && python3 -u '$EGO_WS/px4_bridge.py' _require_depth_before_takeoff:=false; exec bash"
else
  echo "⏭️  START_TAKEOFF=false，跳过自动起飞。"
fi

echo "✅ 一键起飞序列执行完毕！"
echo "🗺️ OctoMap 建图: $ENABLE_OCTOMAP"
echo "☁️ UAV→UGV 世界系点云: $START_UAV_POINTS_TO_UGV, 输出: $UAV_POINTS_OUTPUT_TOPIC, frame: $UAV_POINTS_TARGET_FRAME"
echo "💾 UAV 累计点云保存: $START_UAV_POINTS_RECORDER, 文件: $UAV_POINTS_MAP_DIR/${UAV_POINTS_MAP_NAME}_latest.pcd"
echo "🗺️ Gazebo world: $GAZEBO_WORLD"
echo "🧱 UGV outdoor city 原始 world 可手动切换: GAZEBO_WORLD=$PX4_OUTDOOR_WORLD"
echo "📡 UAV LiDAR SDF: $LIDAR_SDF"
echo "🧭 Gazebo GUI: $PX4_GAZEBO_GUI, RViz: $START_RVIZ, MAVROS: $START_MAVROS"
echo "🔌 位姿源: $([ "$START_MAVROS" = "true" ] && echo MAVROS || ([ "$HAS_ODOMETRY" = "true" ] && echo Gazebo_Odom_Bridge || echo 无))"
echo "🗺️ OctoMap: $ENABLE_OCTOMAP, EGO-Planner: $START_EGO_PLANNER, UAV→UGV: $START_UAV_POINTS_TO_UGV, EGO flight: $START_TAKEOFF"
echo "📍 RViz: Simulation → drone_visual (Iris), ugv_robot_model (Husky UGV)"
echo "🎮 飞行控制必须使用 MAVROS OFFBOARD；Gazebo Odom Bridge 只适合静态建图调试"
echo "🔎 起飞后检查: rostopic echo -n 1 /mavros/state"
