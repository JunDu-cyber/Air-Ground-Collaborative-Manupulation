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

if ! command -v gnome-terminal &>/dev/null; then
    echo "❌ 找不到 gnome-terminal，本脚本需要 GNOME Terminal"
    echo "   请安装: sudo apt-get install gnome-terminal"
    exit 1
fi

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
GAZEBO_LOAD_WAIT=${GAZEBO_LOAD_WAIT:-10}
PX4_WAIT=${PX4_WAIT:-8}
MAVROS_WAIT=${MAVROS_WAIT:-5}
ROS_WAIT=${ROS_WAIT:-18}
# ENABLE_OCTOMAP default is set later, after HAS_ODOMETRY is known
UAV_POINTS_MAP_DIR=${UAV_POINTS_MAP_DIR:-$HOME/pointcloud_maps}
SPAWN_X=${SPAWN_X:-0.0}
SPAWN_Y=${SPAWN_Y:--18.0}
SPAWN_Z=${SPAWN_Z:-1.5}
SPAWN_YAW=${SPAWN_YAW:-1.5707963}

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
# 无里程计时默认关闭
ENABLE_OCTOMAP=${ENABLE_OCTOMAP:-false}

START_TAKEOFF=${START_TAKEOFF:-$START_MAVROS}

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
source /usr/share/gazebo/setup.sh && \
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

echo "⏳ 等待 $PX4_WAIT 秒，让 PX4 与 Gazebo 完成连接..."
sleep "$PX4_WAIT"

# 2. 仓库加载好后，独立连接 MAVROS
if [ "$START_MAVROS" = "true" ]; then
  gnome-terminal --tab --title="2_MAVROS" -- bash -c "source /opt/ros/noetic/setup.bash && roslaunch '$MAVROS_PX4_LAUNCH' fcu_url:=\"udp://:14540@127.0.0.1:14580\"; exec bash"
  echo "⏳ 等待 $MAVROS_WAIT 秒，让 MAVROS 连上飞控..."
  sleep "$MAVROS_WAIT"
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
  ugv_ws:='$UGV_WS' \
  ugv_urdf:='$UGV_URDF' \
  forest_world_file:='$FOREST_WORLD' \
  ; exec bash"

echo "⏳ 等待 $ROS_WAIT 秒，让所有 ROS 节点启动就绪..."
sleep "$ROS_WAIT"

# 4. 启动翻译官，解锁起飞！
if [ "$START_TAKEOFF" = "true" ]; then
  gnome-terminal --tab --title="6_Takeoff" -- bash -c "source /opt/ros/noetic/setup.bash && source '$EGO_WS/devel/setup.bash' && python3 -u '$EGO_WS/px4_bridge.py' _require_depth_before_takeoff:=false; exec bash"
else
  echo "⏭️  START_TAKEOFF=false，跳过自动起飞。"
fi

echo "✅ 一键起飞序列执行完毕！"
echo "🗺️ OctoMap 建图: $ENABLE_OCTOMAP"
echo "💾 UAV 点云保存目录: $UAV_POINTS_MAP_DIR"
echo "🗺️ Gazebo world: $GAZEBO_WORLD"
echo "🧱 UGV outdoor city 原始 world 可手动切换: GAZEBO_WORLD=$PX4_OUTDOOR_WORLD"
echo "📡 UAV LiDAR SDF: $LIDAR_SDF"
echo "🧭 Gazebo GUI: $PX4_GAZEBO_GUI, RViz: $START_RVIZ, MAVROS: $START_MAVROS"
echo "🔌 位姿源: $([ "$START_MAVROS" = "true" ] && echo MAVROS || ([ "$HAS_ODOMETRY" = "true" ] && echo Gazebo_Odom_Bridge || echo 无))"
echo "🗺️ OctoMap: $ENABLE_OCTOMAP, EGO flight: $START_TAKEOFF"
echo "📍 RViz: Simulation → drone_visual (Iris), ugv_robot_model (Husky UGV)"
echo "🎮 飞行控制必须使用 MAVROS OFFBOARD；Gazebo Odom Bridge 只适合静态建图调试"
echo "🔎 起飞后检查: rostopic echo -n 1 /mavros/state"
