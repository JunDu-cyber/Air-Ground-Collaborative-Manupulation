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
PX4_GAZEBO_GUI=${PX4_GAZEBO_GUI:-true}
# 锁步 SITL 实时倍率。必须=1，否则 lockstep 会以 200-300x 狂奔，
# 而 px4_bridge/MAVROS/EGO 按墙钟 50Hz 发指令，在 PX4 仿真时间里变成亚赫兹，
# PX4 因 OFFBOARD 指令超时而怠速不起飞。
PX4_SIM_SPEED_FACTOR=${PX4_SIM_SPEED_FACTOR:-1}
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
# 低空穿楼建图模式（LOW_ALT=true）：4m 固定低空、横向绕楼。
# 飞行高度/速度在 bash 里算死并打印出来，避免 roslaunch 那边 eval 没生效你还不知道。
LOW_ALT=${LOW_ALT:-true}    # 默认低空穿街(4m, 开全局规划绕楼); LOW_ALT=false 切 12m 俯扫
if [ "$LOW_ALT" = "true" ]; then
  FLIGHT_H=${FLIGHT_H:-4.0}; MAXV=${MAXV:-1.2}; GROUND_FILTER=${GROUND_FILTER:-0.5}
else
  FLIGHT_H=${FLIGHT_H:-12.0}; MAXV=${MAXV:-1.5}; GROUND_FILTER=${GROUND_FILTER:-10.0}
fi
echo "════════════════════════════════════════════"
echo "🛩️  LOW_ALT=$LOW_ALT  飞行高度=${FLIGHT_H}m  速度=${MAXV}m/s  EGO地面过滤=${GROUND_FILTER}"
echo "    EGO避障: ground_filter=${GROUND_FILTER}(0.5=看得见楼能避 / 10=看不见不避) 视距15m"
echo "    (LOW_ALT=true → 4m穿楼避障 / false → 12m俯扫；FLIGHT_H= MAXV= GROUND_FILTER= 可覆盖)"
echo "════════════════════════════════════════════"

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
export PX4_SIM_SPEED_FACTOR='$PX4_SIM_SPEED_FACTOR' && \
export GAZEBO_MODEL_DATABASE_URI='' && \
export GAZEBO_PLUGIN_PATH=\"$PX4_DIR/build/px4_sitl_default/build_gazebo-classic:\$GAZEBO_PLUGIN_PATH\" && \
export GAZEBO_MODEL_PATH=\"$EGO_WS/models:$EGO_WS/src/uav_truth_tracker/models:$UGV_WS/src/gazebo_models_worlds_collection/models:$UGV_WS/src/mobile_manipulator/gazebo_models:$UGV_WS/src/mobile_manipulator/models:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$HOME/.gazebo/gazebo_models:$HOME/.gazebo/models:\$GAZEBO_MODEL_PATH\" && \
export GAZEBO_RESOURCE_PATH=\"$EGO_WS/worlds:$UGV_WS/src/mobile_manipulator/worlds:\$GAZEBO_RESOURCE_PATH\" && \
roslaunch gazebo_ros empty_world.launch world_name:='$GAZEBO_WORLD' paused:=true use_sim_time:=true gui:='${PX4_GAZEBO_GUI}' 2>&1 | grep -v \"parser.cc\"; exec bash"

echo "⏳ 给 Gazebo $GAZEBO_LOAD_WAIT 秒加载轻量 forest 世界..."
sleep "$GAZEBO_LOAD_WAIT"

# 1.4 关键(非锁步 SITL): 把物理步长放大到 CPU 能跑满实时(RTF≈1.0)再启动 PX4。
# PX4 用 NOLOCKSTEP 编译(墙钟), Gazebo 用 sim-time。若 RTF<1(默认 0.004×250=目标1.0
# 但重 world 下 gzserver 只能跑到 ~213Hz → RTF 0.85), PX4 墙钟比 sim 快 ~15%, EKF 把
# IMU 多积分 15% → 高度估计发散(以为在 4~5m 高空)→ 收油门 → 永远起不来。
# 实测把步长 0.004→0.005 (250→200Hz) 让 gzserver 跑满实时, RTF≈0.99, EKF 收敛, 可起飞。
# 必须在 PX4 spawn 之前设好, 让 PX4 一上来就处在 RTF≈1 的世界, 避免发散后再纠正引发的瞬态。
PHYSICS_STEP=${PHYSICS_STEP:-0.005}
PHYSICS_RATE=${PHYSICS_RATE:-200.0}
echo "⚙️  设置物理步长=$PHYSICS_STEP, 更新率=$PHYSICS_RATE (非锁步 SITL 需 RTF≈1.0 防 EKF 发散)..."
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
" ) 2>/dev/null && echo "✅ 物理参数已设置" || echo "⚠️  set_physics_properties 调用失败"

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
export PX4_SIM_SPEED_FACTOR='$PX4_SIM_SPEED_FACTOR' && \
roslaunch uav_truth_tracker px4_spawn_existing_gazebo.launch vehicle:=iris_depth_camera sdf:='$LIDAR_SDF' x:='$SPAWN_X' y:='$SPAWN_Y' z:='$SPAWN_Z' Y:='$SPAWN_YAW' sim_speed_factor:='$PX4_SIM_SPEED_FACTOR' 2>&1 | grep -v \"parser.cc\"; exec bash"

echo "⏳ 等待 PX4 启动并与 Gazebo 插件建立 TCP(4560) 连接..."
sleep "$PX4_WAIT"

# 关键: Gazebo 以 paused 启动，避免在 PX4/模型出现前自由空跑(700x)。
# PX4 是 lockstep 编译版，会持续向插件发送执行器(HIL_ACTUATOR_CONTROLS)。
# 若 Gazebo 在握手前已空跑，插件不会进入 lockstep 读取循环 -> 执行器数据在
# TCP 4560 上积压不被读取(Send-Q/Recv-Q 卡 93B) -> 电机停在 disarmed 怠速 -> 无法起飞。
# 现在 PX4 已连接，unpause 让 lockstep 从干净状态握手并开始消费执行器指令。
echo "▶️  Unpause Gazebo，让 lockstep 干净握手(修复执行器死锁)..."
( source /opt/ros/noetic/setup.bash && \
  rosservice call --wait /gazebo/unpause_physics "{}" ) 2>/dev/null \
  && echo "✅ Gazebo 已 unpause" \
  || echo "⚠️  unpause 调用失败，请手动执行: rosservice call /gazebo/unpause_physics"
sleep 2

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
  low_altitude:='$LOW_ALT' \
  flight_height:='$FLIGHT_H' \
  max_vel:='$MAXV' \
  ground_filter_margin:='$GROUND_FILTER' \
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
