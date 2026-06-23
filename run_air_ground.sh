#!/bin/bash
# ============================================================================
# 空地协同：UAV + UGV 在同一个 Gazebo 世界，分终端【按序】启动。
#
# 为什么分终端按序（而不是一个 launch 一把梭）：
#   一把梭会把所有节点同时起，UGV 的机械臂控制器还没抓住折叠姿态、物理就被
#   PX4/UAV 那堆负载一冲 → 机械臂垮下去穿模；而且一个终端日志糊成一团、也卡。
#   按序：先 gazebo(paused) + UGV(控制器抓住折叠臂) → 再 PX4+UAV → 统一 unpause
#   → MAVROS → UAV飞行栈 → UGV导航。机械臂全程被控制器held住，负载也分散。
#
# 命名空间隔离：UAV 用 map→uav_base_link + 目标话题 /uav/goal；UGV 用
#   map→odom→base_link + /move_base_simple/goal —— 两台互不打架。
#
# 用法：  bash run_air_ground.sh          # 默认低空(4m)+全局规划绕楼
#         LOW_ALT=false bash run_air_ground.sh   # 12m 俯扫
# ============================================================================
set -u

echo "🧹 清理旧进程..."
killall -9 gzserver gzclient rosmaster roscore px4 mavros 2>/dev/null
pkill -9 -f "mavros_node|px4_bridge.py|forest_uav_mapping|px4_spawn_existing|ugv_terrain_nav|air_ground_world|move_base|ekf_localization|ekf_global" 2>/dev/null || true
sleep 3
command -v gnome-terminal >/dev/null || { echo "❌ 需要 gnome-terminal"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS=${WS:-$SCRIPT_DIR}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
GUI=${GUI:-true}
START_RVIZ=${START_RVIZ:-true}
PX4_SIM_SPEED_FACTOR=${PX4_SIM_SPEED_FACTOR:-1}
WORLD=${WORLD:-$WS/src/mobile_manipulator/worlds/outdoor_city.world}   # 已 250Hz, 含地形+楼
LIDAR_SDF=${LIDAR_SDF:-$WS/models/iris_depth_camera_lidar_mapping_light/model.sdf}
MAVROS_PX4_LAUNCH=${MAVROS_PX4_LAUNCH:-/opt/ros/noetic/share/mavros/launch/px4.launch}

# UAV 出生点（和 UGV 在原点错开）
UAV_X=${UAV_X:-0.0}; UAV_Y=${UAV_Y:--18.0}; UAV_Z=${UAV_Z:-1.5}; UAV_YAW=${UAV_YAW:-1.5707963}
# 命名空间隔离
UAV_BASE_FRAME=${UAV_BASE_FRAME:-uav_base_link}
UAV_GOAL_TOPIC=${UAV_GOAL_TOPIC:-/uav/goal}
# ★坐标统一：三处 GPS 锚点必须同一个 datum，否则 UAV(PX4默认苏黎世47.4) 和 UGV(49.9)
#   差 ~280km、align 报 MISALIGNED、UAV 建的图落不到 UGV 的 map 系里。
#   ① UGV navsat datum(config 里固定49.9,8.9) ② UGV Husky hector GPS(GAZEBO_WORLD_LAT/LON)
#   ③ UAV PX4 home(PX4_HOME_LAT/LON) ←之前漏了, 这里补上, 三者都钉到 DATUM。
DATUM_LAT=${DATUM_LAT:-49.9}; DATUM_LON=${DATUM_LON:-8.9}; DATUM_ALT=${DATUM_ALT:-0}

# 低空/高空（同 one_key）
LOW_ALT=${LOW_ALT:-true}
if [ "$LOW_ALT" = "true" ]; then FLIGHT_H=${FLIGHT_H:-4.0}; MAXV=${MAXV:-1.2}; GROUND_FILTER=${GROUND_FILTER:-0.5}
else FLIGHT_H=${FLIGHT_H:-12.0}; MAXV=${MAXV:-1.5}; GROUND_FILTER=${GROUND_FILTER:-10.0}; fi

# 等待（重 world 慢就调大）
GAZEBO_LOAD_WAIT=${GAZEBO_LOAD_WAIT:-12}; PX4_WAIT=${PX4_WAIT:-8}; MAVROS_WAIT=${MAVROS_WAIT:-5}; ROS_WAIT=${ROS_WAIT:-15}
GPLUGIN="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic"

[ -f "$WORLD" ]     || { echo "❌ 找不到 world: $WORLD"; exit 1; }
[ -f "$LIDAR_SDF" ] || { echo "❌ 找不到 UAV SDF: $LIDAR_SDF"; exit 1; }
[ -f "$MAVROS_PX4_LAUNCH" ] || { echo "❌ 找不到 MAVROS px4.launch (装 ros-noetic-mavros)"; exit 1; }

echo "🛩️  LOW_ALT=$LOW_ALT 飞行=${FLIGHT_H}m | UAV出生(${UAV_X},${UAV_Y}) base=$UAV_BASE_FRAME goal=$UAV_GOAL_TOPIC"

# ── 终端1: Gazebo(paused, 全模型路径) + UGV(折叠臂, 不 unpause) ──
gnome-terminal --tab --title="1_Gazebo+UGV" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && source /usr/share/gazebo/setup.sh && \
export GAZEBO_PLUGIN_PATH=\"$GPLUGIN:\$GAZEBO_PLUGIN_PATH\" && export GAZEBO_MODEL_DATABASE_URI='' && \
export PX4_SIM_SPEED_FACTOR='$PX4_SIM_SPEED_FACTOR' && \
export GAZEBO_WORLD_LAT='$DATUM_LAT' && export GAZEBO_WORLD_LON='$DATUM_LON' && \
roslaunch mobile_manipulator air_ground_world.launch spawn_uav:=false unpause_on_spawn:=false gui:='$GUI' world:='$WORLD' 2>&1 | grep -v parser.cc; exec bash"
echo "⏳ Gazebo+UGV 加载 ${GAZEBO_LOAD_WAIT}s（机械臂在 paused 下被控制器抓住折叠）..."
sleep "$GAZEBO_LOAD_WAIT"

# ── 物理：同 one_key 设 200Hz/0.005 让 RTF≈1，防 PX4 EKF 发散（PX4 spawn 前设好） ──
( source /opt/ros/noetic/setup.bash && rosservice call --wait /gazebo/set_physics_properties "
time_step: 0.005
max_update_rate: 200.0
gravity: {x: 0.0, y: 0.0, z: -9.8}
ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}" ) 2>/dev/null && echo "✅ 物理已设" || echo "⚠️  set_physics 失败"

# ── 终端1.5: PX4 + UAV 注入同一个 gazebo ──
gnome-terminal --tab --title="1.5_PX4+UAV" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && cd '$PX4_DIR' && \
export GAZEBO_MODEL_DATABASE_URI='' && source Tools/simulation/gazebo-classic/setup_gazebo.bash \$(pwd) \$(pwd)/build/px4_sitl_default && \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\$(pwd):\$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic && \
export PX4_SIM_SPEED_FACTOR='$PX4_SIM_SPEED_FACTOR' && \
export PX4_HOME_LAT='$DATUM_LAT' && export PX4_HOME_LON='$DATUM_LON' && export PX4_HOME_ALT='$DATUM_ALT' && \
roslaunch uav_truth_tracker px4_spawn_existing_gazebo.launch vehicle:=iris_depth_camera sdf:='$LIDAR_SDF' x:='$UAV_X' y:='$UAV_Y' z:='$UAV_Z' Y:='$UAV_YAW' sim_speed_factor:='$PX4_SIM_SPEED_FACTOR' 2>&1 | grep -v parser.cc; exec bash"
echo "⏳ PX4 启动 + 与 Gazebo 插件握手 ${PX4_WAIT}s..."
sleep "$PX4_WAIT"

# ── 统一 unpause：此时 UGV 控制器已 held 住折叠臂；让 lockstep 干净握手 ──
( source /opt/ros/noetic/setup.bash && rosservice call --wait /gazebo/unpause_physics "{}" ) 2>/dev/null && echo "▶️  已 unpause" || echo "⚠️  手动: rosservice call /gazebo/unpause_physics"
sleep 2

# ── 终端2: MAVROS ──
gnome-terminal --tab --title="2_MAVROS" -- bash -c "source /opt/ros/noetic/setup.bash && roslaunch '$MAVROS_PX4_LAUNCH' fcu_url:='udp://:14540@127.0.0.1:14580'; exec bash"
echo "⏳ MAVROS 连飞控 ${MAVROS_WAIT}s..."; sleep "$MAVROS_WAIT"

# ── 终端3: UAV 飞行栈(EGO + 全局规划绕楼 + 建图; 命名空间隔离) ──
#    start_rviz:=false —— 不用 forest 那个(只显示 UGV 底盘 marker、没机械臂)；
#    改用下面终端6 的合并版 RViz(带臂 UGV RobotModel + UAV + 两个目标工具)。
gnome-terminal --tab --title="3_UAV_Stack" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && \
roslaunch uav_truth_tracker forest_uav_mapping.launch use_mavros:=true start_rviz:=false \
  low_altitude:='$LOW_ALT' flight_height:='$FLIGHT_H' max_vel:='$MAXV' ground_filter_margin:='$GROUND_FILTER' \
  uav_child_frame:='$UAV_BASE_FRAME' uav_goal_topic:='$UAV_GOAL_TOPIC'; exec bash"
echo "⏳ UAV 节点起 ${ROS_WAIT}s..."; sleep "$ROS_WAIT"

# ── 终端4: UGV 导航(复用终端1的双EKF; 样例地形图; align 阈值放大不误刷) ──
#    align_warn_threshold:=500 —— 两台相距几十~上百米是正常物理间距, 只有 datum
#    真不一致(~km级)才该报警。(坐标统一靠脚本开头三处 GPS 锚点都钉到 DATUM。)
gnome-terminal --tab --title="4_UGV_Nav" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && \
roslaunch mobile_manipulator ugv_terrain_nav.launch localization:=false use_sample_map:=true rviz:=false align_warn_threshold:=500; exec bash"

# ── 终端5: 起飞 bridge(MAVROS OFFBOARD 解锁) ──
gnome-terminal --tab --title="5_Takeoff" -- bash -c "source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && python3 -u '$WS/px4_bridge.py' _require_depth_before_takeoff:=false; exec bash"

# ── 终端6: 合并版 RViz(带臂 UGV + UAV mesh/点云/路径 + 两个 2D Nav Goal) ──
if [ "$START_RVIZ" = "true" ]; then
gnome-terminal --tab --title="6_RViz" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && \
rviz -d '$WS/src/mobile_manipulator/rviz/air_ground.rviz'; exec bash"
fi

echo "════════════════════════════════════════════"
echo "✅ 空地协同分终端启动完毕（终端 1 / 1.5 / 2 / 3 / 4 / 5 / 6_RViz）"
echo "  • RViz 工具栏有【两个 2D Nav Goal】: 第1个发 UGV(/move_base_simple/goal),"
echo "    第2个发 UAV($UAV_GOAL_TOPIC)。悬停看话题区分。"
echo "  • UGV 应显示带 UR5 机械臂的完整模型(RobotModel), 不是光底盘。"
echo "  • 坐标统一自检: rostopic echo -n1 /mavros/global_position/global 的 lat 应≈$DATUM_LAT(不是47.4);"
echo "    align 偏差应从~280000m 掉到几十米(真实间距)。"
echo "  • TF 应为 map→$UAV_BASE_FRAME(UAV) 和 map→odom→base_link(UGV), 无双 base_link。"
echo "════════════════════════════════════════════"
