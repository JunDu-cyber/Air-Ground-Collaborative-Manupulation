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
#         ENABLE_MINE_DETECTION=false bash run_air_ground.sh  # 临时关闭排雷感知
# ============================================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS=${WS:-$SCRIPT_DIR}

echo "🧹 清理旧进程..."
killall -9 gzserver gzclient rosmaster roscore px4 mavros 2>/dev/null
pkill -9 -f "mavros_node|px4_bridge.py|forest_uav_mapping|px4_spawn_existing|ugv_terrain_nav|air_ground_world|move_base|move_group|ekf_localization|ekf_global|mine_seg_localizer|mine_map_fusion|mine_camera_diagnostics|uav_goal_arbiter|mine_detection_sim_evaluator|mine_mission_manager|mine_grasp_executor|wrist_mine_localizer|mock_mine_grasp_server|mine_survey_waypoints" 2>/dev/null || true
# roslaunch can exit before every child it spawned.  Those orphaned EGO,
# mapping and helper nodes reconnect to the next rosmaster and create two
# competing flight/navigation stacks.  A run_air_ground restart owns every
# node from this workspace, so remove all stale devel-space children as well.
pkill -9 -f "$WS/devel/lib/" 2>/dev/null || true
sleep 3
command -v gnome-terminal >/dev/null || { echo "❌ 需要 gnome-terminal"; exit 1; }

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
GUI=${GUI:-true}
START_RVIZ=${START_RVIZ:-true}
PX4_SIM_SPEED_FACTOR=${PX4_SIM_SPEED_FACTOR:-1}
WORLD=${WORLD:-$WS/src/mobile_manipulator/worlds/outdoor_city.world}   # 已 250Hz, 含地形+楼
# 前视深度相机继续供 EGO 避障；新增的 960x540 下视 RGB-D 专门负责地雷分割与定位。
LIDAR_SDF=${LIDAR_SDF:-$WS/models/iris_depth_camera_lidar_mine/model.sdf}
MAVROS_PX4_LAUNCH=${MAVROS_PX4_LAUNCH:-/opt/ros/noetic/share/mavros/launch/px4.launch}

# 排雷感知：模型只在 UAV/地面计算机侧运行，UGV 订阅确认后的 map 坐标。
SPAWN_MINES=${SPAWN_MINES:-true}
ENABLE_MINE_DETECTION=${ENABLE_MINE_DETECTION:-true}
MINE_MODEL=${MINE_MODEL:-$WS/mine_seg_v2_delivery/weights/best.pt}
MINE_CONFIDENCE=${MINE_CONFIDENCE:-0.65}
MINE_INFERENCE_RATE=${MINE_INFERENCE_RATE:-1.0}
MINE_TORCH_THREADS=${MINE_TORCH_THREADS:-2}
MINE_SIM_EVAL=${MINE_SIM_EVAL:-true}
MINE_OUTPUT_DIR=${MINE_OUTPUT_DIR:-$WS/mine_detection_output}
# Absolute z is now only a numerical sanity gate. Mine/terrain validity is
# checked against the local ground-depth ring, so hills are not rejected merely
# because their world z differs from zero.
MINE_MIN_MAP_Z=${MINE_MIN_MAP_Z:--20.0}
MINE_MAX_MAP_Z=${MINE_MAX_MAP_Z:-50.0}
# 默认只接受 RViz /uav/manual_goal：不启动自动覆盖航线，也不让
# survey 话题覆盖用户点选目标。
UAV_MANUAL_ONLY=${UAV_MANUAL_ONLY:-true}
MINE_AUTO_SURVEY=${MINE_AUTO_SURVEY:-false}
if [ "$UAV_MANUAL_ONLY" = "true" ] && [ "$MINE_AUTO_SURVEY" = "true" ]; then
  echo "⚠️  UAV_MANUAL_ONLY=true，已禁用 MINE_AUTO_SURVEY，避免自动航线接管"
  MINE_AUTO_SURVEY=false
fi
MINE_SURVEY_DWELL=${MINE_SURVEY_DWELL:-8.0}
MINE_CANDIDATE_HOLD=${MINE_CANDIDATE_HOLD:-18.0}
MINE_MIN_CONFIRMED=${MINE_MIN_CONFIRMED:-5}
MINE_SURVEY_RESCAN_ROUNDS=${MINE_SURVEY_RESCAN_ROUNDS:-3}
MINE_SURVEY_RESCAN_WAIT=${MINE_SURVEY_RESCAN_WAIT:-10.0}
MINE_SURVEY_LOITER_X=${MINE_SURVEY_LOITER_X:-0.0}
MINE_SURVEY_LOITER_Y=${MINE_SURVEY_LOITER_Y:--6.0}

# UGV 排雷任务：默认启动本仓库经过单雷验证的 wrist RGB-D 最小抓取器。
# external 仍可接队友实现；mock_* 只验证任务调度，不代表真实抓取。
# 尚未接入时，用 MINE_ARM_MODE=mock_success 做完整仿真，或 mock_timeout 验证
# 超时后保留危险区、锁住底盘并转人工复位的安全逻辑。
ENABLE_MINE_MISSION=${ENABLE_MINE_MISSION:-true}
MINE_ARM_MODE=${MINE_ARM_MODE:-minimal}  # minimal|external|mock_success|mock_failure|mock_fatal|mock_timeout
MINE_GRASP_PHYSICAL=${MINE_GRASP_PHYSICAL:-true}
MINE_GRASP_WAIT=${MINE_GRASP_WAIT:-8}
# UAV YOLO already identifies/map-registers the mine. At the stopped UGV use
# bounded wrist HSV+depth by default, avoiding a second CPU YOLO process while
# Gazebo and MoveIt execute the precision approach. Set hybrid to re-enable it.
MINE_WRIST_LOCALIZATION_MODE=${MINE_WRIST_LOCALIZATION_MODE:-color_only}
MINE_AUTO_START=${MINE_AUTO_START:-true}
MINE_RESUME=${MINE_RESUME:-false}         # 仿真默认不继承上次状态；真机可设 true
MINE_MOCK_START=false
MINE_MOCK_MODE=success
MINE_MINIMAL_START=false
UGV_FREE_BORDER=${UGV_FREE_BORDER:-35.0}  # 要覆盖 UGV(y=-18) 到 UAV/雷区(y≈0)
# UR5 六关节物理阻尼（N*m*s/rad）。整链上线值 1.5 仅改 URDF dynamics，
# 不改重力、质量、控制器 PID 或抓取的 0.05 rad/s 实测稳定门。
UR5_JOINT_DAMPING=${UR5_JOINT_DAMPING:-1.5}
# The measured residual is isolated to shoulder_lift.  Keep the other five on
# the validated common value and expose the shoulder value independently.
UR5_SHOULDER_LIFT_DAMPING=${UR5_SHOULDER_LIFT_DAMPING:-20.0}
case "$MINE_ARM_MODE" in
  minimal) MINE_MINIMAL_START=true ;;
  external) ;;
  mock_success) MINE_MOCK_START=true; MINE_MOCK_MODE=success ;;
  mock_failure) MINE_MOCK_START=true; MINE_MOCK_MODE=failure ;;
  mock_fatal)   MINE_MOCK_START=true; MINE_MOCK_MODE=fatal ;;
  mock_timeout) MINE_MOCK_START=true; MINE_MOCK_MODE=timeout ;;
  *) echo "❌ MINE_ARM_MODE=$MINE_ARM_MODE 无效（minimal|external|mock_success|mock_failure|mock_fatal|mock_timeout）"; exit 1 ;;
esac

# ★UAV 从【世界原点(0,0)】起飞 -> MAVROS 局部系=世界帧(零偏移) -> EGO 轨迹/模型/点云/UGV
#   全在同一帧重合;所有按 UAV_X/UAV_Y 参数化的偏移自动归零。UGV 让到 (0,-18)(air_ground_world)。
UAV_X=${UAV_X:-0.0}; UAV_Y=${UAV_Y:-0.0}; UAV_Z=${UAV_Z:-1.5}; UAV_YAW=${UAV_YAW:-1.5707963}
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
# CPU YOLO needs roughly one second per inference.  At 1.2 m/s a narrow mine
# could leave the downward camera footprint after only one usable frame.  The
# 0.9 m/s low-altitude default gives the strict two-frame fast-confirmation
# gate a complete fly-by without changing manual-goal control or EGO routing.
if [ "$LOW_ALT" = "true" ]; then FLIGHT_H=${FLIGHT_H:-4.0}; MAXV=${MAXV:-0.9}; GROUND_FILTER=${GROUND_FILTER:-0.5}
else FLIGHT_H=${FLIGHT_H:-12.0}; MAXV=${MAXV:-1.2}; GROUND_FILTER=${GROUND_FILTER:-10.0}; fi

# 等待（重 world 慢就调大）
GAZEBO_LOAD_WAIT=${GAZEBO_LOAD_WAIT:-12}; PX4_WAIT=${PX4_WAIT:-8}; MAVROS_WAIT=${MAVROS_WAIT:-5}; ROS_WAIT=${ROS_WAIT:-15}; TAKEOFF_WAIT=${TAKEOFF_WAIT:-12}
GPLUGIN="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic"

[ -f "$WORLD" ]     || { echo "❌ 找不到 world: $WORLD"; exit 1; }
[ -f "$LIDAR_SDF" ] || { echo "❌ 找不到 UAV SDF: $LIDAR_SDF"; exit 1; }
[ -f "$MAVROS_PX4_LAUNCH" ] || { echo "❌ 找不到 MAVROS px4.launch (装 ros-noetic-mavros)"; exit 1; }
if [ "$ENABLE_MINE_DETECTION" = "true" ]; then
  [ -f "$MINE_MODEL" ] || { echo "❌ 找不到地雷模型: $MINE_MODEL"; exit 1; }
  mkdir -p "$MINE_OUTPUT_DIR"
fi
if [ "$ENABLE_MINE_MISSION" = "true" ] && [ "$ENABLE_MINE_DETECTION" != "true" ]; then
  echo "⚠️  ENABLE_MINE_DETECTION=false，自动关闭 UGV 排雷调度（没有可靠地雷输入）"
  ENABLE_MINE_MISSION=false
fi

echo "🛩️  LOW_ALT=$LOW_ALT 飞行=${FLIGHT_H}m | UAV出生(${UAV_X},${UAV_Y}) base=$UAV_BASE_FRAME goal=$UAV_GOAL_TOPIC"
echo "💣 地雷场=$SPAWN_MINES | 识别=$ENABLE_MINE_DETECTION conf=$MINE_CONFIDENCE 输出=$MINE_OUTPUT_DIR"
echo "🗺️  UAV手动目标模式=$UAV_MANUAL_ONLY | 自动覆盖航线=$MINE_AUTO_SURVEY | 地雷map高度门控=${MINE_MIN_MAP_Z}..${MINE_MAX_MAP_Z}m"
echo "🚙 排雷调度=$ENABLE_MINE_MISSION | 机械臂接口=$MINE_ARM_MODE | 导航外圈=0.84m/抓取停靠=0.62m | 投放区=初始点附近"
echo "🦾 UR5物理阻尼: shoulder_lift=${UR5_SHOULDER_LIFT_DAMPING}, 其余五关节=${UR5_JOINT_DAMPING} N*m*s/rad"
echo "🟨 腕部引信定位=$MINE_WRIST_LOCALIZATION_MODE（color_only=ROI颜色+深度，hybrid=YOLO+颜色）"

# ── 终端1: Gazebo(paused, 全模型路径) + UGV(折叠臂, 不 unpause) ──
gnome-terminal --tab --title="1_Gazebo+UGV" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && source /usr/share/gazebo/setup.sh && \
export GAZEBO_PLUGIN_PATH=\"$GPLUGIN:\$GAZEBO_PLUGIN_PATH\" && export GAZEBO_MODEL_DATABASE_URI='' && \
export PX4_SIM_SPEED_FACTOR='$PX4_SIM_SPEED_FACTOR' && \
export GAZEBO_WORLD_LAT='$DATUM_LAT' && export GAZEBO_WORLD_LON='$DATUM_LON' && \
roslaunch mobile_manipulator air_ground_world.launch spawn_uav:=false spawn_mines:='$SPAWN_MINES' unpause_on_spawn:=false gui:='$GUI' world:='$WORLD' ur5_joint_damping:='$UR5_JOINT_DAMPING' ur5_shoulder_lift_damping:='$UR5_SHOULDER_LIFT_DAMPING' 2>&1 | grep -v parser.cc; exec bash"
echo "⏳ Gazebo+UGV 加载 ${GAZEBO_LOAD_WAIT}s（机械臂在 paused 下被控制器抓住折叠）..."
sleep "$GAZEBO_LOAD_WAIT"

# ── 物理：★必须 250Hz/0.004(=outdoor_city.world 里为 PX4 锁步设的值)。之前误抄 one_key 的
#    200Hz/0.005 会【打破 PX4 锁步】-> "Time jump detected / Resetting time synchroniser" ->
#    sim 时间抖动 -> TF 时间戳重复被丢(TF_REPEATED_DATA) -> 真值 TF 冻住 -> 点云只堆一处、
#    /odom 也卡。改回 250/0.004 与世界一致,锁步稳、时间不跳,TF/点云/odom 才正常。
#    (one_key 是【非锁步】单机才用 200/0.005;本合并世界的 PX4 是锁步,必须 250/0.004。)
#    保留少迭代 ODE 求解器(50 iters)在重负载下提速。 ──
( source /opt/ros/noetic/setup.bash && rosservice call --wait /gazebo/set_physics_properties "
time_step: 0.004
max_update_rate: 250.0
gravity: {x: 0.0, y: 0.0, z: -9.8}
ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}" ) 2>/dev/null && echo "✅ 物理已设 250Hz/0.004(锁步)" || echo "⚠️  set_physics 失败"

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
  start_ugv_helper:=false uav_auto_align:=true uav_align_off_x:='$UAV_X' uav_align_off_y:='$UAV_Y' \
  use_truth_tf:=true uav_spawn_x:='$UAV_X' uav_spawn_y:='$UAV_Y' enable_octomap:=false \
  low_altitude:='$LOW_ALT' flight_height:='$FLIGHT_H' max_vel:='$MAXV' ground_filter_margin:='$GROUND_FILTER' \
  uav_child_frame:='$UAV_BASE_FRAME' uav_goal_topic:='$UAV_GOAL_TOPIC'; exec bash"
echo "⏳ UAV 节点起 ${ROS_WAIT}s..."; sleep "$ROS_WAIT"

# ── 终端5: 先起飞并稳定，再加载 CPU YOLO/UGV 导航 ──
#    960x540 分割模型加载和首帧推理会瞬时占用多个 CPU 核。如果它与 PX4 EKF 初始化/解锁
#    同时发生，重世界可能产生 time jump，导致本地位姿漂移后才起飞。这里强制错峰。
gnome-terminal --tab --title="5_Takeoff" -- bash -c "source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && python3 -u '$WS/px4_bridge.py' _require_depth_before_takeoff:=false; exec bash"
echo "⏳ UAV 起飞并稳定 ${TAKEOFF_WAIT}s（此阶段不启动 YOLO，避免抢占 PX4）..."; sleep "$TAKEOFF_WAIT"

# ── 终端3.5: UAV 下视地雷分割 → 深度反投影 → map 多帧融合/落盘 ──
if [ "$ENABLE_MINE_DETECTION" = "true" ]; then
gnome-terminal --tab --title="3.5_Mine_Detection" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && \
nice -n 10 roslaunch uav_truth_tracker mine_detection.launch \
  model_path:='$MINE_MODEL' uav_base_frame:='$UAV_BASE_FRAME' confidence:='$MINE_CONFIDENCE' \
  inference_rate:='$MINE_INFERENCE_RATE' torch_threads:='$MINE_TORCH_THREADS' \
  min_map_z:='$MINE_MIN_MAP_Z' max_map_z:='$MINE_MAX_MAP_Z' \
  output_dir:='$MINE_OUTPUT_DIR' enable_sim_evaluator:='$MINE_SIM_EVAL' \
  pause_after_survey_complete:=false manual_only:='$UAV_MANUAL_ONLY'; exec bash"
fi

# ── 终端3.6: 不依赖地雷真值的道路覆盖航线；让 UAV 边飞边建图并给每段留足确认帧 ──
if [ "$ENABLE_MINE_DETECTION" = "true" ] && [ "$MINE_AUTO_SURVEY" = "true" ]; then
gnome-terminal --tab --title="3.6_Mine_Survey" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && \
roslaunch uav_truth_tracker mine_survey.launch goal_topic:='/uav/survey_goal' \
  flight_height:='$FLIGHT_H' dwell:='$MINE_SURVEY_DWELL' \
  candidate_hold_duration:='$MINE_CANDIDATE_HOLD' \
  minimum_confirmed_mines:='$MINE_MIN_CONFIRMED' \
  max_rescan_rounds:='$MINE_SURVEY_RESCAN_ROUNDS' \
  rescan_wait_duration:='$MINE_SURVEY_RESCAN_WAIT' \
  completion_loiter_x:='$MINE_SURVEY_LOITER_X' \
  completion_loiter_y:='$MINE_SURVEY_LOITER_Y'; exec bash"
fi

# ── 终端4: UGV 导航(复用终端1的双EKF; 样例地形图; align 阈值放大不误刷) ──
#    align_warn_threshold:=500 —— 两台相距几十~上百米是正常物理间距, 只有 datum
#    真不一致(~km级)才该报警。(坐标统一靠脚本开头三处 GPS 锚点都钉到 DATUM。)
gnome-terminal --tab --title="4_UGV_Nav" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && \
roslaunch mobile_manipulator ugv_terrain_nav.launch localization:=false use_sample_map:=true rviz:=false align_warn_threshold:=500 \
  live_cloud_topic:='/uav0/mapping/points_accumulated' auto_align:=false \
  aligned_cloud_topic:='/uav0/mapping/points_aligned' \
  align_offset_x:='$UAV_X' align_offset_y:='$UAV_Y' free_border:='$UGV_FREE_BORDER' \
  ur5_joint_damping:='$UR5_JOINT_DAMPING' ur5_shoulder_lift_damping:='$UR5_SHOULDER_LIFT_DAMPING'; exec bash"

# ── 终端4.4: 腕部 RGB-D → 引信定位 → 双层锁定抓取/运输 → 定点释放 Action ──
if [ "$ENABLE_MINE_MISSION" = "true" ] && [ "$MINE_MINIMAL_START" = "true" ]; then
gnome-terminal --tab --title="4.4_Mine_Grasp" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && \
roslaunch mine_grasp_minimal mine_grasp_minimal.launch \
  model_path:='$MINE_MODEL' physical_grasp_enabled:='$MINE_GRASP_PHYSICAL' \
  localization_mode:='$MINE_WRIST_LOCALIZATION_MODE' \
  start_test_manager:=false; exec bash"
echo "⏳ 腕部感知、MoveIt 与抓取 Action 初始化 ${MINE_GRASP_WAIT}s..."; sleep "$MINE_GRASP_WAIT"
fi

# ── 终端4.5: 已确认雷点 -> 最短可达顺序 -> UGV外圈交接/0.62m精停 -> PICK/返航/PLACE ──
#    手动 UAV 模式下 require_survey_complete=false，因此不等待不存在的
#    survey COMPLETE；map 中一有 confirmed 雷点就能派发 UGV。
if [ "$ENABLE_MINE_MISSION" = "true" ]; then
gnome-terminal --tab --title="4.5_Mine_Mission" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && \
roslaunch mobile_manipulator mine_mission.launch \
  auto_start:='$MINE_AUTO_START' start_mock:='$MINE_MOCK_START' mock_mode:='$MINE_MOCK_MODE' \
  require_survey_complete:='$MINE_AUTO_SURVEY' \
  resume_from_file:='$MINE_RESUME' persistence_file:='$MINE_OUTPUT_DIR/mine_mission_state.yaml'; exec bash"
fi

# ── 终端6: 合并版 RViz(带臂 UGV + UAV mesh/点云/路径 + 两个 2D Nav Goal) ──
if [ "$START_RVIZ" = "true" ]; then
gnome-terminal --tab --title="6_RViz" -- bash -c "
source /opt/ros/noetic/setup.bash && source '$WS/devel/setup.bash' && \
rviz -d '$WS/src/mobile_manipulator/rviz/air_ground.rviz'; exec bash"
fi

echo "════════════════════════════════════════════"
echo "✅ 空地协同分终端启动完毕（终端 1 / 1.5 / 2 / 3 / 3.5感知 / 3.6覆盖 / 4导航 / 4.4抓放 / 4.5排雷 / 5 / 6_RViz）"
echo "  • RViz 工具栏有【两个 2D Nav Goal】: 第1个发 UGV(/move_base_simple/goal),"
echo "    第2个发 UAV人工目标(/uav/manual_goal)，仲裁后输出到$UAV_GOAL_TOPIC。"
if [ "$UAV_MANUAL_ONLY" = "true" ]; then
echo "  • UAV 自动起飞稳定后只等待第2个手动目标；不启动/恢复 survey 自动航线。"
fi
echo "  • UGV 应显示带 UR5 机械臂的完整模型(RobotModel), 不是光底盘。"
echo "  • 坐标统一自检: rostopic echo -n1 /mavros/global_position/global 的 lat 应≈$DATUM_LAT(不是47.4);"
echo "    align 偏差应从~280000m 掉到几十米(真实间距)。"
echo "  • TF 应为 map→$UAV_BASE_FRAME(UAV) 和 map→odom→base_link(UGV), 无双 base_link。"
if [ "$ENABLE_MINE_DETECTION" = "true" ]; then
echo "  • 地雷: RViz /mine_detection/markers；调度输入 /mine_detection/map（仅 confirmed 条目）；兼容坐标 /mine_detection/confirmed；文件 $MINE_OUTPUT_DIR/mine_map.yaml"
echo "  • 仿真评分: rostopic echo /mine_detection/sim_evaluation；文件 $MINE_OUTPUT_DIR/simulation_evaluation.yaml"
if [ "$MINE_AUTO_SURVEY" = "true" ]; then
echo "  • UAV覆盖: rostopic echo /mine_survey/status（路线不读取 Gazebo 地雷真值）"
fi
if [ "$ENABLE_MINE_MISSION" = "true" ]; then
echo "  • 排雷状态: rostopic echo /mine_mission/status；当前任务 /mine_mission/current_task"
echo "  • 控制: rosservice call /mine_mission/pause（另有 start/resume/reset）；状态文件 $MINE_OUTPUT_DIR/mine_mission_state.yaml"
echo "  • 机械臂: $MINE_ARM_MODE，通过 /mine_grasp PICK/PLACE Action 接入"
echo "  • Action检查: rostopic type /mine_grasp/status 必须为 actionlib_msgs/GoalStatusArray"
echo "  • 抓取阶段/失败码: rostopic echo /mine_grasp/executor_status"
echo "  • 运送锁: rostopic echo /mine_grasp/retained；携雷速度≤0.25m/s；投放点 /mine_disposal/poses"
echo "  • 只有 PICK→返回HOME附近→PLACE 全部成功后，原地雷危险区才清除并继续下一颗"
echo "  • 最终完成: /mine_mission/all_known_cleared 仅在 UAV 覆盖 COMPLETE 且全部已投放后为 true"
fi
fi
echo "════════════════════════════════════════════"
