# UGV 腕部视觉地雷抓取最小系统

`mine_grasp_minimal` 是一个与旧 `master_control.py`、旧巡航状态机和旧抓取偏移完全独立的 ROS1 软件包。第一阶段只完成：腕部 RGB-D 识别地雷、定位黄色引信、UR5 + Robotiq 2F-140 夹住引信、将整枚地雷竖直抬升并保持，以及连续重复测试。

本包不实现车载容器、UAV–UGV 任务调度、多地雷巡航、GPD、GraspGen、腕部力控或完整移动机械臂动力学控制。它现已提供 PICK 以及地面定点 PLACE Action；投放槽分配和多雷循环由 `mobile_manipulator` 中的任务管理器负责。

> 安全默认值：`physical_grasp_enabled:=false`。必须依次通过控制器保持、感知、空载轨迹三个门禁，才可显式改成 `true`。

## 1. 软件结构

```text
mine_grasp_minimal/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── config/
│   ├── perception.yaml
│   └── grasp.yaml
├── launch/
│   ├── controller_hold_test.launch
│   └── mine_grasp_minimal.launch
├── scripts/
│   ├── controller_hold_test.py
│   ├── wrist_mine_localizer.py
│   ├── mine_grasp_executor.py
│   ├── grasp_test_manager.py
│   └── transport_pose_probe.py
├── src/mine_grasp_minimal/
│   └── motion_stability.py
└── README.md
```

职责边界：

- `wrist_mine_localizer.py`：只从 RGB、深度、CameraInfo 和带图像时间戳的 TF 生成引信三维坐标，不读取 Gazebo 地雷真值。
- `mine_grasp_executor.py`：检查接口和底盘静止状态，生成解析抓取候选，执行预抓取、直线接近、接触自适应夹紧、MoveIt attach、竖直抬升和保持。
- `grasp_test_manager.py`：负责测试夹具的随机复位、结果打分和 CSV/JSON 记录。Gazebo 真值只在这里用于复位和验证，绝不作为抓取目标。
- `motion_stability.py`：统一判定 FJT、MoveIt、Cartesian 和夹爪动作的实测终点稳定性，保留原始 action/控制器错误。
- `controller_hold_test.py`：在重力开启时让 look 与配置的 M002 pregrasp 各重复 10 次，再验证 30 秒保持、机械臂下垂、底盘姿态和车轮离地情况。
- `transport_pose_probe.py`：只读扫描带 attached 地雷时的运输 IK/碰撞候选，不执行轨迹。

## 2. 已确认接口

### 2.1 机械臂、夹爪与 MoveIt

| 项目 | 实际接口 |
|---|---|
| UR5 MoveIt group | `ur5_arm` |
| 夹爪 MoveIt group | `hand_e_gripper` |
| 原末端 link | `ur5_tool0` |
| 新抓取 IK/TCP link | `grasp_tcp` |
| MoveIt planning frame | `base_link` |
| UR5 基座 frame | `ur5_base_link` |
| 腕部相机光学 frame | `realsense_camera_optical_frame` |
| UR5 FJT action | `/ur5_arm_controller/follow_joint_trajectory` |
| 夹爪 FJT action | `/gripper_controller/follow_joint_trajectory` |
| 关节状态 | `/joint_states` |
| UR5 控制器状态 | `/ur5_arm_controller/state` |
| 夹爪控制器状态 | `/gripper_controller/state` |

实机仿真检查时，`ur5_arm_controller`、`gripper_controller`、`husky_joint_publisher` 和 `husky_velocity_controller` 均为 `running`。夹爪的实际控制关节是 `finger_joint`，程序没有猜测“4 cm 对应哪个 joint 值”，而是使用当前 Robotiq URDF 的连杆映射和物理接触事件。

检查命令：

```bash
rosservice call /controller_manager/list_controllers
rostopic echo /ur5_arm_controller/state
rostopic echo /gripper_controller/state
rostopic echo /joint_states
```

### 2.2 腕部 RGB-D 相机

| 数据 | 话题 | 实测格式 |
|---|---|---|
| RGB | `/camera/color/image_raw` | `640×480`, `rgb8` |
| 对齐深度 | `/camera/depth/image_raw` | `640×480`, `32FC1`, 单位 m |
| CameraInfo | `/camera/color/camera_info` | `640×480` 内参 |
| 光学 frame | — | `realsense_camera_optical_frame` |

RGB 和深度由同一个 Gazebo OpenNI 深度相机插件产生，分辨率、时间和光学 frame 一致，可按像素直接对齐。`/camera/depth/camera_info` 虽被声明，但实测没有消息，因此定位器明确使用有消息的 `/camera/color/camera_info`；这只适用于当前已对齐仿真相机。

相机 TF 链由 `robot_state_publisher` 发布：`ur5_tool0 → realsense_camera_link → realsense_camera_optical_frame`。光学 frame 遵守 REP-103：`Z` 向前、`X` 向右、`Y` 向下。

### 2.3 Gazebo 地雷

| 项目 | 实际值 |
|---|---|
| 测试 model 名称 | `landmine_test` |
| link | `body` |
| 圆盘 collision | `disc_collision` |
| 引信 collision | `detonator_collision` |
| 圆盘 | 半径 `0.068 m`，高度 `0.025 m`，中心 `z=0.0125 m` |
| 黄色引信 | `0.04 × 0.04 × 0.06 m`，中心 `z=0.055 m` |
| 最高点 | `z=0.085 m` |
| 总质量 | `0.30 kg` |
| 圆盘摩擦 | `mu=mu2=1.0` |
| 引信摩擦 | `mu=mu2=1.2` |

圆盘和引信是同一个非静态刚性 link。`gazebo_grasp_fix` 已安装在机器人模型中，arm 为 `robotiq_140`，检测左右内指的相对接触；`disable_collisions_on_attach=false`。程序从 `/mine_grasp_event_republisher/grasp_events` 验证真实 attach/detach。

## 3. 重力下控制器保持门禁

已在不关闭重力、不修改质量、不扩大底盘质量的前提下执行 30 秒保持测试：

| 指标 | 实测值 | 结论 |
|---|---:|---|
| 最大关节漂移 | `9.267e-7 rad` | 通过 |
| 最大控制器误差 | `1.03e-4 rad` | 通过 |
| 最大 roll | `0.0300°` | 通过 |
| 最大 pitch | `1.1423°` | 通过 |
| 最大车轮上升 | `6.84e-8 m` | 未离地 |
| 保持时间 | `30 s` | 通过 |

复测命令：

```bash
source devel/setup.bash
roslaunch mine_grasp_minimal controller_hold_test.launch hold_duration:=30.0
```

报告发布在 `/mine_grasp/controller_hold_report`。若此门禁失败，应先检查 PID、effort limit、transmission、URDF 质量/惯量和 Gazebo 更新率，禁止通过关闭重力绕过。

## 4. `grasp_tcp` 的几何建立

新增位置：

- URDF：`src/mobile_manipulator/urdf/husky_ur5.urdf.xacro`
- SRDF chain tip/end-effector：`src/husky_ur5_moveit_config/config/husky_ur5.srdf`

Robotiq 当前 URDF 在 `finger_joint=0.499 rad` 时，两侧内指垫中心相对 `robotiq_arg2f_base_link` 为：

```text
y = ±0.023621135212 m
z =  0.198590595984 m
内侧间隙 = 39.742270424 mm
```

因此 TCP 取两指实际接触中心，不使用 `GPD_TOOL_OFFSET`、`FINGER_REACH` 或世界坐标 magic offset：

```xml
<joint name="grasp_tcp_joint" type="fixed">
  <parent link="robotiq_arg2f_base_link"/>
  <child link="grasp_tcp"/>
  <origin xyz="0 0 0.1985905960" rpy="0 0 1.5707963268"/>
</joint>
```

绕 `Z` 旋转 `+π/2` 后，`grasp_tcp +X` 是夹爪闭合方向，`grasp_tcp +Z` 是接近方向。

RViz 验证：

1. 启动 `robot_state_publisher` 和 MoveIt；
2. 在 RViz 添加 `TF`，勾选 `grasp_tcp`；
3. 确认原点位于两指垫之间；
4. 用空载 dry-run 检查沿 `grasp_tcp +Z` 的运动确实朝向地雷；
5. 确认闭合时黄色引信位于两片指垫接触带内。

## 5. 腕部视觉定位方法

定位器支持两种模式：

- `hybrid`（独立包默认）：YOLO 地雷 mask 内优先寻找黄色引信，YOLO 漏检时才启用受限颜色 ROI。
- `color_only`（`run_air_ground.sh` 默认）：UAV 已完成地雷识别和任务选取后，腕部节点不再加载第二份 YOLO，直接在受限 ROI 中做 HSV+深度定位，以降低 CPU 占用和 Gazebo 时序抖动。

两种模式都不读取 Gazebo 真值。`color_only` 也不是全图寻找黄色：候选仍须通过面积、长宽比、填充率、有效深度、UR5 工作空间和连续三帧稳定门禁。定位链路为：

```text
YOLO11s-seg 地雷 mask（color_only 时跳过）
→ 仅在地雷 mask 或受限 ROI 内做黄色 HSV 分割
→ 形态学开闭运算与向内腐蚀 2 px
→ 删除 0/NaN/越界深度
→ 深度中位数 + MAD 异常值剔除
→ 每个黄色像素按自身深度反投影
→ 使用图像时间戳将完整点云转换到重力坐标系
→ 用顶部稳健分位数和顶部点带估计引信顶面
→ 拟合局部地面并验证顶面高度约 85 mm
→ 沿重力方向下移 30 mm 得到引信中心
→ 转换到 ur5_base_link
→ 连续 3 帧稳定性门禁
```

协同任务先由 move_base 进入雷心 `0.84 ± 0.07 m` 的碰撞安全外圈。外圈交接只检查位置；取消 move_base 后，独立低速 `/mine_mission/fine_cmd_vel` 通道再将径向/偏航误差闭环消除到 `0.67 ± 0.020 m` 和 `3°` 以内，然后才允许机械臂动作。若调试图完全看不到黄色引信，应先检查该停靠门禁，而不是放宽 HSV 或给 TCP 增加偏移。

反投影使用 CameraInfo 的真实内参：

```text
X = (u - cx) Z / fx
Y = (v - cy) Z / fy
Z = depth
```

深度最少保留 15 个有效像素；范围为 `0.05～3.0 m`。黄色区域只允许出现在 YOLO 地雷 mask 或配置的受限 ROI 内，不会在整幅图中无约束寻找黄色。RGB-D 观测首先得到引信顶面，再按 SDF 中 `60 mm` 的引信高度沿重力方向转换为引信物理中心。

模型参数：

- 权重：`mine_seg_v2_delivery/weights/best.pt`
- 类别：`0: landmine`
- `imgsz=960`
- CPU 推理，默认 `2 Hz`
- 腕部初始置信度：`0.45`，可通过 launch 参数 `confidence:=...` 调整
- UAV 验证推荐值 `0.7337` 没有被写死到腕部相机

有效目标必须同时满足：连续至少 3 帧、`xy/z std ≤ 0.01 m`、黄色面积合理、深度样本足够、图像时间戳 TF 成功。在固定场景中用 Gazebo 真值做过一次只读验证，视觉目标误差约 `1.42 mm`；该真值没有进入定位或抓取生成。

输出：

```text
/mine_grasp/target_observation  geometry_msgs/PoseWithCovarianceStamped
/mine_grasp/target_pose         geometry_msgs/PoseStamped（兼容调试）
/mine_grasp/target_valid        std_msgs/Bool（兼容调试）
/mine_grasp/debug_image         sensor_msgs/Image
/mine_grasp/confidence          std_msgs/Float32（兼容调试）
/mine_grasp/target_std          std_msgs/Float32MultiArray（兼容调试）
/mine_grasp/localizer_status    std_msgs/String
/mine_grasp/motion_diagnostics diagnostic_msgs/DiagnosticArray
/mine_grasp/debug_markers       visualization_msgs/MarkerArray
/mine_grasp/tcp_trace           nav_msgs/Path
```

执行器只接受抓取动作开始后产生、时间戳推进且与冻结地图先验匹配的原子
`target_observation`。每次尝试的图像、感知 JSON 和执行 JSON 均保存到独立的
`mine_grasp_results/<attempt_id>/` 目录，不会被后续失败帧覆盖。

## 6. 解析抓取与执行流程

第一版只生成确定的解析候选：优先 top-down，候选 yaw 为 `0°/90°`；若不可达，再尝试 `±10°` 小倾角。每个候选都分别验证 pregrasp、approach 和 grasp 三个位姿的碰撞感知 IK，并拒绝关节极限、手臂接近完全伸直和超出稳定工作区的解。

完整执行顺序：

```text
检查控制器、MoveIt、IMU 和里程计
→ 持续发布零速度并确认底盘静止
→ 移动到 look 姿态并验证保持
→ 等待稳定视觉目标
→ 张开夹爪
→ 添加地面/圆盘碰撞保护
→ 选择解析 IK 候选
→ MoveIt 移动到 pregrasp
→ 第一次腕部重新定位并重新验证 IK/碰撞
→ 低速笛卡尔移动到 coarse approach
→ 第二次腕部重新定位并重新验证 IK/碰撞
→ 低速笛卡尔直线下降到 grasp
→ 分阶段寻找真实双指接触
→ 接触位置上只追加 0.003 rad 微夹紧
→ 连续 1 秒确认 gazebo_grasp_fix attach
→ MoveIt attachObject
→ 沿 odom 世界竖直方向抬升 0.08 m
→ 保持 3 秒并监测掉落、滑移和底盘姿态
```

40 mm 方形引信旋转后投影宽度会改变，因此固定闭合 joint 值会过夹或空夹。本实现依次尝试 `q=[0.480, 0.490, 0.495, 0.498, 0.499, 0.500]`；一旦收到真实相对接触事件，就以实测接触位置为基准仅增加 `0.003 rad`，速度不超过 `0.01 rad/s`。这避免了旧方案固定闭合值把地雷推出或把车体翘起。

抓取高度也不是魔法 TCP 偏移：指垫接触面和引信均高 `60 mm`，将接触带相对引信中心上移 `8 mm`，可保留 `52 mm` 夹持重叠，同时让指垫下边缘与宽圆盘保持 `8 mm` 间隙。

安全参数：

```yaml
arm_velocity_scale: 0.08
arm_acceleration_scale: 0.04
approach_speed: 0.02
lift_speed: 0.03
maximum_target_planar_reach: 0.58
max_base_roll_deg: 5.0
max_base_pitch_deg: 5.0
lift_distance: 0.08
hold_duration: 3.0
```

任何轨迹执行期间都会持续发布零 `/cmd_vel`，并监测 IMU roll/pitch。超限会取消动作并缓慢回到已验证的低伸展 look 姿态。

两次重新定位都按同一规则处理：更新量 `≤20 mm` 时重建目标并重新做
IK/碰撞检查，`20–50 mm` 时退回 pregrasp 重新规划，`>50 mm` 时报告
定位/停车错误并禁止下降。最终下降分别记录横向、竖直和姿态误差。

FJT、MoveIt 与 Cartesian 执行共享实测终点稳定门。控制器返回
`GOAL_TOLERANCE_VIOLATED` 时最多额外观察 3 秒，只有同时满足下列条件才可
继续：最大关节误差 `≤0.01 rad`、最大关节速度 `≤0.05 rad/s`、连续
`0.5 s` 关节跨度 `≤0.002 rad`、TCP 位置/姿态误差 `≤8 mm/3°`，且底盘
roll/pitch 均 `≤5°`。夹爪还必须满足实测指关节稳定和新鲜的 grasp-fix
接触事件，不能无条件接受控制器终点错误。

## 7. MoveIt 与 Gazebo 状态一致性

夹紧后必须同时满足两层状态：

- Gazebo：`gazebo_grasp_fix` 发布 `attached=true`，地雷随夹爪运动；
- MoveIt：`detected_landmine` 作为 AttachedCollisionObject 附着到 `grasp_tcp`，touch links 仅包含夹爪相关 link。

成功条件不是“夹爪命令执行完”：还必须确认非空抓取、Gazebo attach、MoveIt attach、离地至少 `45 mm`、目标随 TCP 运动、相对平移小于 `15 mm`、相对旋转小于 `8°`，且保持 3 秒未掉落。

重复测试复位同样遵守物理生命周期：

1. 取消上一动作；
2. 发送 open goal；
3. 用夹爪实际关节位置确认已张开，失败最多重试 3 次；
4. 等待真实 `attached=false` 并稳定 1 秒；
5. 才清理 MoveIt attached/world object 和 octomap；
6. 才允许测试管理器移动地雷夹具。

代码不会通过强制清本地布尔值代替物理 detach，也不会在 grasp-fix 仍持有内部 joint 时删除 Gazebo 模型。

## 8. 状态与失败原因

执行器的 JSON 状态发布在 `/mine_grasp/executor_status`，每次完整
JSON 报告发布在 `/mine_grasp/report`。`/mine_grasp/status` 是
Actionlib 保留话题，类型必须是 `actionlib_msgs/GoalStatusArray`，不能用于
发布 `std_msgs/String` 诊断，否则 Action 客户端会因类型冲突而永远无法
连接。明确失败码：

```text
NO_DETECTION
NO_DETONATOR
INVALID_DEPTH
TF_FAILED
TARGET_UNSTABLE
PLANNING_FAILED
CONTROLLER_UNSETTLED
TRUE_POSITION_ERROR
TCP_NOT_REACHED
MOTION_TIMEOUT
IK_FAILED
COLLISION_FAILED
PREGRASP_FAILED
APPROACH_FAILED
TARGET_SHIFT_EXCESSIVE
GRIPPER_FAILED
EMPTY_GRASP
GAZEBO_ATTACH_FAILED
MOVEIT_ATTACH_FAILED
LIFT_FAILED
OBJECT_DROPPED
BASE_UNSTABLE
TRANSPORT_FAILED
PLACE_FAILED
CANCELLED
INTERNAL_ERROR
```

## 9. 严格测试顺序与命令

所有命令均在仓库根目录执行，每个终端先运行：

```bash
source devel/setup.bash
```

### 9.1 启动无 GUI 仿真

```bash
roslaunch mobile_manipulator spawn_outdoor_city.launch gui:=false
```

不要同时启动第二个同名 Gazebo/ROS world。确认控制器均为 `running` 后再继续。

### 9.2 单独验证机械臂保持

```bash
roslaunch mine_grasp_minimal controller_hold_test.launch hold_duration:=30.0
```

只有 `/mine_grasp/controller_hold_report` 中 `passed=true` 才继续。

### 9.3 单独验证感知

若世界中还没有测试地雷，可生成一个仅供相机观察的动态模型：

```bash
/opt/ros/noetic/lib/gazebo_ros/spawn_model \
  -sdf \
  -file src/mobile_manipulator/gazebo_models/landmine/model.sdf \
  -model landmine_test \
  -x 0.63 -y -0.08 -z 0.10
```

只启动定位器：

```bash
roslaunch mine_grasp_minimal mine_grasp_minimal.launch \
  start_executor:=false \
  start_test_manager:=false \
  start_move_group:=false \
  start_grasp_event_republisher:=false
```

检查：

```bash
rostopic echo /mine_grasp/localizer_status
rostopic echo /mine_grasp/target_pose
rqt_image_view /mine_grasp/debug_image
```

应看到连续 `VALID`、合理的 `ur5_base_link` 坐标，以及只在地雷 mask/受限 ROI 内标出的黄色引信。

协作项目默认使用低负载颜色定位，也可以显式比较两种模式：

```bash
MINE_WRIST_LOCALIZATION_MODE=color_only bash run_air_ground.sh
# 重新启用腕部 YOLO+HSV：
MINE_WRIST_LOCALIZATION_MODE=hybrid bash run_air_ground.sh
```

### 9.4 空载 pregrasp/approach/lift

默认即为 dry-run：

```bash
roslaunch mine_grasp_minimal mine_grasp_minimal.launch \
  physical_grasp_enabled:=false \
  start_test_manager:=false
```

执行：

```bash
rosservice call /mine_grasp/reset_executor
rosservice call /mine_grasp/execute
```

dry-run 会验证视觉、IK、碰撞、直线接近和竖直抬升轨迹，但在目标上方保留 `0.10 m` 安全净空，不闭合、不 attach。

### 9.5 单次实物抓取

确认前三个门禁全部通过后，才可显式开启：

```bash
roslaunch mine_grasp_minimal mine_grasp_minimal.launch \
  physical_grasp_enabled:=true \
  start_test_manager:=false
```

```bash
rosservice call /mine_grasp/reset_executor
rosservice call /mine_grasp/execute
```

第一阶段到抬升并保持结束，不会转向容器或释放到容器。

### 9.6 连续 10 次抓取

```bash
roslaunch mine_grasp_minimal mine_grasp_minimal.launch \
  physical_grasp_enabled:=true \
  start_test_manager:=true \
  test_count:=10
```

```bash
rosservice call /start_grasp_test
```

停止或复位：

```bash
rosservice call /reset_grasp_test
rosservice call /mine_grasp/reset_executor
```

进度和摘要：

```bash
rostopic echo /mine_grasp/test_progress
rostopic echo /mine_grasp/test_summary
```

每次运行生成：

```text
mine_grasp_results/run_YYYYMMDD_HHMMSS/
├── trials.csv
├── trial_01.json
├── ...
├── trial_10.json
└── summary.json
```

CSV/JSON 包含检测、置信度、三维稳定度、IK、碰撞、接触 joint、MoveIt/Gazebo attach、实测抬升、滑移、roll/pitch、车轮离地、机械臂下垂、耗时和失败原因。

## 10. 当前实测证据与历史连续测试

### 10.1 当前 `0.67 m` 单雷实测

在重力开启的 Gazebo 中，当前代码已完成一次“`color_only` 定位 → 抓住黄色引信 → Gazebo/MoveIt 双层 attach → 世界竖直抬升 → 保持 3 秒”：

- 引信中心（`ur5_base_link`）：`(0.48724, 0.00144, -0.32592) m`；
- 各轴定位 `std < 1e-7 m`；
- 抓取成功，MoveIt attached object 和 Gazebo grasp-fix 均已确认；
- 抬升 `0.0800 m`，3 秒保持期间相对 TCP 漂移 `0.17 mm`；
- 最大 pitch `1.066°`、最大 roll `0.0066°`。

该结果验证了当前感知和抓取/抬升链路，不包含修改后的 transport、返程或 PLACE。

### 10.2 历史 standalone 10 次基线

以下是旧 standalone 测试配置的运行目录：

```text
mine_grasp_results/run_20260713_051123
```

随机种子为 `41`，每轮将地雷放到随机 `x/y/yaw`，但这些值只进入测试夹具和评分器。目标位姿始终来自腕部 RGB-D。这份 `10/10` 是历史 standalone `/mine_grasp/execute` 基线，不是协作任务中的 PICK/transport/RETURN/PLACE Action 成功率，也不代表当前修改后的运输姿态已验证。

| 次数 | x (m) | y (m) | yaw (°) | conf | 最大 xyz std (mm) | 首次接触 q | 抬升 (mm) | TCP 相对滑移 (mm) | 最大 pitch (°) | 下垂误差 (mrad) | 耗时 (s) | 结果 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 0.622 | -0.090 | -10.0 | 0.8259 | 0.0082 | 0.47948 | 80.0 | 0.420 | 1.119 | 1.221 | 57.11 | 成功 |
| 2 | 0.644 | -0.076 | 5.7 | 0.8479 | 0.0007 | 0.48925 | 80.1 | 0.163 | 1.111 | 1.221 | 58.81 | 成功 |
| 3 | 0.629 | -0.084 | 6.9 | 0.8441 | 0.0069 | 0.47982 | 80.1 | 0.065 | 1.111 | 1.215 | 55.95 | 成功 |
| 4 | 0.630 | -0.068 | 4.8 | 0.8458 | 0.0001 | 0.48937 | 80.2 | 0.059 | 1.112 | 1.221 | 59.50 | 成功 |
| 5 | 0.641 | -0.081 | -10.3 | 0.8513 | 0.0011 | 0.47943 | 80.0 | 0.503 | 1.121 | 1.219 | 55.76 | 成功 |
| 6 | 0.613 | -0.070 | -10.0 | 0.8151 | 0.0002 | 0.47948 | 80.0 | 0.257 | 1.118 | 1.220 | 56.26 | 成功 |
| 7 | 0.633 | -0.074 | -13.2 | 0.8399 | 0.0048 | 0.47920 | 80.0 | 0.291 | 1.126 | 1.223 | 57.44 | 成功 |
| 8 | 0.638 | -0.098 | 11.0 | 0.8480 | 0.0007 | 0.47939 | 80.1 | 0.119 | 1.112 | 1.215 | 56.37 | 成功 |
| 9 | 0.641 | -0.090 | 6.8 | 0.8432 | 0.0006 | 0.47972 | 80.2 | 0.054 | 1.111 | 1.218 | 56.13 | 成功 |
| 10 | 0.636 | -0.094 | 5.1 | 0.8431 | 0.0007 | 0.48932 | 80.2 | 0.079 | 1.112 | 1.217 | 58.45 | 成功 |

汇总：

- 检测成功：`10/10 = 100%`，超过 `≥90%` 目标；
- 抓取抬升成功：`10/10 = 100%`，超过 `≥80%` 目标；
- 失败原因统计：`{}`，没有失败码；
- 置信度均值 `0.840439`，范围 `0.815098～0.851325`；
- 首次物理接触 joint 范围 `0.479195～0.489370 rad`，证明需要接触自适应而非固定闭合值；
- 抬升均值 `80.090 mm`，范围 `79.971～80.193 mm`；
- 3 秒保持期间最大相对平移 `0.503 mm`、最大相对旋转 `0.076954°`；
- 最大 roll `0.023235°`、最大 pitch `1.126378°`，均远低于 `5°` 停止阈值；
- 车轮离地 `0/10`，掉落 `0/10`；
- 最大机械臂跟踪/下垂误差 `0.001223 rad`，没有持续下垂；
- 单次平均耗时 `57.179 s`。

该历史基线的原始证据为 `trials.csv`、10 个逐次 JSON 和 `summary.json`。开发期间保留的其他运行目录用于记录并修复 wall-time、stale planning scene、Gazebo 删除竞态、固定闭合值和 grasp-fix 释放竞态，不应混作当前协作 Action 成功率。

## 11. 已知限制

- 当前验证对象是仓库内指定的单一 Gazebo 地雷几何和摩擦参数，真实硬件需重新做相机内外参、深度噪声、夹爪宽度和摩擦标定。
- `hybrid` 模式下 YOLO 在本机 CPU 上限制为约 `2 Hz`；协作启动默认用 `color_only`，避免腕部第二份 YOLO 与 Gazebo/MoveIt 争抢 CPU。颜色模式依赖场景内黄色干扰物较少且相机已朝向当前目标。
- 模型主要由 UAV 数据训练，腕部视角使用了缩放画布和较低可调阈值；若真实腕部域差明显，应补采腕部 RGB-D 数据微调。
- 当前只生成 top-down、`0°/90°` yaw 和 `±10°` 倾角候选，不是任意 6D 抓取规划器。
- 没有腕部力/力矩传感器，接触依据 Gazebo 双指接触事件；真实机器人需换成夹爪电流、位置残差或力传感状态。
- 没有在线全车 COM 动力学控制，只实现静止底盘、低速、稳定工作区、roll/pitch 和车轮离地门禁。
- 必须存在 `robot_state_publisher` 的完整 TF 链和 `odom` 重力参考 frame；缺失会明确返回 `TF_FAILED`。
- 必须加载 `gazebo_grasp_fix`；缺失或未确认物理 attach 会返回 `GAZEBO_ATTACH_FAILED`，不会仅靠 MoveIt attach 宣告成功。
- Robotiq 的五个 mimic joint 由 Gazebo 插件驱动；修改/重编译插件后必须结束旧 `gzserver` 并完整重启世界。当前插件只在真实关节误差超过 `0.0001 rad` 时纠偏并保留从动指的世界速度，以避免腕部运动时高频抖动。
- 本包不分配车载容器或多雷路线；地面定点 PLACE、投放槽和多雷任务循环由 `mobile_manipulator` 任务管理器通过 Action 调用。

## 12. 后续容器投放接口预留

当前执行器在成功后保持：

- 地雷仍由 Gazebo 物理 joint 附着；
- `detected_landmine` 仍在 MoveIt 中附着到 `grasp_tcp`；
- `/mine_grasp/report` 已包含目标、抓取和稳定性结果；
- `/mine_grasp/executor_status` 的 JSON `code` 为 `GRASP_SUCCESS`。

现已在第二阶段加入地面投放接口（见下一节），用于运回起点附近后放到独立槽位；车载容器、容器内放置和更高层 UAV–UGV 调度仍不属于本包本身。

## 13. UAV–UGV 往返排雷接入（第二阶段）

当前执行器现已直接提供 `mobile_manipulator/MineGraspAction`，同一个
`/mine_grasp` Action 通过 `operation=PICK/PLACE` 完成完整运输闭环：

```text
PICK：腕部定位 → 抓取抬升 → grasp-fix + MoveIt 双锁
    → attached 地雷参与碰撞检查
    → 首选 transport_pose_xyz=[0.36, 0.0, 0.05]
      （失败时依次检查配置中的三个安全备选）
PLACE：到达 map 下 drop_pose → 慢速下降 → 张开夹爪
    → 确认 grasp-fix 解锁 → MoveIt detach → 上撤
```

新增的 `/mine_grasp/retained` 是物理运输门禁。它只在 Gazebo 固定关节、
MoveIt attached object、夹爪接触位置和相对位姿都通过后才为 `true`。运输
看门狗会保持实际接触 joint；任何意外 detach 会立即把它改为 `false`。
已持雷时任何新 PICK 都会被拒绝，因此不会因为重复任务误张开夹爪。
解锁只出现在 PLACE 中，且必须等到 `gazebo_grasp_fix`
真实发布 `attached=false` 后才会从 MoveIt 解除附着。

完整一键任务由：

```bash
GUI=false MINE_ARM_MODE=minimal bash run_air_ground.sh
```

启动。任务调度、起点投放槽、携雷限速、危险区转移和失败恢复详见：

```text
docs/UAV-UGV协同排雷任务链与机械臂接口.md
```

本阶段是在已完成的单雷 10/10 抓取结果上增加运输和放置逻辑；不要把前述
单雷结果误写成完整五雷往返结果。完整 Gazebo 世界需要按新文档第 9 节由用户
先进行一颗雷闭环测试，再开放自动连续任务。
