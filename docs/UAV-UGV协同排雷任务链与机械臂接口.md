# UAV–UGV 自动往返排雷与机械臂接口

本文对应当前仓库的完整闭环：UAV 边飞边建图和确认地雷，UGV 逐颗接近，用腕部 RGB-D 二次定位并抓住黄色引信，将地雷锁定后低速运回初始点附近的投放区，放下后再处理下一颗。任务管理器会一直监听 UAV 地雷地图；即使一度“全部完成”，后来发现的新地雷仍会自动加入队列。

## 1. 一键启动

先编译：

```bash
cd ~/Air-Ground-Collaborative-Manupulation
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_BLACKLIST_PACKAGES=gpd_ros
source devel/setup.bash
```

默认使用新实现的 `mine_grasp_minimal`：

```bash
GUI=false bash run_air_ground.sh
```

等价的显式写法：

```bash
GUI=false \
MINE_ARM_MODE=minimal \
MINE_GRASP_PHYSICAL=true \
bash run_air_ground.sh
```

可选模式：

| 参数 | 含义 |
|---|---|
| `MINE_ARM_MODE=minimal` | 默认；启动腕部感知、MoveIt、真实 Gazebo 抓取和 PICK/PLACE Action |
| `MINE_ARM_MODE=external` | 使用外部 `/mine_grasp` Action server |
| `MINE_ARM_MODE=mock_success` | 仅测试调度状态机，不代表真实抓取 |
| `MINE_GRASP_PHYSICAL=false` | 阻止 Action 执行物理抓取，用于安全检查 |
| `MINE_AUTO_START=false` | 启动后等待 `/mine_mission/start` |
| `MINE_AUTO_SURVEY=false` | 禁用 UAV 自动覆盖航线，改为手动飞行 |

## 2. 完整数据流

```text
UAV 下视 RGB-D + YOLO11s-seg
  → mask 深度反投影
  → 图像时间戳 TF 转到 map
  → 多帧融合 /mine_detection/map
  → MineHazardLayer 将未处理地雷设为致命障碍
  → mine_mission_manager 选择当前最短可达地雷
  → move_base 进入雷心 0.84 m 外圈（径向误差 ±0.07 m）
  → 位置到达后取消 move_base，用独立低速通道精对准到 0.62 ± 0.020 m / 3°
  → 锁住底盘
  → /mine_grasp operation=PICK
      → 腕部 RGB-D 重新定位黄色引信
      → 解析抓取、夹紧、抬升
      → gazebo_grasp_fix 物理锁 + MoveIt attached
      → 收到紧凑运输姿态
  → 切换到物理锁定返程 DWA 配置（上限 3.4 m/s）
  → move_base 返回初始点附近的独立投放槽
  → 锁住底盘
  → /mine_grasp operation=PLACE
      → 慢速下降
      → 张开夹爪
      → 确认 grasp-fix 解锁
      → MoveIt detach
      → 机械臂竖直上撤并回到 look
  → 原地雷状态变为 CLEARED
  → 投放后的地雷加入 /mine_disposal/poses 危险区
  → 定距后退 0.8 m，恢复正常导航参数并选择下一颗
```

Gazebo 真值不生成抓取目标。UGV 抓取位置只来自腕部 RGB、对齐深度、CameraInfo 和 TF。Gazebo 接触/位姿只用于确认物理锁、抬升和放置结果。

### 2.1 黄点、确认和持久记录

RViz 中黄色球体是尚未确认的 `CAND` 候选点，红色圆柱才是已写入 UGV 任务链的 `CONF` 地雷。融合节点使用两条确认路径：

- 严格快速确认：至少 2 个时间跨度不小于 `0.15 s` 的观测，每帧置信度均不低于 `0.75`，且 map 平面坐标标准差不高于 `0.06 m`；
- 普通多帧确认：至少 3 帧，坐标标准差不高于 `0.10 m`。

这样 CPU 推理的一次快速飞越只要留下两个高质量独立观测就能记录，但单帧误检不会直接派给 UGV。未确认候选保留 `300 s` 以等待 UAV 复查；已确认点在当次任务中不超时，并持续写入：

```text
mine_detection_output/mine_map.yaml
```

一键启动默认推理频率为 `1.0 Hz`、PyTorch 使用 2 个 CPU 线程，自动覆盖点默认驻留 `8 s`。出现新的黄色候选后，覆盖节点还会在当前相机位姿最多悬停 `18 s`，为同一目标补足 2～3 个独立观测；确认完成会提前继续航线。可用 `MINE_INFERENCE_RATE`、`MINE_TORCH_THREADS`、`MINE_SURVEY_DWELL` 和 `MINE_CANDIDATE_HOLD` 覆盖。UGV 只订阅 `confirmed=true` 的条目；地图中还有黄色候选时，不会误报全部排雷完成。

山坡地雷不再使用“世界坐标 z 必须接近 0”的判断。mask 内每个像素都保留自己的深度做反投影，经 MAD 去除离群点后用 depth 图时间戳的精确 TF 整批转换到 `map`。定位器再在 YOLO mask 周围建立深度环，稳健拟合局部坡面 `z=ax+by+c`，并在地雷目标 XY 处计算地面高度，检查相对高度是否位于 `-0.06～0.18 m`。`MINE_MIN_MAP_Z/MINE_MAX_MAP_Z` 只保留为极端数值保护，因此坡上的有效检测不会再因为绝对海拔或环形采样不对称而消失。调试图中的 `h=...` 表示估计的相对地面高度。

实时排查：

```bash
rostopic echo /mine_detection/localizer_status
rostopic echo /mine_detection/map
rostopic echo /mine_mission/current_task
watch -n 1 "grep -E 'confirmed_count|candidate_count' mine_detection_output/mine_map.yaml"
```

## 3. 起点和投放区

`mine_mission_manager` 启动后，在 UGV 尚未接受首个任务前记录：

- `map → base_link`：初始 UGV 位姿和航向；
- `map → base_footprint`：起点地面高度。

该位姿发布在 `MineMission.home_pose`。投放槽相对初始航向生成，默认布局为：

- 第一排中心在起点前方 `1.8 m`；
- 首排最多 5 个槽，按“中心、右、左、更右、更左”顺序向外展开；
- 横向间距 `1.20 m`；
- 后续排的纵向间距 `2.00 m`；
- 放置时底盘距槽中心 `0.65 m`，车头朝向投放槽。

每颗地雷在 PICK 成功时预留唯一槽位，不会全部堆到同一点。调度器同时检查投放中心和 UGV 的 `0.65 m` 停车点；任一位置距尚未处理或已投放地雷过近时都会跳过该槽。这避免第二排停车点压到第一排的地雷。投放区内已确认放下的地雷会被忽略为新任务，避免 UAV 再次看到它们后形成无限循环。

默认参数位于：

```text
src/mobile_manipulator/launch/mine_mission.launch
```

## 4. 双层运输锁

抓住引信后使用两层真实状态，而不是软件变量假装成功：

1. `gazebo_grasp_fix` 在左右指垫形成相向接触后建立 Gazebo 物理固定关节；
2. MoveIt PlanningScene 将整枚地雷作为 attached collision object 挂到 `grasp_tcp`。

夹爪控制器同时保持第一次稳定接触时的实际 joint 位置。运输看门狗每秒检查：

- grasp-fix 是否仍 attached；
- 夹爪 joint 是否偏离接触位置；
- 地雷相对 TCP 是否发生超限平移或旋转；
- MoveIt attached object 是否仍存在。

状态发布：

```bash
rostopic echo /mine_grasp/retained
rostopic echo /mine_grasp/executor_status
rostopic type /mine_grasp/status
```

`/mine_grasp/status` 是 Actionlib 保留话题，其类型必须始终为
`actionlib_msgs/GoalStatusArray`；抓取阶段、失败码和 JSON 诊断发布在
`/mine_grasp/executor_status` 上。不得在 `/mine_grasp/status` 上发布
`std_msgs/String`，否则 UGV 调度器无法连接抓取 Action，会出现到雷前
却不抓取的故障。

CPU-only Gazebo 中，PICK Action 的 wall-time 上限为 `360 s`。这只是给低实时率规划和轨迹执行留出余量；任务管理器仍依据阶段反馈和失败码退出，不会用放大超时掩盖 IK、碰撞或感知失败。

携雷运输姿态以 `transport_pose_xyz=[0.36, 0.0, 0.05]` 为首选，并按顺序检查 `[0.40, 0.0, 0.00]`、`[0.40, 0.0, 0.05]`、`[0.44, 0.0, 0.00]` 备选。每个候选都必须在已附着完整 `0.136 m` 地雷几何的实时 PlanningScene 中通过 IK 和碰撞检查后才执行。旧姿态 `[0.28, 0.0, -0.10]` 已确认与车体碰撞，不再使用。上述新运输姿态在本轮修改后未再进行物理运输运行（按用户要求停止继续测试），因此不得记为已完成往返验证。

`retained=false`、底盘 roll/pitch 超过 `5°` 或 IMU 丢失时，携雷导航立即取消并发布 `/mine_mission/base_lock=true`。调度器不会继续开车或处理下一颗。

唯一允许解锁的位置是投放槽。顺序固定为：

```text
底盘静止并锁住
→ 机械臂慢速下降
→ 张开夹爪
→ 等待 gazebo_grasp_fix attached=false 稳定成立
→ MoveIt detach
→ 上撤
```

## 5. 导航和危险区

未处理地雷和已投放地雷都不能被 UGV 穿越：

- `/mine_detection/map`：UAV 确认的原始危险位置；
- `/mine_disposal/poses`：已经运回起点附近的地雷位置；
- `MineHazardLayer`：将两类位置写入 global/local costmap。

危险圆盘半径为 `0.10 m`，再结合 Husky 的 `1.0 × 0.68 m` footprint 和 inflation layer。这样 UGV 不能从雷上驶过，同时允许车体中心停在 `0.62 m` 的机械臂工作区边缘。

长距离导航和抓取停靠分为两层：move_base 先到达雷心 `0.84 m` 的碰撞安全外圈，径向误差不超过 `0.07 m`。这一交接只检查位置，不要求 move_base 在外圈上完成车头对准；位置到达后取消导航，由任务管理器通过专用 `/mine_mission/fine_cmd_vel`（高于 move_base 的 mux 优先级）以不超过 `0.12 m/s`、`0.25 rad/s` 同时做径向和朝向闭环。连续稳定 `0.6 s` 后才允许机械臂启动。最终 TF 门禁为：

- 雷心距离与 `0.62 m` 停靠半径的误差不超过 `0.020 m`；
- 车头朝向地雷的误差不超过 `3°`。

空载平地出程的 DWA 线速度上限为 `2.8 m/s`；接近后的精对准另行低速控制。地雷进入无反力物理锁定状态后，返程切换为下列配置：

携雷返回前通过 dynamic_reconfigure 将 DWA 限制为：

```text
max_vel_x       = 3.40 m/s
max_vel_trans   = 3.40 m/s
max_vel_theta   = 1.20 rad/s
acc_lim_x       = 3.00 m/s²
acc_lim_theta   = 1.80 rad/s²
```

如果限速服务不可用，默认拒绝解除底盘锁。PLACE 成功后恢复原导航参数。

## 6. PICK/PLACE Action 合同

Action：

```text
/mine_grasp
mobile_manipulator/MineGraspAction
```

Goal 中新增：

```text
PICK=0
PLACE=1
uint8 operation
geometry_msgs/PoseStamped drop_pose
```

PICK 使用腕部视觉，不把 `mine_pose` 直接当毫米级抓取位置。PICK 的 SUCCESS 仅表示地雷已抬起、双层锁定并进入运输姿态，不会清除原危险区。

PLACE 使用任务管理器生成的 `map` 坐标投放中心。只有 PLACE 返回 Action `SUCCEEDED`、`result.success=true`、`outcome=SUCCESS` 且 `/mine_grasp/retained=false` 后，任务才进入 `CLEARED`。

新增任务状态：

```text
CARRYING
RETURNING_HOME
AT_DROPOFF
PLACING
```

完整顺序：

```text
PENDING → NAVIGATING → AT_STANDOFF → WAITING_ARM
→ CARRYING → RETURNING_HOME → AT_DROPOFF → PLACING → CLEARED
```

## 7. 运行监控

```bash
rostopic echo /mine_detection/map
rostopic echo /mine_mission/status
rostopic echo /mine_mission/current_task
rostopic echo /mine_mission/carrying
rostopic echo /mine_mission/all_known_cleared
rostopic echo /mine_grasp/retained
rostopic echo /mine_grasp/executor_status
rostopic type /mine_grasp/status  # 必须是 actionlib_msgs/GoalStatusArray
rostopic echo /mine_disposal/poses
rostopic echo /mine_hazard/active
```

控制：

```bash
rosservice call /mine_mission/pause
rosservice call /mine_mission/resume
rosservice call /mine_mission/reset
```

暂停会立刻取消 move_base/机械臂 Action 并锁住底盘。已经完成 PICK 并进入 `CARRYING/RETURNING_HOME` 时，恢复后先继续返航/投放，不会重新去抓另一颗。若 PICK 正在执行时被中断，且物理锁已经建立但机械臂还没有到运输姿态，则进入 `MANUAL_REQUIRED` 并保持底盘锁定，不会冒险自动行车。物理持雷时拒绝 reset，避免 reset 隐式松开危险物。

默认自动覆盖模式下，`/mine_mission/all_known_cleared=true` 只在 `/mine_survey/status=COMPLETE` 且所有已发现任务都完成投放后发布，避免 UAV 只找到第一颗时就误报“全部排完”。调度器仍保持监听，后续确认的新地雷会使其重新变为 false 并自动开始下一轮。当 `MINE_AUTO_SURVEY=false` 时没有自动覆盖完成门禁，该话题表示“当前已知地雷”全部完成。

## 8. 失败处理

以下情况不会清除原地雷危险区：

- 导航失败或停靠误差过大；
- 腕部检测、深度、TF、IK 或碰撞检查失败；
- 夹爪空闭合；
- Gazebo/MoveIt 任一层未锁定；
- 收臂运输姿态不可达；
- 携雷限速设置失败；
- 运输中掉落或底盘不稳定；
- 返回投放区失败；
- PLACE 未确认物理解锁和 MoveIt detach。

PICK 之前的失败按原策略延后并重试。PICK 成功后的运输/投放失败直接进入 `MANUAL_REQUIRED`，底盘保持锁住，因为此时不能安全地自动假设地雷还在原处或已经放好。

状态持久化到：

```text
mine_detection_output/mine_mission_state.yaml
```

进程重启后不会自动恢复一个中断的持雷动作，因为无法仅凭文件证明 Gazebo/真机夹持仍然有效；该情况恢复为 `MANUAL_REQUIRED`。

### 8.1 本轮故障的判读

- `NO_DETONATOR: colour ROI found no geometrically safe detonator` 且调试图中没有地雷：这是底盘/腕部视野问题，不是 IK 卡死。执行器正在 `WAIT_TARGET`，超时后才安全恢复；先看当前任务是否出现 `fine alignment stable`。
- 夹爪随腕部运动持续抖动：必须完全退出旧 `gzserver` 后重启。Robotiq mimic 插件是 Gazebo 动态库，正在运行的进程不会自动换成重新编译的新版本。
- UAV 图像有 mask、RViz 黄点未转红：查看 `/mine_detection/localizer_status` 和 `/mine_survey/status`。新版本坡面应报告 `ground_rejected=0`，调试图应显示合理的 `h=...`，覆盖状态会短暂进入 `VERIFYING_CANDIDATE` 悬停补帧；确认点仍需满足两帧快速或三帧普通融合门禁。
- 机械臂“走到一半停住”时先看 `/mine_grasp/executor_status` 的 `stage`。只有进入 `PREGRASP/APPROACH` 才是轨迹阶段；`WAIT_TARGET` 表示仍在等待腕部视觉，不能把这段安全等待误判为 MoveIt 死锁。

## 9. 当前实测证据与后续顺序

早期 `0.67 m` 单雷实体链路已在重力开启的 Gazebo 中完成过真实抓取、抬升和保持；当前集成默认停靠距离已改为 `0.62 m`：

- `color_only` 引信中心（`ur5_base_link`）：`(0.48724, 0.00144, -0.32592) m`；
- 三维稳定度：各轴 `std < 1e-7 m`；
- 抓取成功，Gazebo grasp-fix 和 MoveIt attached object 同时成立；
- 世界竖直抬升 `0.0800 m`；
- 3 秒保持期间地雷相对 TCP 平移漂移 `0.17 mm`；
- 底盘最大 pitch `1.066°`、最大 roll `0.0066°`，未超过 `5°` 门禁。

这一证据证明了“腕部颜色+深度定位 → 引信抓取 → 双层 attach → 抬升保持”。它不包含修改后的携雷运输姿态、返回投放区或 PLACE，因此不等于完整 UAV–UGV 往返闭环已通过。

继续完整世界测试时，建议先观察一颗雷，再开放多雷连续运行：

```bash
GUI=false MINE_AUTO_SURVEY=false bash run_air_ground.sh
```

手动让 UAV 只确认第一颗，依次确认：

1. move_base 在 `0.84 ± 0.07 m` 外圈交接，随后 UGV 精停到 `0.62 ± 0.020 m` 且车头朝雷；
2. PICK 后 `/mine_grasp/retained=true`；
3. 机械臂在紧凑运输姿态，地雷未碰车体或地面；
4. 空载 DWA 上限为 `2.8 m/s`，物理锁定返程上限为 `3.4 m/s`；
5. UGV 返回起点附近唯一槽位；
6. 底盘锁住后才开始 PLACE；
7. 放下后 `/mine_grasp/retained=false`；
8. 原雷点清障，投放点成为新的危险障碍；
9. UGV 能离开投放点并继续下一颗。

确认一颗闭环后，直接使用默认自动覆盖：

```bash
GUI=false bash run_air_ground.sh
```
