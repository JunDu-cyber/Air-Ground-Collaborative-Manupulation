# 空地协同操作系统 — 项目进展总结（供分析用）

> 本文是一份**自包含**的项目进展说明，用于发给外部模型分析。读者没有仓库访问权限，因此本文把架构、数据流、关键工程决策（含"为什么"）、以及尚未解决的问题都写清楚了。
> 仓库：`Air-Ground-Collaborative-Manupulation`（ROS Noetic / Ubuntu 20.04 / Gazebo 11）。

---

## 1. 项目目标（一句话）

构建一个**空地协同自主系统**：先由**无人机（UAV，PX4 + EGO-Planner）自主飞行建图**，把环境扫成点云 / 高程图；再由**地面车（UGV：Husky + UR5 机械臂 + Robotiq 2F-140 夹爪）**利用这张图做**定位、地形感知导航与抓取操作**。最终愿景是"无人机看路、地面车干活"的协同操作（air-ground collaborative manipulation）。

整体链路：

```
UAV 自主飞行建图   →   点云转高程图        →   UGV 定位 + 地形感知导航 + 抓取
PX4 + EGO-Planner      elevation_mapping        AMCL/EKF + move_base + MoveIt
   ↓                       ↓                          ↓
~/pointcloud_maps/     /elevation_mapping/        Husky 自主驾驶 + UR5 抓取
*.pcd                  elevation_map_postproc
```

---

## 2. 当前系统组成

### 地面车（UGV）侧——较成熟
- **平台**：Clearpath Husky（差速四轮）+ Universal Robots UR5（6 DOF，850 mm）+ Robotiq **2F-140** 平行夹爪（140 mm 行程）。
- **室外定位**：双 EKF + GPS。局部 EKF（轮速里程计 + IMU → `odom→base_link`）；`navsat_transform_node` 把仿真 GPS 转成 map 系里程计；全局 EKF 融合 GPS → `map→odom`。
- **室内定位**：SLAM Toolbox（位姿图 SLAM）。
- **导航**：ROS Navigation Stack（navfn 全局 + DWA 局部）。
- **运动规划**：MoveIt + OctoMap + OMPL(RRTConnect) + KDL IK。
- **感知**：YOLOv8m（ONNX→TensorRT，GPU 推理）+ RGB-D 深度图转 PointCloud2 做 3D 物体定位。
- **LLM Agent**：OpenAI 兼容 API（DeepSeek/Groq/Ollama/OpenAI）的工具调用，自然语言下达任务；Flask Web UI 仪表盘。
- **地形建图（地面车视角）**：ANYbotics `elevation_mapping` + ETH `grid_map`，把无人机俯视点云融合成以车为中心的高程 + 坡度栅格图。

### 无人机（UAV）侧——本阶段主攻，已打通
- **PX4 SITL（v1.14.3）+ Gazebo Classic 11 + MAVROS**，lockstep 物理仿真。
- **EGO-Planner（ZJU-FAST-Lab）**：局部 B 样条 rebound 规划器，自主避障飞行。
- **机载传感**：俯视深度相机（iris_depth_camera 机型）+ Velodyne LiDAR。
- **点云累积导出**：飞行中把世界系点云累积保存为 PCD（`uav_truth_tracker`），随时手动 / 退出时自动存盘。

### 关键脚本 / 入口
| 文件 | 作用 |
|------|------|
| `setup_uav.sh` | 一键安装 UAV 仿真环境（ROS 依赖、克隆 EGO-Planner、装 PX4 SITL、装 MAVROS 地理数据集、配置 Gazebo 路径、应用 `patches/`、编译） |
| `one_key_takeoff.sh` | 一键启动 Gazebo 世界 + PX4 + MAVROS + EGO-Planner 自主飞行建图 |
| `forest_uav_mapping.launch` | `one_key_takeoff.sh` 内部调用，暴露 `flight_height` / `ground_filter_margin` / `map_size` 等关键参数 |
| `pcd_to_elevation.launch` | 离线：PCD → 体素降采样 → elevation_mapping → 高程图层 |
| `bringup.sh` | 室内 LLM 驱动的抓取演示（Agent + Web UI） |
| `patches/` | 对 EGO-Planner 源码（`grid_map.*`、`planner_manager.cpp`、`ego_replan_fsm.cpp`、RViz、scripts）的本地改动，安装时复制进 `src/ego-planner/` |

---

## 3. 本阶段做了什么（按根因分类的修复）

本阶段表面问题都是"跑不起来 / 飞着飞着卡死"，实则是 5 个**互相独立**的根因，逐个定位解决：

1. **`rosrun` 没装** → `octomap_manager.py` / `bt_to_2dmap.py` spawn `rosrun` 时 `FileNotFoundError` 把节点搞死。
   修复：改用 `roslib.packages.find_node(package, exe)` 直接解析二进制路径，绕开 rosrun。

2. **PX4 lockstep 物理频率不对** → 世界 `real_time_update_rate=100`，但 PX4 lockstep 要求是 **250 Hz 的整数倍**，否则 gzserver abort → `/clock` 冻结 → 无人机标记 / 目标点点击 / UGV 全死。
   修复：`outdoor_city.world` 物理块改 `max_step_size 0.01→0.004`、`real_time_update_rate 100→250`。

3. **RViz 无人机模型指错 topic** → 模型停在带偏移的 Gazebo 世界系调试位姿处，一个"无形的无人机"在跟轨迹动。
   修复：RViz Marker topic `/gazebo_drone_visual → /drone_visual`（后者由 `mavros_tf_bridge.py` 按 `/mavros/local_position/odom` 实时发布，跟随真实无人机）。

4. **EGO 对大体量建筑避障极弱（核心约束）** → 直线初始轨迹一旦扎进楼体，局部优化弹不出来，`first_optimize_step_success=0` 死循环，原地卡死；点别的目标点也不动。
   修复策略：**飞到所有建筑之上、从楼顶往下俯扫**——
   - `flight_height` 默认 **12 m**（`outdoor_city` 楼最高约 11 m）；
   - `ground_filter_margin` 默认 **10**（EGO 避障地图忽略 z ≤ 10 m 的所有点，巡航高度时地图近乎为空 → 不撞楼、不卡）。

5. **EGO 规划地图有边界** → 地图以世界原点为中心、`map_size` 决定覆盖范围；目标点超出 ±(map_size/2) 直接 `terminal in obstacle` 规划失败，表现为"飞着飞着卡住"。
   修复：`map_size_x/y 60→200`（即 x,y ∈ [−100,100] m），`grid_map/resolution 0.15→0.3`（放大地图同时控内存）。

**调参过程中的一段弯路**：曾尝试把 LiDAR 点云喂给 EGO 做低空精细避障，并加大膨胀（inflation 0.45、dist0 0.5），结果把城区本来能穿过的缝也堵死，更容易卡。用户反馈"明明能从旁边绕"/"别搞了就用官方的吧"，于是**全部回滚到 EGO 官方配置，只保留 `dist0=0.4` 一处**，并移除 LiDAR 喂入开关。

---

## 4. 完整数据流（含 topic / frame）

```
[UAV 飞行建图]
 PX4 SITL ──MAVROS──> /mavros/local_position/odom ──> mavros_tf_bridge.py
                                                        ├─> /drone_visual (RViz mesh marker)
                                                        └─> TF: map → base_link
 俯视深度相机 ─> /depth/image_dilated ─> EGO grid_map（避障，巡航高度时被 ground_filter_margin 清空）
 Velodyne LiDAR ─> 世界系点云累积 ─> /uav_pointcloud_map_recorder
      └─> ~/pointcloud_maps/uav_points_map_latest.pcd  (+ 带时间戳版本)
 EGO-Planner: A* 前端 + B样条后端，FSM(GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ)，自主飞向目标

[PCD → 高程图]  pcd_to_elevation.launch
 PCD 回放 ─> 体素降采样 ─> elevation_mapping ─> grid_map
      ├─> /elevation_mapping/elevation_map_raw          (原始融合)
      └─> /elevation_mapping/elevation_map_postprocessed (inpaint→法线→slope层)

[UGV 导航]
 高程/坡度图 ─(规划中)→ grid_map_costmap_2d 可通行层 ─> move_base
 双EKF+GPS 定位 ─> map→odom ─> AMCL/move_base ─> /cmd_vel ─> Husky
```

**世界系一致性规则**：UAV 建图、PCD、UGV 导航必须共用同一世界原点 / 坐标系，否则高程图与地面车定位对不上。

---

## 5. 关键工程决策与本质约束

- **为什么城区建图必须从楼顶俯扫？** EGO-Planner 是**局部规划器**，没有全局路径搜索能"绕过"大体量实心建筑。楼高 ~11 m ⇒ 不存在"低空 + 安全避障"的中间地带：要么飞够高（地图清空、绝不撞）、要么贴墙低飞（必卡）。**真正的低空贴墙建图需要换带全局前端的规划器**。

- **俯扫的代价（值得分析的点）**：俯视飞行把地面细节（z<1 m，约 51% 点）扫得不错，但 **z∈[1,4] m 的"半墙"区域有空洞**——这是自上而下视角的固有缺陷。结论是：这对 UGV 的 **2D/2.5D 导航无害**（导航只需要地面高程 + 建筑物占地footprint，二者都拿到了），但若后续要做更精细的三维理解会受限。

- **`flight_height` 的权衡**：压低能拿到更密的低处细节，但**不能低于楼高+1 m**，否则撞楼 / 卡死。默认 12 m 是"安全俯扫"与"细节"的折中。

- **`map_size` 与 `resolution` 的权衡**：扫更远要同时加大 `map_size` 并调粗 `resolution`，否则栅格内存爆。当前 200 m / 0.3 m 分辨率。

- **安装产物 gitignore**：`~/PX4-Autopilot`、`src/ego-planner/` 由 `setup_uav.sh` 生成、未提交；`patches/` 是受控改动，安装时复制进去。队友重新 clone 后跑 `setup_uav.sh` 即可复现。

---

## 6. 已知限制 / 待解决问题（建议重点分析）

1. **EGO 局部规划器对大建筑无能为力**——目前靠"飞高俯扫"规避，但这堵死了低空贴墙精细建图的路。是否值得引入带全局前端的规划器（如 Fast-Planner kino-A* / EGO 全局版 / 或在 EGO 外面套一层全局路径）？

2. **z∈[1,4] m 半墙空洞**——自上而下视角固有。若协同操作需要侧面信息，是否需要补斜视航线 / 多高度分层扫 / 加侧视相机？

3. **目标点边界**——`map_size` 限定 ±100 m，超界即规划失败。大场景建图需要**滑动 / 滚动地图**或分区扫描策略，目前没有。

4. ~~PCD → 高程图 → costmap 还没闭环~~ **已打通（见 §10）**：`pcd_to_occupancy_map.py` 把整张 PCD 投影成"按坡度分级可通行"的静态全局占据图 `/terrain_map`，直接当 move_base 全局 costmap 的 static_layer。剩余待完善：目前是**一次性离线投影**（非在线增量更新）、以及与局部 LiDAR costmap 的融合调参。

5. **自动航线 vs 手动点目标**——当前建图既支持预设航点也支持 RViz 点目标，但**没有覆盖式自动建图（coverage planning）**，建图完整性依赖人工点点。

6. **真机 UAV 尚未接入**——目前 UAV 是 PX4 SITL 仿真。架构上预留了切换点（把 `input_sources` 指向真机点云 topic、去掉仿真相机即可），但未验证。

---

## 7. 路线图（下一步）

1. ~~闭环地形感知导航~~ **已实现（见 §10）**：PCD → 坡度分级可通行图 `/terrain_map` → `move_base` 全局 costmap → GPS 定位下跑地形路径。后续：在线增量更新、支持更大场景。
2. **覆盖式自动建图**：给 UAV 加 coverage planner，减少人工点目标。
3. **滚动地图 / 分区扫描**：突破 `map_size` 边界，支持大场景。
4. **评估更强规划器**：若需要低空精细建图，替换 / 增强 EGO 的全局搜索能力。
5. **真机 UAV 接入验证**。

---

## 8. 复现方式（给分析者参考环境）

```bash
# 安装 UAV 仿真环境
bash setup_uav.sh

# 一键起飞建图（自动起飞 + EGO 自主避障飞行）
source devel/setup.bash
bash one_key_takeoff.sh
#   可调参数示例：
#   flight_height:=10            压低飞行高度（不要低于楼高+1m）
#   START_RVIZ=false PX4_GAZEBO_GUI=true   关 RViz / 开 Gazebo GUI

# 飞行中手动存一版点云
rosservice call /uav_pointcloud_map_recorder/save "{}"
#   产物：~/pointcloud_maps/uav_points_map_latest.pcd

# PCD 转高程图
roslaunch mobile_manipulator pcd_to_elevation.launch \
    pcd_file:=~/pointcloud_maps/uav_points_map_latest.pcd
#   输出：/elevation_mapping/elevation_map_postprocessed
```

---

## 9. 给分析者的提问（如果你要帮我看）

- 在"EGO 局部规划器 + 11 m 高楼"的约束下，**有没有比'飞高俯扫'更好的低空精细建图方案**？换规划器值不值？
- **PCD → 高程图 → move_base costmap** 这段闭环，ROS Noetic 生态里最省事的实现路径是什么（`grid_map_costmap_2d` 还是别的）？
- z∈[1,4] m 半墙空洞对后续**空地协同抓取**到底有多大影响？要不要现在就补侧视？
- 大场景下 `map_size` 边界问题，**滚动地图**和**分区扫描**哪个更适合 EGO 这套？
```
```

---

## 10. 地形可通行性建图 + UGV 地形导航闭环（本轮新增，已打通）

> 落地了 §6.4 / §7.1 一直挂着的"PCD → 高程图 → costmap 闭环"。核心节点 **`pcd_to_occupancy_map.py`**：把 UAV 建好的整张 PCD **一次性离线投影**成一张【静态、全局、按坡度分级可通行】的 2D 占据图 `/terrain_map`，直接当 move_base 全局 costmap 的 static_layer。不用 `elevation_mapping`（那是以车为中心的滚动局部图，不适合全局规划）。

### 10.1 坡度可通行性方法（四步）

1. **估地面 DEM**（Digital Elevation Model = 每个地面格子的高度栅格图）：0.25 m 栅格，逐格统计离群剔除 → 取格内**最低点 z** 作地面候选 `dem_min` → 最近邻填洞 → **中值滤波**（窗口 2 m）得 `ground`。
   *为什么用中值而非最小值滤波：* 最小值滤波会把单个低噪点放大成一片"假坑/假坡"，让平地坡度虚高到 20°+ 被误判；楼顶/树冠等高点也"钻不进"地面面。所以得到的是**地面 DEM 而非 DSM（含楼顶表面）**。

2. **算坡度（梯度法）**：`ground_s = uniform_filter(ground)` 再平滑一层抗残余噪声 → `gy,gx = np.gradient(ground_s, 0.25)` → `slope = arctan(hypot(gx, gy))`。即对地面 DEM 求空间梯度、梯度模长取反正切得倾角。

3. **分级可通行性（关键：分级代价，不是二值障碍）**：
   - `坡度 ≤ 12°（slope_free_deg）` → 平地 **free(0)**
   - `12°–30°（slope_max_deg）` → 可爬缓坡，线性给 **1..99 代价**（`cost=(slope-12)/(30-12)*98+1`，越陡代价越大，规划器优先走平地、需要时仍能爬）
   - `> 30°` 或 成片高出本地地面 `obstacle_height` 的点（楼/墙，`high_count ≥ 3`）→ **致命 100**

4. **两路可视化**：`/terrain_slope`（坡度场，线性缩放 0..100）；`/terrain_cloud`（彩色坡面：每格一个点放在真实地面高程上，按坡度 `t=slope/slope_max` 连续着色 **绿→黄→红**，`R=2t,G=2(1-t)`，致命障碍统一深红）。

> **为什么按坡度分级、不按绝对高度、也不二值：** UGV 后续要**上坡**。按绝对高度判会把能爬的山丘整个误判成障碍；二值（slope>阈值=障碍）在本图上把 ~21% 的平/缓地块误标致命、车直接拒绝规划（"灰色方块走不了"）。改分级后实测：**平地 free 67% / 可爬缓坡 26% / 致命 6.5% = 93.5% 可通行**，坡度信息全留在代价梯度里。

### 10.2 接入 move_base

- `terrain_layer.yaml`：StaticLayer 指向 `/terrain_map`；`track_unknown_space:false`（UAV 没扫到的格=可走，靠局部 LiDAR 兜底，否则大片 unknown 让全局规划拒绝通行）；**`trinary_costmap:false`（必须）**——否则 1..99 的坡度分级代价被压成只剩"可走/致命"，缓坡信息全丢、缓坡也会被当障碍。
- **GPS 锚定**：`pcd_to_occupancy_map.py` 开机自动读建图时存的 `~/pointcloud_maps/uav_map_origin.yaml`，把 PCD 平移到公共 datum 帧（49.9, 8.9），UAV 图与 UGV 定位对齐。

### 10.3 同轮一起修的 UGV 导航问题

- **离障碍物更远**：膨胀 `inflation_radius 0.8→1.2`，并设 `cost_scaling_factor`（默认 10）→ **2.5**（代价向外衰减更缓，整段膨胀区都保持较高代价，全局规划主动走中间）。**且修正了命名空间**——原 `costmap_common.yaml` 把 `observation_sources` / 膨胀参数写在 costmap 顶层，插件读不到 → **实时 LiDAR 根本没在标障碍、矮障碍全看不见、直接撞**。改嵌到各 `inflation_layer` / `obstacle_layer` 命名空间下（对齐已验证可用的 `auto_mapping.launch`）。
- **上坡打滑 / 上不去**：4 个驱动轮接触摩擦 `mu1=mu2 1.0→2.0`（在 `husky_ur5.urdf.xacro` 用 `<gazebo reference>` 覆盖，SDF 转换验证生效）。轮关节是无 effort 限制的 continuous → 瓶颈是**打滑**不是扭矩。
- **RViz 里 UGV 被埋在坡面下**：双 EKF 原 `two_d_mode:true` 把 z 锁死成 0。加 Gazebo 真值插件（`libgazebo_ros_p3d.so`）发 `/ground_truth/state`，**global EKF `two_d_mode:false` 只融合其中的 z**（GPS 高度太吵不用），roll/pitch 由 IMU 提供，x/y/yaw 仍由 GPS+轮速主导；local EKF 保持 2D。RViz fixed frame=map，`map→odom` 会带上 z → UGV 贴着 3D 坡面、随坡倾斜，而 2D 导航行为不变（代价图仍是 2D）。

### 10.4 涉及文件

`pcd_to_occupancy_map.py`（投影+坡度可通行）、`ugv_terrain_nav.launch`（参数+节点）、`config/nav/{terrain_layer,costmap_common}.yaml`、`config/ekf_global.yaml`（3D + 真值 z）、`urdf/husky_ur5.urdf.xacro`（轮摩擦 + p3d）、`rviz/terrain_nav.rviz`（Terrain 3D = Flat Squares + RGB8）。

---

## 11. 工程化 / 可移植性 + UGV 上坡与收臂最终修复（2026-06-22）

### 11.1 可移植性（队友 clone 后跑不起来的根因）
- **依赖不在仓库**：`src/ego-planner/` 与 PX4 被 `.gitignore`，由 `setup_uav.sh` 现场 clone/安装。**必须先 `bash setup_uav.sh` 再 `catkin_make`**，否则缺 ego-planner 包必然编译失败。已在 README 置顶写明。
- **去掉写死的绝对路径**：5 个 UAV marker/bridge 脚本的 iris 网格 `/home/lnwuu/PX4-Autopilot/...` → 经 `px4_paths.py`（`PX4_AUTOPILOT_PATH` 或 `~/PX4-Autopilot`）；`mobile_manipulator` 里 `/home/jun/learning_ws/...`（前一开发者）→ 包内相对路径；两个 `.sh` 的 `WS_DIR` → 从脚本位置推算 catkin 根。与队友独立提交的 `px4_paths` 方案合并（保留队友版）。删除指向他人机器的失效软链接 `etc`/`test_data`。

### 11.2 UGV 上坡打滑根因 = 地形没有摩擦
轮子 `mu` 调到再高也打滑下滑——因为 Gazebo 接触摩擦由**两个面共同决定**，而坡所在的 `vrc_heightmap_1` 高程地形**碰撞面根本没定义摩擦**（用默认值，被地面端拖住）。修复：给地形碰撞面显式 `mu=2.0` + `kp/kd`，草地 `mu 0.5→1.5`，轮 `mu 3→4`/`kd 1→10`，DWA `acc_lim_x→1.0`（上坡不空转）。

### 11.3 其它
- **UR5 收成行车姿态**：用 URDF 正运动学求出真正平铺贴车顶、夹爪不入车身的姿态（`shoulder_lift=-2.9, elbow=2.9, wrist_1=-1.2, wrist_2=0`），峰高从 +0.51m 降到 +0.25m，降重心。
- **录图持续性**：放宽 relay 的转弯门槛（`0.8→1.5 rad/s`）与 odom 时效（`0.3→1.0s`），飞行中不再整片丢帧。
- **崩溃修复**：`global_path_planner._shortcut` 在目标落在 ±half 栅格外时 `line_clear` 越界（IndexError/负索引）→ 出界则不抽稀。

### 11.4 样例地形图（队友无需飞图即可跑 UGV）
仓库自带 `mobile_manipulator/maps/uav_terrain_sample.pcd`（33k 点，世界系原点附近）。`ugv_terrain_nav.launch use_sample_map:=true` 即用它（偏移 0、自动对齐），默认仍读你自己的 `~/pointcloud_maps` 最新图（工作流不变）。

---

*生成日期：2026-06-19；§10 于 2026-06-21、§11 于 2026-06-22 增补。当前分支 `main`（已合并 air-ground-terrain-nav）。*
