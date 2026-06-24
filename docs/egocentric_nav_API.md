# Egocentric 导航栈 API 文档

> Husky+UR5 UGV 的 **egocentric**(以 `odom` 为唯一权威系、无 `map`/GPS)导航栈:
> 3D LiDAR + DLIO 里程计 + CMU local_planner + 高程图代价源 + 空地锚定 + 无人机目标点依次导航。
> 本文是接口契约(话题/服务/消息/坐标系/参数/launch);测试步骤见
> [`egocentric_nav_testing.md`](egocentric_nav_testing.md)。
> 分支 `feature/egocentric-nav`。一键复现依赖:`bash setup_nav.sh`(FAST-LIO 加 `BUILD_FASTLIO=1`)。

---

## 1. 架构总览

```
/velodyne_points ─[robot_body_filter 自滤波]─> /velodyne_points_filtered
        + /lidar/imu(含重力)                         │
                                          ┌──────────┴──────────┐
                                          │  DLIO (主) / FAST-LIO │  ← odom_source
                                          └──────────┬──────────┘
                          odom→base_link(唯一发布者) │ /dlio/odom_node/odom + deskewed
                                          ┌──────────┴──────────┐
                                          │     loam_interface   │
                                          └──────────┬──────────┘
                          /state_estimation(odom)    │   /registered_scan(odom)
                ┌──────────────────────┬─────────────┴───────────────┐
        terrain_analysis(A)     elevation_mapping+adapter(B)   →  /terrain_map(odom, XYZI=代价)
                                          │
   /ugv/goal ─[goal_to_waypoint]─> /way_point ─> localPlanner ─> pathFollower
                  ↑                                                     │ /cmd_vel_stamped
   ugv_target_tour ←[/detected_targets, /ugv/start_tour]      [twiststamped_to_twist]
                                                                        │ /cmd_vel
                                                                   twist_mux → Husky
```

**核心不变量**:`odom` 是唯一根系,**TF 树里没有 `map` 帧**;`odom→base_link` 只有 DLIO 一个发布者;
下游对里程计来源无感(`odom_source:=dlio|fastlio` 切换不动下游)。

---

## 2. Launch 接口

### `egocentric_nav.launch`(顶层,单机)
| arg | 默认 | 说明 |
|---|---|---|
| `nav` | `cmu` | `off`(仅 sim+LIO) / `cmu`(带规划器) |
| `odom_source` | `dlio` | `dlio`(主) / `fastlio`(A/B 基线) |
| `cost_source` | `terrain_analysis` | `terrain_analysis`(A,自建) / `elevation`(B,高程图) |
| `self_filter` | `true` | 是否自滤波机械臂 |
| `maxSpeed` | `1.5` | 底盘上限 m/s |
| `start_gazebo` / `gui` | `true` | 起 Gazebo / GUI |
| `rviz` | `false` | 起 RViz(需 `rviz/egocentric_nav.rviz`) |

> GPU Velodyne 需要 GL 上下文:无头跑要 `export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority`。

```bash
DISPLAY=:0 roslaunch mobile_manipulator egocentric_nav.launch \
    nav:=cmu odom_source:=dlio cost_source:=terrain_analysis
```

### `lidar_odometry.launch`(LIO 前端)
`odom_source:=dlio|fastlio` · `self_filter:=true|false` · `raw_cloud_topic:=/velodyne_points` ·
`imu_topic:=/lidar/imu`。产出 `/state_estimation` + `/registered_scan`,DLIO 发 `odom→base_link`。

### `cmu_planner.launch`(规划器 + 目标管线)
| arg | 默认 | 说明 |
|---|---|---|
| `cost_source` | `terrain_analysis` | 代价源 A/B |
| `maxSpeed` | `1.5` | |
| `vehicleLength`/`vehicleWidth` | `0.99`/`0.67` | Husky 尺寸 |
| `sensorOffsetX`/`Y` | `0`/`0` | DLIO 已出 base_link 位姿 |
| `ground_radius` / `vehicle_height` | `0.75`/`1.0` | Phase B 适配器(局部地面窗口/代价上限) |
| `ugv_goal_topic` | `/ugv/goal` | UGV 目标口(**不是** `/move_base_simple/goal`) |
| `target_tour` | `true` | 是否起目标依次导航节点 |

### `spawn_outdoor_city.launch`(机器人)
`egocentric:=true` 去掉 GPS/双 EKF/navsat(DLIO 接管 `odom→base_link`,无 `map`)。
`egocentric:=false`(默认)保留旧 DWA 链。`start_gazebo` / `spawn_unpause` 供协同仿真复用。

### `airground_takeoff.sh`(空地协同,桌面 gnome-terminal)
env:`UGV_NAV=true`(起 UGV 规划器+目标导航)、`START_RVIZ`、`SPAWN_X/Y/Z`、`FLIGHT_H` 等。
起完后 UGV **静止**,直到 `rosservice call /ugv/start_tour`。

---

## 3. 话题契约

### 感知 / 里程计
| 话题 | 类型 | 帧 | 说明 |
|---|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | `velodyne` | 原始 VLP-16(GPU 插件,惰性发布) |
| `/velodyne_points_filtered` | `PointCloud2` | `velodyne` | 自滤波(去机械臂),喂 LIO |
| `/lidar/imu` | `sensor_msgs/Imu` | `imu_lidar_link` | **含重力**(DLIO 必需) |
| `/dlio/odom_node/odom` | `nav_msgs/Odometry` | `odom`/`base_link` | DLIO 原始里程计 |
| `/state_estimation` | `nav_msgs/Odometry` | `odom`/`base_link` | loam_interface 统一输出 |
| `/registered_scan` | `PointCloud2` | `odom` | 配准点云(自滤波) |
| `/ground_truth/state` | `nav_msgs/Odometry` | `map` | Gazebo 真值,**仅**锚定/离线评估用 |

### 导航 / 控制
| 话题 | 类型 | 帧 | 说明 |
|---|---|---|---|
| `/terrain_map` | `PointCloud2` (XYZI) | `odom` | 地形代价,`intensity=离地高度` |
| `/way_point` | `geometry_msgs/PointStamped` | `odom` | 规划器目标(localPlanner 读 x,y) |
| `/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | `vehicle` | pathFollower 输出 |
| `/cmd_vel` | `geometry_msgs/Twist` | — | twist_mux 优先级 1 输入 |
| `/joy_teleop/cmd_vel` | `Twist` | — | 遥控(优先级 10,可抢占规划器) |

### 目标点接口(本次新增)
| 话题/服务 | 类型 | 帧 | 说明 |
|---|---|---|---|
| `/detected_targets` | `mobile_manipulator/WorldTarget` | 任意(header) | **预留**:无人机检测器一次发一个 |
| `/ugv/goal` | `geometry_msgs/PoseStamped` | 任意(header→odom) | UGV 唯一目标口(手动/序列器) |
| `/ugv/start_tour` | `std_srvs/Trigger` (srv) | — | 冻结目标集并开始依次导航 |
| `/target_tour_markers` | `visualization_msgs/MarkerArray` | `odom` | RViz:目标点 + 编号顺序 + 路线 |

> ⚠️ `/move_base_simple/goal` 是**无人机 EGO-planner** 的目标口(`goal_elevator.py` 让无人机飞过去)。
> UGV 一律走 `/ugv/goal`,**不要**共用 `/move_base_simple/goal`。

---

## 4. 消息:`mobile_manipulator/WorldTarget`

```
string                     class_name    # 视觉类别(导航不用,留给上层)
int32                      class_id
float32                    confidence     # 同位置多次检出时取最高
geometry_msgs/PointStamped point          # 仅位置;frame 在 point.header.frame_id
```
**契约**:无人机相机检测器每检出一个目标发一条(流式)。`point.header.frame_id` 决定坐标系
(相机系/`uav0/map_local`/`odom` 均可),`ugv_target_tour` 会按它 TF 到 `odom`。

---

## 5. 坐标系(TF 树)

```
odom ─(DLIO)→ base_link ─(URDF)→ {top_plate, velodyne_mast→velodyne, imu_lidar_link, ur5_*, ...}
  │                          └─(static)→ vehicle(=base_link, 给规划器 RViz 用)
  ├─(DLIO 私有标签)→ dlio_lidar / dlio_imu
  └─(§1g 锚定,latch)→ uav0/map_local   ← 空地系才有
```
- **没有 `map`**。`view_frames` 根必须是 `odom`。
- 外参(URDF 实测):`base_link→velodyne = [-0.4188, 0, 0.6827]`(后置桅杆),`base_link→imu_lidar_link = I`。
- `dlio_lidar`/`dlio_imu` 是 DLIO 私有标签帧,避免和 robot_state_publisher 的 `velodyne` 撞父节点。
- `odom→uav0/map_local` 由 `airground_anchor_latch.py` 一次性捕获后锁存(全位姿,偏航可观测)。

---

## 6. 关键参数文件

| 文件 | 作用 | 关键点 |
|---|---|---|
| `config/dlio_ugv.yaml` | DLIO 覆盖 | frames=odom/base_link;extrinsics;**`deskew:false`**(Gazebo time 字段全 0)、**`map/waitUntilMove:false`**(否则不动不发点云) |
| `config/ugv_velodyne.yaml` | FAST-LIO | `extrinsic_T:[-0.4188,0,0.6827]` |
| `config/robot_body_filter.yaml` | 自滤波 | 裁剪 UR5/夹爪链;`frames/fixed=base_link`(无 odom 依赖) |
| `config/elevation_mapping_ugv.yaml` | Phase B 高程图 | 吃 `/velodyne_points_filtered`;`robot_pose_with_covariance_topic:""`→纯 TF |
| `cmu_planner.launch` 内联 | local_planner | `goalClearRange=0.5`(到点容差)、`obstacleHeightThre=0.15`、`twoWayDrive=true` |

### `ugv_target_tour` 参数
| 参数 | 默认 | 说明 |
|---|---|---|
| `~detected_topic` | `/detected_targets` | 检测输入 |
| `~goal_topic` | `/ugv/goal` | 输出目标 |
| `~odom_topic` | `/state_estimation` | UGV 位姿(到点判定/排序起点) |
| `~terrain_topic` | `/terrain_map` | 地形门控 |
| `~dedup_radius` | `0.5` m | 同位置去重半径 |
| `~reach_tolerance` | `0.6` m | 小于即"到达"并推进下一个 |
| `~goal_timeout` | `60.0` s | 单目标超时则跳过 |
| `~obstacle_cost_thre` | `0.3` | 地形代价超此则该目标降权到最后 |

---

## 7. 扩展点:接入真实无人机检测器

将来无人机相机检测器只需:
1. 检出一个目标 → 发一条 `mobile_manipulator/WorldTarget` 到 `/detected_targets`,
   `point.header.frame_id` 填检测所在系(相机系或 `uav0/map_local`)。
2. 扫描+检测完成 → 调 `/ugv/start_tour`(`std_srvs/Trigger`)。

其余(TF 到 odom、去重、排序、依次 plan)由 `ugv_target_tour` 完成,**无需改动下游**。
排序策略(当前:最近邻+地形门控)如需换 TSP/优先级,只改 `ugv_target_tour._compute_order()`。

---

## 8. 里程计 A/B(STEP 2 结论)

同一条 97s/~32m 闭环,`evo_ape` vs `/ground_truth/state`(SE3 对齐):

| 前端 | RMSE | mean | max |
|---|---|---|---|
| **DLIO**(主) | **3.77 cm** | 2.85 cm | 15.1 cm |
| FAST-LIO(基线) | 46.0 cm | 37.7 cm | 95.1 cm |

DLIO 默认主前端。FAST-LIO 仅作对比基线(`odom_source:=fastlio`,需 `BUILD_FASTLIO=1` 构建)。
