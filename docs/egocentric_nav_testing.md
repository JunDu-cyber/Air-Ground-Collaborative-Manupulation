# Egocentric 导航栈 测试指南

> 配套 [`egocentric_nav_API.md`](egocentric_nav_API.md)。每步带 **✓ 验证**,过不了就停在该步排查。
> 全程在 Gazebo `outdoor_city` 仿真。

## 环境约定(踩过的坑)
- 工作区用 **`catkin build`**(不是 catkin_make)。
- GPU Velodyne 需要 GL 上下文:`export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority`(本机有 RTX + X)。
- **`rostopic hz`/`echo` 在本机偶发卡死/误报**:验证优先用下面给的 Python `wait_for_message` 片段。
- 杀进程用显式 PID:`pgrep -f X | while read p; do kill -9 $p; done`(`pkill -f` 偶发不可靠)。
- 后台节点用 `nohup ... &` 易被沙箱回收;长驻进程优先用 `roslaunch`/`rosrun`(本指南假设你在真实终端跑)。

---

## STEP 0 — 清场 + 构建

```bash
cd /home/jun/learning_ws
pgrep -f "gzserver|gzclient|rosmaster|dlio_odom" | while read p; do kill -9 $p; done
ss -ltn | grep -q :11311 && echo "PORT BUSY" || echo "port free"
source /opt/ros/noetic/setup.bash
bash setup_nav.sh            # 首次:克隆+打补丁+构建(FAST-LIO 加 BUILD_FASTLIO=1)
source devel/setup.bash
```
**✓ 验证**:`port free`;`catkin build` 全绿;
```bash
for p in direct_lidar_inertial_odometry loam_interface local_planner terrain_cost_adapter robot_body_filter; do
  rospack find $p >/dev/null 2>&1 && echo "OK $p" || echo "MISS $p"; done
rosmsg show mobile_manipulator/WorldTarget | head -1   # -> string class_name
```

---

## STEP 1 — 起 sim + LIO,验感知

```bash
export HUSKY_LMS1XX_ENABLED=1 DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
roslaunch mobile_manipulator egocentric_nav.launch nav:=off odom_source:=dlio gui:=false
```
等约 40~60s。**新终端**验证:
```bash
source devel/setup.bash
# IMU 含重力(z≈9.8):
python3 -c "import rospy;from sensor_msgs.msg import Imu;rospy.init_node('t',anonymous=True);\
m=rospy.wait_for_message('/lidar/imu',Imu,timeout=20);print('accel.z=%.2f'%m.linear_acceleration.z)"
# Velodyne 出点(25k):
python3 -c "import rospy;from sensor_msgs.msg import PointCloud2;rospy.init_node('t',anonymous=True);\
m=rospy.wait_for_message('/velodyne_points',PointCloud2,timeout=20);print('velodyne width=%d'%m.width)"
```
**✓ 验证**:`accel.z≈9.8`;`velodyne width≈25000`。
- 雷达无点 → GPU 插件没拿到 GL:确认 `DISPLAY=:0` 且 URDF 里 `gpu:=true`。
- IMU 不含重力 → 用错了 hector `/imu/data`;DLIO 必须用 `/lidar/imu`。

---

## STEP 2 — TF 树:`odom` 为根、**无 `map`**

```bash
python3 - <<'PY'
import rospy,tf2_ros,yaml
rospy.init_node('tf',anonymous=True); b=tf2_ros.Buffer(); tf2_ros.TransformListener(b)
r=rospy.Rate(2)
for _ in range(8): r.sleep()
F=yaml.safe_load(b.all_frames_as_yaml()) or {}
print("frames=%d  has_map=%s"%(len(F), any('map' in f for f in F)))
print("base_link parent =", F.get('base_link',{}).get('parent'))
print("velodyne  parent =", F.get('velodyne',{}).get('parent'))
PY
```
**✓ 验证**:`has_map=False`;`base_link parent = odom`;`velodyne parent = velodyne_base_link`(单父节点)。
- 出现 `map` → `spawn_outdoor_city.launch` 没传 `egocentric:=true`。
- `velodyne` 有两个父 → DLIO 没用私有标签帧;检查 `dlio_ugv.yaml` 的 `frames/lidar: dlio_lidar`。

---

## STEP 3 — Phase A 导航(目标→自主开过去)

另起规划器(或直接 `nav:=cmu` 重起整栈):
```bash
roslaunch mobile_manipulator egocentric_nav.launch nav:=cmu cost_source:=terrain_analysis maxSpeed:=1.2 gui:=false
```
发一个 odom 系目标,看车开过去(注意是 `/ugv/goal`,**不是** `/move_base_simple/goal`):
```bash
rostopic pub -1 /ugv/goal geometry_msgs/PoseStamped \
  '{header:{frame_id: odom}, pose:{position:{x: 5.0, y: 3.0}, orientation:{w: 1.0}}}'
# 监控位姿逼近目标:
python3 -c "import rospy,math;from nav_msgs.msg import Odometry;rospy.init_node('t',anonymous=True);\
import time;t0=time.time()
while time.time()-t0<40:
 m=rospy.wait_for_message('/state_estimation',Odometry,timeout=5);p=m.pose.pose.position;print('x=%.2f y=%.2f'%(p.x,p.y))"
```
**✓ 验证**:`/way_point` 收到 (5,3);车位姿向 (5,3) 收敛;`/terrain_map` 有点(`PointCloud2`,frame `odom`)。
- 车不动 → 检查 `/cmd_vel` 是否有值、twist_mux 是否被遥控占用。

---

## STEP 4 — Phase B(高程图代价源)

```bash
roslaunch mobile_manipulator egocentric_nav.launch nav:=cmu cost_source:=elevation gui:=false
```
**✓ 验证**:`/elevation_mapping/elevation_map_postprocessed`(GridMap)出图;`/terrain_map` 由
`gridmap_to_terrainmap` 发出;**代价是离地相对值**(平地≈0):
```bash
python3 - <<'PY'
import rospy,numpy as np
from sensor_msgs.msg import PointCloud2; import sensor_msgs.point_cloud2 as pc2
rospy.init_node('c',anonymous=True)
m=rospy.wait_for_message('/terrain_map',PointCloud2,timeout=30)
c=np.array([p[0] for p in pc2.read_points(m,field_names=('intensity',),skip_nans=True)])
print("cost min=%.2f max=%.2f mean=%.2f  >0.15占比=%.0f%%"%(c.min(),c.max(),c.mean(),100*(c>0.15).mean()))
PY
```
**✓ 验证**:`mean` 较小(~0.1-0.2)、不是接近平均绝对高程;否则适配器把绝对高程当了代价(检查 `ground_radius`)。
同时确认 `terrainAnalysis` 节点**没**在跑(单一代价源)。

---

## STEP 5 — 目标点依次导航(核心)

`nav:=cmu` 默认带 `ugv_target_tour`。发 3 个目标,触发,看依次到达:
```bash
P(){ rostopic pub -1 /detected_targets mobile_manipulator/WorldTarget \
  "{class_name: '$1', confidence: 0.9, point:{header:{frame_id: odom}, point:{x: $2, y: $3}}}"; }
P rock 4 0 ; P box 6 3 ; P cone 2 6
rosservice call /ugv/start_tour          # -> success:True, order=[...]
# 看进度:
LOG=$(ls -t ~/.ros/log/latest/ugv_target_tour*.log | head -1); tail -f "$LOG"
```
**✓ 验证**:`order` 是从车当前位姿起的最近邻序列;日志依次出 `reached 1/3`、`2/3`、`3/3`、`tour complete`;
RViz 订 `/target_tour_markers`(odom 系)看到编号球+路线。
- `success:False no targets` → 目标发早了(节点还没起)或被去重;查 `collected target #N` 日志。
- 到不了点一直超时跳过 → `reach_tolerance` 太小 或 目标在障碍上(被地形门控,见 STEP 5b)。

### STEP 5b — 地形门控
把一个目标放到障碍格(高程突变处),触发后 **✓ 验证**:日志出
`N target(s) on untraversable terrain -> deprioritized`,该目标被排到最后。

---

## STEP 6 — 坐标系转移(UAV 系目标 → odom)

模拟 §1g 锚定,发一个 `uav0/map_local` 系目标,验证落到正确 odom 点:
```bash
rosrun tf2_ros static_transform_publisher 0 -18 0 0 0 0 odom uav0/map_local &
# 重起一个干净的 tour 节点(原来的可能已 DONE):
rosrun mobile_manipulator ugv_target_tour.py __name:=ft &
sleep 3
rostopic pub -1 /detected_targets mobile_manipulator/WorldTarget \
  "{class_name: uav_t, confidence: 0.8, point:{header:{frame_id: uav0/map_local}, point:{x: 1, y: 2}}}"
# 读收集到的 odom 坐标(marker 球位置):
python3 -c "import rospy;from visualization_msgs.msg import MarkerArray;rospy.init_node('m',anonymous=True);\
a=rospy.wait_for_message('/target_tour_markers',MarkerArray,timeout=6);\
[print('odom (%.2f, %.2f)'%(k.pose.position.x,k.pose.position.y)) for k in a.markers if k.ns=='targets']"
```
**✓ 验证**:`uav0/map_local (1,2)` + 锚定 (0,-18) → 收集到 **odom (1.00, -16.00)**。

---

## STEP 7 — 里程计漂移 A/B(可选,需 evo)

```bash
pip3 install --user evo
# 起 nav:=cmu odom_source:=dlio,录一段固定轨迹的 rosbag(并行可起裸 fastlio_mapping 做 A/B):
rosbag record -O ab.bag /dlio/odom_node/odom /Odometry /ground_truth/state
# 开车跑个方环(发几个 /ugv/goal 或目标 tour),停录后:
evo_ape bag ab.bag /ground_truth/state /dlio/odom_node/odom -a   # DLIO
evo_ape bag ab.bag /ground_truth/state /Odometry            -a   # FAST-LIO
```
**✓ 验证(参考值)**:DLIO RMSE≈**0.04 m**,FAST-LIO≈**0.46 m**(DLIO 应明显更优)。

---

## STEP 8 — 空地协同 + 锚定(桌面交互)

> 需要 PX4 SITL + GUI,**桌面真终端**跑(无头后台跑不了 PX4 起飞)。

```bash
bash airground_takeoff.sh        # 起 Gazebo+PX4+UAV + UGV(egocentric DLIO + CMU 规划器 + tour)
```
起齐后 **✓ 验证**:
```bash
rosrun tf2_ros tf2_echo odom base_link        # DLIO 在发(无 map)
rosrun tf2_ros tf2_echo odom uav0/map_local   # §1g 锚定锁存后出现
```
- RViz「2D Nav Goal」**只飞无人机**,UGV 不动(已解耦)。✓
- 无人机扫完检完(目标进 `/detected_targets`)后:`rosservice call /ugv/start_tour` → UGV 依次访问目标。
- 锚定校验(独立验证过):锁存的 `odom→uav0/map_local` 与"真值+DLIO 现算"一致到 4 位小数,
  ~0.68m 的 x 偏移来自 odom-世界系 0.036 rad 偏航 × 18m 基线(全位姿锚定能抓到,单点 GPS 抓不到)。

---

## 复测前的清场(每次)

```bash
pgrep -f "gzserver|gzclient|rosmaster|dlio_odom|robot_state_publisher|loamInterface|localPlanner|\
pathFollower|terrainAnalysis|elevation_mapping|ugv_target_tour|twist_mux|controller|spawner|static_transform" \
  | while read p; do kill -9 $p; done
ss -ltn | grep -q :11311 && echo "PORT BUSY(再杀 rosmaster)" || echo "port free"
```

---

## 附录 — 旧版独立工作流测试(UAV PCD 建图 → UGV move_base/`map` 系导航)

> 这条路径与上面 STEP 0-8 是**两套不同的系统**,不是同一栈的另一种跑法:上面测的是当前主线
> egocentric 栈(DLIO、CMU 规划器、无 `map` 系);这里测的是 README「Standalone workflows」里仍保留的
> 旧路径(`one_key_takeoff.sh` → PCD 存图 → `ugv_terrain_nav.launch`,move_base + `map` 系 + AMCL 风格定位)。
> 两条路径都还在用,不要混着排查——本节原是独立的 `RUNBOOK_air_ground_terrain_nav.md`,归并于此仅为减少
> 文档数量,内容未做改写。

> 任务背景:UAV 12m 俯扫把整片区域建图(顺带在开阔区找雷)→ UGV 用这张图定位+地形导航(去抓雷)。
> 每步都有 **✓ 验证**,过不了就停在那步排查,别往下走——这样不会再出现"跑到最后啥也没有"。
> 关键认知:连接拒绝/打不开 = 没有 ROS master 或残留僵尸进程;先过 STEP 0。

### STEP 0 — 清干净 + 确认 master 能起(每次重测都先做)

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f rosmaster
pkill -9 -f roslaunch; pkill -9 -f rosout; pkill -9 -f rviz
pkill -9 -f px4; pkill -9 -f mavros
sleep 3
pgrep -fa "rosmaster|gzserver" || echo "CLEAN"
echo "MASTER=$ROS_MASTER_URI"
ping -c1 $(hostname) >/dev/null 2>&1 && echo "HOST_OK" || echo "HOST_BAD(/etc/hosts 问题)"
```
**✓ 验证**:必须看到 `CLEAN` + `HOST_OK`。
- `MASTER=` 为空 → `export ROS_MASTER_URI=http://localhost:11311`
- `HOST_BAD` → `/etc/hosts` 里加一行 `127.0.0.1  $(hostname)`(连接拒绝的常见根因)

### STEP 1 — Phase A:UAV 12m 俯扫建图

PX4 home 必须设成 Husky 的 GPS 参考(49.9,8.9),GPS 锚定才对得上:

```bash
cd ~/Air-Ground-Collaborative-Manupulation && source devel/setup.bash
export PX4_HOME_LAT=49.9 PX4_HOME_LON=8.9 PX4_HOME_ALT=0
bash one_key_takeoff.sh
```
等约 30~60s 让 Gazebo/PX4/MAVROS 起齐。**新开一个终端**验证:
```bash
source devel/setup.bash
rostopic list | grep -q /clock && echo "SIM_OK(master+gazebo)" || echo "SIM_BAD"
rostopic hz /uav0/mapping/points_world      # 建图点云在累积(Ctrl-C 退出)
```
**✓ 验证**:`SIM_OK` + `points_world` 有频率。
- `SIM_BAD` / 连接拒绝 → 回 STEP 0 再清一次;看 `1_Gazebo` 终端最上面的红字(gazebo 真正崩的原因在最上面)。

无人机自动起飞后,在它的 RViz 里**多点几个目标点,把整片城区都飞一遍**(俯扫要覆盖全)。扫够了存图:
```bash
rosservice call /uav_pointcloud_map_recorder/save "{}"
ls -la ~/pointcloud_maps/uav_points_map_latest.pcd
ls -la ~/pointcloud_maps/uav_map_origin.yaml      # GPS 锚定原点(自动存的)
```
**✓ 验证**:两个文件都存在。`uav_map_origin.yaml` 是 GPS 自动对齐用的,没有它对齐会退回 offset 0。

存完 **Ctrl-C 关掉 Phase A**,再做一次 STEP 0 清理。

### STEP 2 — 清洗 PCD(去重叠 + 去杂散噪点)

```bash
cd ~/Air-Ground-Collaborative-Manupulation
python3 src/uav_truth_tracker/scripts/clean_pcd.py ~/pointcloud_maps/uav_points_map_latest.pcd
```
**✓ 验证**:打印 `NNNN -> MMMM points`,并生成 `~/pointcloud_maps/uav_points_map_latest_clean.pcd`。

### STEP 3 — Phase B:UGV 世界 + 定位(终端1)

```bash
source devel/setup.bash
roslaunch mobile_manipulator spawn_outdoor_city.launch
```
等约 20s。**新终端**验证:
```bash
source devel/setup.bash
rostopic hz navsat/fix                         # UGV GPS 在发(~40Hz)
rostopic echo -n1 /tf 2>/dev/null | grep -E "odom|base_link" | head   # map->odom->base_link 存在
```
**✓ 验证**:`navsat/fix` 有频率 + TF 里能看到 odom/base_link。
- 没有 → spawn_outdoor_city 没起全,看它终端的报错。

### STEP 4 — Phase B:地图 + move_base + RViz(终端2,复用终端1定位)

```bash
source devel/setup.bash
roslaunch mobile_manipulator ugv_terrain_nav.launch localization:=false \
    pcd_file:=~/pointcloud_maps/uav_points_map_latest_clean.pcd
```
**新终端**验证(这步是你之前卡的地方):
```bash
source devel/setup.bash
rosnode list | grep pcd_to_occupancy_map && echo "NODE_ALIVE"
rostopic hz /terrain_cloud                     # 三维高程图在发(Ctrl-C 退出)
rostopic echo -n1 /terrain_map/info | grep -E "width|height"   # 地图非空
rostopic hz /move_base/global_costmap/costmap  # 全局 costmap 起来了
```
**✓ 验证**:`NODE_ALIVE` + `/terrain_cloud` 有频率 + width/height 非 0。
- 节点没在列表里 / 没频率 → 看**终端2**里 `[pcd_to_occupancy_map]` 那行:
  - `PCD not found` → pcd_file 路径错(`ls ~/pointcloud_maps/*.pcd` 对一下真实名字)
  - 一段红 Traceback → 发我
- 自动 GPS 锚定生效时,终端2 会打印 `自动GPS锚定(sidecar ...) offset=(x,y)`。

**RViz 看图**(这个 launch 自带的 RViz):
- 三维高程图 = 左边 **`Terrain 3D (/terrain_cloud)`**(绿=可走/红=障碍,高度=真实地面)
- 顶视图 = **`UAV Map (/terrain_map)`**
- **Global Options → Fixed Frame 必须 = `map`**(否则 map 系的图不显示)

### STEP 5 — 导航

RViz 顶栏点 **2D Nav Goal** → 在图上点目标 → **Husky 出发**,绿色 `Global Plan` 绕开楼/陡坡。
```bash
# 命令行发目标也行:
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
rostopic hz /cmd_vel       # 有速度=车在走
```
**✓ 验证**:`/cmd_vel` 有输出 + 车动。
- 点远目标不动 → 超出地图边界,调大 `free_border:=40` 重起终端2。
- 报 `Could not transform global plan` → ekf TF 滞后,确认终端1 在跑(已加 transform_time_offset 兜底)。

### 一页速查(顺序,每步先 source devel/setup.bash)

```bash
# STEP0 清理(每次重测都做)
pkill -9 -f 'gzserver|gzclient|rosmaster|roslaunch|rviz|px4|mavros'; sleep 3

# STEP1 建图(终端A)
export PX4_HOME_LAT=49.9 PX4_HOME_LON=8.9 PX4_HOME_ALT=0
bash one_key_takeoff.sh                      # 飞遍城区
rosservice call /uav_pointcloud_map_recorder/save "{}"     # 存图;Ctrl-C 退出

# STEP2 清洗
python3 src/uav_truth_tracker/scripts/clean_pcd.py ~/pointcloud_maps/uav_points_map_latest.pcd

# STEP3 UGV世界(终端B)
roslaunch mobile_manipulator spawn_outdoor_city.launch

# STEP4 导航栈(终端C)
roslaunch mobile_manipulator ugv_terrain_nav.launch localization:=false \
    pcd_file:=~/pointcloud_maps/uav_points_map_latest_clean.pcd
# RViz: 看 "Terrain 3D" / "UAV Map", Fixed Frame=map, 2D Nav Goal 点目标
```

### 故障速查(旧版工作流)

| 现象 | 原因 | 解 |
|---|---|---|
| `Connection refused` / 打不开 | 没有 master / 僵尸进程 | STEP 0 清理 + 起 roscore 确认;查 `ROS_MASTER_URI`、`/etc/hosts` |
| RViz 没高程图 | 看了空显示项 / 节点没起 / Fixed Frame 错 | 看 `Terrain 3D`(不是别的);STEP4 验证节点;Fixed Frame=map |
| `PCD not found` | pcd_file 路径错 | `ls ~/pointcloud_maps/*.pcd` 对名字 |
| 地图大半空白 | free_border 太大 / 单次飞覆盖窄 | 12m 俯扫飞全;free_border 默认已降到 15 |
| 点远目标不动 | 超出地图 | `free_border:=40` |
| `MISALIGNED`/对不上 | PX4_HOME 没设成 49.9/8.9 | STEP1 先 export 再 one_key_takeoff |

> 找雷说明:航拍只在**开阔地**能看见地面的雷;**密林树下的雷航拍看不到**,那片要 UGV 自己带探雷器进去排查。
