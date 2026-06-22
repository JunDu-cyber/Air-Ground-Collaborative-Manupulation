# 完整测试过程：UAV 建图(找雷) → UGV 地形导航

> 任务背景：UAV 12m 俯扫把整片区域建图(顺带在开阔区找雷)→ UGV 用这张图定位+地形导航(去抓雷)。
> 每步都有 **✓ 验证**，过不了就停在那步排查，别往下走——这样不会再出现"跑到最后啥也没有"。
> 关键认知：连接拒绝/打不开 = 没有 ROS master 或残留僵尸进程；先过 STEP 0。

---

## STEP 0 — 清干净 + 确认 master 能起(每次重测都先做)

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f rosmaster
pkill -9 -f roslaunch; pkill -9 -f rosout; pkill -9 -f rviz
pkill -9 -f px4; pkill -9 -f mavros
sleep 3
pgrep -fa "rosmaster|gzserver" || echo "CLEAN"
echo "MASTER=$ROS_MASTER_URI"
ping -c1 $(hostname) >/dev/null 2>&1 && echo "HOST_OK" || echo "HOST_BAD(/etc/hosts 问题)"
```
**✓ 验证**：必须看到 `CLEAN` + `HOST_OK`。
- `MASTER=` 为空 → `export ROS_MASTER_URI=http://localhost:11311`
- `HOST_BAD` → `/etc/hosts` 里加一行 `127.0.0.1  $(hostname)`（连接拒绝的常见根因）

---

## STEP 1 — Phase A：UAV 12m 俯扫建图

PX4 home 必须设成 Husky 的 GPS 参考(49.9,8.9)，GPS 锚定才对得上：

```bash
cd ~/Air-Ground-Collaborative-Manupulation && source devel/setup.bash
export PX4_HOME_LAT=49.9 PX4_HOME_LON=8.9 PX4_HOME_ALT=0
bash one_key_takeoff.sh
```
等约 30~60s 让 Gazebo/PX4/MAVROS 起齐。**新开一个终端**验证：
```bash
source devel/setup.bash
rostopic list | grep -q /clock && echo "SIM_OK(master+gazebo)" || echo "SIM_BAD"
rostopic hz /uav0/mapping/points_world      # 建图点云在累积(Ctrl-C 退出)
```
**✓ 验证**：`SIM_OK` + `points_world` 有频率。
- `SIM_BAD` / 连接拒绝 → 回 STEP 0 再清一次；看 `1_Gazebo` 终端最上面的红字(gazebo 真正崩的原因在最上面)。

无人机自动起飞后，在它的 RViz 里**多点几个目标点，把整片城区都飞一遍**(俯扫要覆盖全)。扫够了存图：
```bash
rosservice call /uav_pointcloud_map_recorder/save "{}"
ls -la ~/pointcloud_maps/uav_points_map_latest.pcd
ls -la ~/pointcloud_maps/uav_map_origin.yaml      # GPS 锚定原点(自动存的)
```
**✓ 验证**：两个文件都存在。`uav_map_origin.yaml` 是 GPS 自动对齐用的，没有它对齐会退回 offset 0。

存完 **Ctrl-C 关掉 Phase A**，再做一次 STEP 0 清理。

---

## STEP 2 — 清洗 PCD(去重叠 + 去杂散噪点)

```bash
cd ~/Air-Ground-Collaborative-Manupulation
python3 src/uav_truth_tracker/scripts/clean_pcd.py ~/pointcloud_maps/uav_points_map_latest.pcd
```
**✓ 验证**：打印 `NNNN -> MMMM points`，并生成 `~/pointcloud_maps/uav_points_map_latest_clean.pcd`。

---

## STEP 3 — Phase B：UGV 世界 + 定位（终端1）

```bash
source devel/setup.bash
roslaunch mobile_manipulator spawn_outdoor_city.launch
```
等约 20s。**新终端**验证：
```bash
source devel/setup.bash
rostopic hz navsat/fix                         # UGV GPS 在发(~40Hz)
rostopic echo -n1 /tf 2>/dev/null | grep -E "odom|base_link" | head   # map->odom->base_link 存在
```
**✓ 验证**：`navsat/fix` 有频率 + TF 里能看到 odom/base_link。
- 没有 → spawn_outdoor_city 没起全，看它终端的报错。

---

## STEP 4 — Phase B：地图 + move_base + RViz（终端2，复用终端1定位）

```bash
source devel/setup.bash
roslaunch mobile_manipulator ugv_terrain_nav.launch localization:=false \
    pcd_file:=~/pointcloud_maps/uav_points_map_latest_clean.pcd
```
**新终端**验证（这步是你之前卡的地方）：
```bash
source devel/setup.bash
rosnode list | grep pcd_to_occupancy_map && echo "NODE_ALIVE"
rostopic hz /terrain_cloud                     # 三维高程图在发(Ctrl-C 退出)
rostopic echo -n1 /terrain_map/info | grep -E "width|height"   # 地图非空
rostopic hz /move_base/global_costmap/costmap  # 全局 costmap 起来了
```
**✓ 验证**：`NODE_ALIVE` + `/terrain_cloud` 有频率 + width/height 非 0。
- 节点没在列表里 / 没频率 → 看**终端2**里 `[pcd_to_occupancy_map]` 那行：
  - `PCD not found` → pcd_file 路径错(`ls ~/pointcloud_maps/*.pcd` 对一下真实名字)
  - 一段红 Traceback → 发我
- 自动 GPS 锚定生效时，终端2 会打印 `自动GPS锚定(sidecar ...) offset=(x,y)`。

**RViz 看图**（这个 launch 自带的 RViz）：
- 三维高程图 = 左边 **`Terrain 3D (/terrain_cloud)`**(绿=可走/红=障碍，高度=真实地面)
- 顶视图 = **`UAV Map (/terrain_map)`**
- **Global Options → Fixed Frame 必须 = `map`**（否则 map 系的图不显示）

---

## STEP 5 — 导航

RViz 顶栏点 **2D Nav Goal** → 在图上点目标 → **Husky 出发**，绿色 `Global Plan` 绕开楼/陡坡。
```bash
# 命令行发目标也行：
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
rostopic hz /cmd_vel       # 有速度=车在走
```
**✓ 验证**：`/cmd_vel` 有输出 + 车动。
- 点远目标不动 → 超出地图边界，调大 `free_border:=40` 重起终端2。
- 报 `Could not transform global plan` → ekf TF 滞后，确认终端1 在跑(已加 transform_time_offset 兜底)。

---

## 一页速查（顺序，每步先 source devel/setup.bash）

```bash
# STEP0 清理(每次重测都做)
pkill -9 -f 'gzserver|gzclient|rosmaster|roslaunch|rviz|px4|mavros'; sleep 3

# STEP1 建图(终端A)
export PX4_HOME_LAT=49.9 PX4_HOME_LON=8.9 PX4_HOME_ALT=0
bash one_key_takeoff.sh                      # 飞遍城区
rosservice call /uav_pointcloud_map_recorder/save "{}"     # 存图；Ctrl-C 退出

# STEP2 清洗
python3 src/uav_truth_tracker/scripts/clean_pcd.py ~/pointcloud_maps/uav_points_map_latest.pcd

# STEP3 UGV世界(终端B)
roslaunch mobile_manipulator spawn_outdoor_city.launch

# STEP4 导航栈(终端C)
roslaunch mobile_manipulator ugv_terrain_nav.launch localization:=false \
    pcd_file:=~/pointcloud_maps/uav_points_map_latest_clean.pcd
# RViz: 看 "Terrain 3D" / "UAV Map", Fixed Frame=map, 2D Nav Goal 点目标
```

---

## 故障速查

| 现象 | 原因 | 解 |
|---|---|---|
| `Connection refused` / 打不开 | 没有 master / 僵尸进程 | STEP 0 清理 + 起 roscore 确认；查 `ROS_MASTER_URI`、`/etc/hosts` |
| RViz 没高程图 | 看了空显示项 / 节点没起 / Fixed Frame 错 | 看 `Terrain 3D`(不是别的)；STEP4 验证节点;Fixed Frame=map |
| `PCD not found` | pcd_file 路径错 | `ls ~/pointcloud_maps/*.pcd` 对名字 |
| 地图大半空白 | free_border 太大 / 单次飞覆盖窄 | 12m 俯扫飞全；free_border 默认已降到 15 |
| 点远目标不动 | 超出地图 | `free_border:=40` |
| `MISALIGNED`/对不上 | PX4_HOME 没设成 49.9/8.9 | STEP1 先 export 再 one_key_takeoff |

> 找雷说明：航拍只在**开阔地**能看见地面的雷；**密林树下的雷航拍看不到**，那片要 UGV 自己带探雷器进去排查。详见对话。
