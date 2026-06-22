# 方案①：以 GPS/UTM 为公共参考，对齐 UAV 建图坐标系与 UGV 导航坐标系

> 目标：让 UAV（PX4 + MAVROS）建出的高程图与 UGV（Husky，双 EKF + navsat_transform）的导航坐标系**锚定到同一个 `map` 原点**，使 UGV 能在 UAV 建好的高程图上正确定位、导航。
>
> 适用仓库：`Air-Ground-Collaborative-Manupulation`（ROS Noetic / Gazebo 11）。本文给出的配置与仓库现有 `navsat_transform.yaml` / `ekf_global.yaml` / `pcd_to_elevation.launch` 一致，可直接落地。
>
> 约束遵守：不引入 RTAB-Map / Cartographer；不修改 `pcd_to_elevation.launch` 内部实现（仅 `include`）；UTM 换算**全部离线自实现**，不依赖网络或外部地图服务；`robot_localization` 用 Noetic 自带版本。

---

## 1. 分析与方案说明

### 1.1 问题本质

- **UAV** 的建图 `map` 原点 = PX4 上电解锁（arm）时的 GPS 位置（PX4 EKF local origin）。
- **UGV** 的 `map` 原点 = `navsat_transform_node` 初始化时取的那一帧 GPS（datum）。
- 两个原点是**地球上不同的点**，因此两套 `map` 之间存在一个**未知的水平刚体偏移**（纯平移 + 可能的航向差）。高程图建在 UAV 的 map 里，UGV 在自己的 map 里定位，二者对不上 → UGV 在高程图上"错位"。

### 1.2 为什么 UTM 能作为公共参考

UTM（Universal Transverse Mercator）把经纬度投影到一个**米制的平面直角坐标系**（Easting/Northing，单位米）。它的关键性质：

1. **全局唯一**：地球上任意一点（在同一 UTM zone 内）对应唯一的 `(E, N)`，与"谁先开机、在哪开机"无关。
2. **米制 + 局部近似笛卡尔**：在几百米~几公里的作业范围内，UTM 平面可近似为笛卡尔平面，距离/角度畸变可忽略（k0=0.9996 的尺度差在中央经线附近 < 0.04%）。
3. **轴向固定**：UTM 的 E 轴指向**地理东**、N 轴指向**地理北**。

因此：只要**让 UAV 的 map 原点和 UGV 的 map 原点对应到同一个 UTM 点**，两套 map 就重合。这就是"原点统一"。

### 1.3 对齐的数学原理

设公共参考点（datum）的 UTM 坐标为 `O = (E0, N0)`，航向参考为地理北。

- **UGV**：`navsat_transform_node` 用 datum `O` 时，它定义 `map(0,0,0) ≡ UTM(E0,N0)，朝向正北`。UGV 在 map 里的位姿 = `(E_ugv − E0, N_ugv − N0)`（再叠加 EKF 的航向）。
- **UAV**：PX4 local ENU 原点在 arm 点 `UTM(E_uav0, N_uav0)`。把它放进公共 map，只需一个**纯平移**：

```
T(map ← uav_local) = ( E_uav0 − E0 ,  N_uav0 − N0 ,  0 )
```

**为什么是纯平移、没有旋转**：PX4 经 MAVROS 输出的 `local_position` 是 **ENU**（x=东, y=北, z=上），UTM 也是 **东/北/上**。两者轴向都对齐到"地理东/北"，所以从 PX4-local 到 UTM-local 只差一个平移，**没有旋转**（前提：UAV 的 PX4 EKF 航向已对齐真北——仿真里默认成立，真机靠磁力计/双天线，见 §1.5）。

于是 UAV 建图点云里的任意一点 `p_local`，在公共 map 中的坐标：

```
p_map = p_local + ( E_uav0 − E0 ,  N_uav0 − N0 ,  0 )
```

这就是方案①的全部数学：**两套局部 ENU，靠 UTM 把各自原点的平移差求出来，统一到同一个 map 原点**。

### 1.4 datum（基准点）的作用

`datum = [lat0, lon0, yaw0]` 是 `navsat_transform_node` 的核心参数，它声明：

> "`map` 坐标系的原点 (0,0,0)，对应地球上的 `(lat0, lon0)`，且 map 的 +x 轴相对正东偏转 `yaw0`。"

- `wait_for_datum: false`（仓库现状）→ 节点**取第一帧有效 GPS 作为 datum**，每台机器各取各的 → 原点不一致（这正是当前 bug 的根源）。
- `wait_for_datum: true` + 显式 `datum: [...]` → 节点用**你指定的固定 datum**。**只要 UAV 侧和 UGV 侧填同一个 datum（或至少同一个 UTM zone + 同一参考点），两套 map 原点就重合**。

> 这是方案①最省事的落地点：**给两侧 navsat_transform 配同一个 datum**。

### 1.5 仿真 vs 真机的处理方式

| | 仿真（Gazebo + PX4 SITL + hector GPS） | 真机 |
|---|---|---|
| **共同世界** | UAV 和 UGV 在**同一个 Gazebo 世界**，本就共享世界原点；GPS 只是把世界坐标映射成经纬度 | 无共享世界，只能靠 GPS/UTM |
| **对齐手段** | 令三者一致即可：`PX4_HOME_LAT/LON` ＝ hector GPS `referenceLatitude/Longitude` ＝ navsat `datum` lat/lon。此时 UAV map = UGV map = Gazebo world | 选定一个固定 datum（勘测点或 UGV 首帧 fix），**两侧 navsat_transform 都用它**；UAV 侧用 `/mavros/global_position/global` |
| **航向** | hector IMU 已给世界系 yaw，`magnetic_declination_radians: 0`、`yaw_offset: 0` | 必须设真实**磁偏角**；UAV 航向需磁力计/双天线 GNSS 保证对齐真北 |
| **高度** | `zero_altitude: true`（地形平、GPS 高程噪声大） | 一般仍 `zero_altitude: true`，z 用高程图/气压计 |
| **偏移来源** | 设好引用点后 ≈ 0 | `Δ = UTM(uav_home) − UTM(datum)` 实测求得 |

### 1.6 两种落地机制（按部署形态二选一）

- **机制 A — datum 统一（推荐，仿真 / 单 ROS master）**：UAV、UGV 两个 `navsat_transform_node` 配**同一个 datum**。两套 map 物理重合，UAV 直接把点云录在公共 `map` 里，UGV 直接用。**最干净。**
- **机制 B — 静态 UTM 偏移（真机 / UAV 与 UGV 不同机器、不同 ROS master）**：离线算 `Δ = UTM(uav_home) − UTM(datum)`，发布 `map → uav_local` 静态 TF（或把已存的 PCD 整体平移 `Δ`）。当两机无法共享 TF/话题时用它。换算用 §4 节点里同一套 `ll_to_utm`。

下面 §2/§3 给机制 A 的配置；§2.3 给机制 B 的离线偏移做法。

---

## 2. UAV 侧配置

UAV 侧要做的是：**让 UAV 建图点云所在的 `map` 与公共 datum 对齐**。

### 2.1 MAVROS：确保 global_position 正常出图

`one_key_takeoff.sh` 已启动 MAVROS。确认 `global_position` 插件在跑、话题有数据即可（无需改 MAVROS 本身）：

```bash
# UAV GPS（lat/lon/alt）——后面 navsat 和校验节点都要用
rostopic echo -n1 /mavros/global_position/global    # sensor_msgs/NavSatFix
# UAV 本地 ENU 里程计（x=东,y=北,z=上，原点=arm 点）
rostopic echo -n1 /mavros/local_position/odom        # nav_msgs/Odometry
```

> ⚠️ **真机注意**：起飞前等 `/mavros/global_position/global` 的 `status.status >= 0`（有定位）、且 EKF 航向收敛（看 `/mavros/local_position/pose` 的 yaw 稳定），再 arm，否则 PX4 local 原点漂移会污染 `Δ`。

### 2.2 机制 A：UAV 侧 navsat_transform（与 UGV 同 datum）

新增参数文件 `config/navsat_transform_uav.yaml`，**datum 必须与 UGV 完全相同**：

```yaml
# config/navsat_transform_uav.yaml
# UAV 侧 navsat_transform：把 /mavros 的 GPS + 本地 ENU 里程计，锚定到与 UGV
# 相同的 datum，使 UAV 的 map 与 UGV 的 map 重合。
frequency: 30.0
transform_timeout: 0.1
delay: 3.0                       # 等 PX4 EKF 航向稳定

magnetic_declination_radians: 0.0   # 仿真=0；真机填作业地磁偏角
yaw_offset: 0.0                      # MAVROS local 已是 ENU，无需补偿
zero_altitude: true

broadcast_cartesian_transform: true # 发布 utm->map 静态 TF（与 UGV 一致）
publish_filtered_gps: true
use_odometry_yaw: true              # UAV 无独立 EKF，直接用 MAVROS 的航向

# === 关键：与 UGV 用同一个 datum（同一 UTM 锚点）===
wait_for_datum: true
datum: [47.3977419, 8.5455938, 0.0]   # [lat, lon, yaw]；占位值，按作业点改，UAV/UGV 必须一致
```

UAV 侧 launch 片段（可并入 `one_key_takeoff.sh` 调用的建图 launch，或单独起）：

```xml
<!-- UAV navsat_transform：消费 MAVROS GPS + local odom，输出对齐到公共 datum -->
<node pkg="robot_localization" type="navsat_transform_node"
      name="navsat_transform_uav" output="screen">
  <rosparam command="load"
      file="$(find mobile_manipulator)/config/navsat_transform_uav.yaml"/>
  <!-- 输入重映射：navsat 期望 gps/fix、imu/data、odometry/filtered -->
  <remap from="gps/fix"          to="/mavros/global_position/global"/>
  <remap from="imu/data"         to="/mavros/imu/data"/>
  <remap from="odometry/filtered" to="/mavros/local_position/odom"/>
  <!-- 输出：odometry/gps（UAV 在公共 map 中的 GPS 位姿，可选回灌）-->
</node>
```

> 录制点云时，把 LiDAR 累积点云发布/保存在 **`map`** 帧（仓库 `pcd_to_elevation.launch` 已假设 PCD 的 `frame_id=map`）。机制 A 下该 `map` 即公共 map，PCD 落盘即对齐，无需后处理。

> ⚠️ **真机注意**：`magnetic_declination_radians` 必须填作业地真实磁偏角（可查 NOAA 离线表，提前写死，不联网）；UAV 与 UGV 的 `datum` 三个数必须**逐位一致**。

### 2.3 机制 B：离线静态 UTM 偏移（真机 / 跨 ROS master 备选）

当 UAV 与 UGV 不在同一 ROS master、无法共享 TF 时，不跑 UAV 侧 navsat，改为**一次性算出平移并发布静态 TF**：

```bash
# 1) 记录 UAV arm 点 GPS（建图开始那一刻）与公共 datum
#    uav_home: 47.3979000 8.5458000   datum: 47.3977419 8.5455938
# 2) 用 §4 节点内的 ll_to_utm 算两点 UTM，输出 Δ=(ΔE, ΔN)
python3 - <<'PY'
from coordinate_align_checker import ll_to_utm      # 复用同一换算
e0,n0,z0 = ll_to_utm(47.3977419, 8.5455938)         # datum
eu,nu,zu = ll_to_utm(47.3979000, 8.5458000)         # uav_home
assert z0==zu, "跨 UTM zone，需特殊处理"
print("static_transform_publisher 平移: x=%.3f y=%.3f"%(eu-e0, nu-n0))
PY
```

```xml
<!-- 把 UAV 本地 map（PX4 ENU）平移到公共 map；yaw=0（两者都对齐真北）-->
<node pkg="tf2_ros" type="static_transform_publisher" name="uav_to_common_map"
      args="ΔE ΔN 0  0 0 0  map uav_map"/>
```

或直接把已存 PCD 整体平移 `Δ`（PCL/numpy 加常量），再喂 `pcd_to_elevation.launch`。**本质和机制 A 等价，只是离线一次性做。**

---

## 3. UGV 侧配置

UGV 侧只需把现有 `navsat_transform.yaml` 从"首帧自动 datum"改成"固定 datum"，**与 UAV 用同一个 datum**。其余沿用仓库现状。

完整 `config/navsat_transform.yaml`（在仓库现有内容基础上，仅末尾两项有实质改动，已标注）：

```yaml
# config/navsat_transform.yaml  (UGV 侧)
# 把 Husky 仿真 GPS(navsat/fix) -> map 系 odometry/gps，供全局 EKF 融合。
frequency: 30.0
transform_timeout: 0.1
delay: 3.0

magnetic_declination_radians: 0.0   # 仿真=0；真机填作业地磁偏角（与 UAV 同值）
yaw_offset: 0.0
zero_altitude: true

broadcast_cartesian_transform: true # 发布 utm->map（RViz 显示 GPS 轨迹）
publish_filtered_gps: true
use_odometry_yaw: false             # UGV 有双 EKF，用融合后的航向（静止时 GPS 无航向）

# === 改动：从"首帧自动 datum"改为"固定 datum"，与 UAV 完全一致 ===
wait_for_datum: true                # 原为 false（首帧自动）→ 改 true 用固定 datum
datum: [47.3977419, 8.5455938, 0.0] # [lat, lon, yaw]；与 navsat_transform_uav.yaml 同值
```

> **同 datum / 同 UTM zone 检查**：UAV 的 `datum` 与 UGV 的 `datum` 必须三位一致。若作业区恰好骑在两个 UTM zone 边界（经度每 6° 一带），把 datum 选在带内、确保两机同带；本系统假设单带作业。

> ⚠️ **真机注意**：
> - datum 取**勘测点**或**UGV 首帧稳定 fix**；记下来写死进两个 yaml。也可保留 `wait_for_datum: false`，改为开机后用服务统一下发：
>   ```bash
>   rosservice call /navsat_transform/datum \
>     "geo_pose: {position: {latitude: 47.3977419, longitude: 8.5455938, altitude: 0.0}, \
>                 orientation: {x: 0,y: 0,z: 0,w: 1}}"
>   ```
>   （对 UAV 侧 `/navsat_transform_uav/datum` 下发**同样的值**。）
> - 磁偏角两侧同值。

仿真里 UGV 的 GPS 引用点要与 datum 一致——确认 Husky 的 hector GPS 插件 `referenceLatitude/referenceLongitude` ＝ datum 的 lat/lon（在 husky 的 xacro/world 里）；PX4 的 `PX4_HOME_LAT/LON` 也设成同值。

---

## 4. 对齐验证节点 `coordinate_align_checker.py`

放在 `src/uav_truth_tracker/scripts/coordinate_align_checker.py`（`chmod +x`）。它把 UAV、UGV 的 GPS 各自转 UTM，算水平偏差并发布 JSON 状态。**UTM 换算自实现，离线可用，无第三方依赖。**

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""coordinate_align_checker.py

订阅 UAV GPS 与 UGV GPS，各自转 UTM，计算水平距离偏差，判断两套坐标系是否对齐。
- 偏差 < align_threshold(0.5m)  -> loginfo  "ALIGNED"
- 偏差 > warn_threshold(2.0m)   -> logwarn  "MISALIGNED"
- 之间                          -> loginfo  "MARGINAL"
发布 /coordinate_align/status (std_msgs/String, JSON)。

注意：本节点比较的是"两台车 GPS 的 UTM 距离"。使用方式有二：
  1) 对齐自检：让 UAV 停在 UGV 旁（已知基线 baseline_offset），看测得距离是否≈基线；
  2) datum 一致性：两侧 navsat 用同一 datum 时，本节点 + 各自 map 位姿可交叉验证。
真正的对齐保证来自"两侧 navsat 共用同一 datum"(§2/§3)；本节点是上线前的快速 sanity check。
"""

import json
import math

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


def ll_to_utm(lat_deg, lon_deg):
    """WGS84 经纬度 -> UTM(easting, northing, zone)。离线、无依赖。
    标准横轴墨卡托正算（Chuck Gantz / NGA 公式）。"""
    a = 6378137.0                      # WGS84 长半轴
    f = 1.0 / 298.257223563            # 扁率
    k0 = 0.9996                        # UTM 中央经线尺度
    e2 = f * (2 - f)                   # 第一偏心率平方
    ep2 = e2 / (1 - e2)                # 第二偏心率平方
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    zone = int((lon_deg + 180) / 6) + 1
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)   # 该带中央经线
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    T = math.tan(lat) ** 2
    C = ep2 * math.cos(lat) ** 2
    A = math.cos(lat) * (lon - lon0)
    M = a * ((1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat
             - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat)
             + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat)
             - (35 * e2**3 / 3072) * math.sin(6 * lat))
    easting = (k0 * N * (A + (1 - T + C) * A**3 / 6
               + (5 - 18 * T + T**2 + 72 * C - 58 * ep2) * A**5 / 120) + 500000.0)
    northing = (k0 * (M + N * math.tan(lat) * (A**2 / 2
                + (5 - T + 9 * C + 4 * C**2) * A**4 / 24
                + (61 - 58 * T + T**2 + 600 * C - 330 * ep2) * A**6 / 720)))
    if lat_deg < 0:
        northing += 10000000.0         # 南半球偏移
    return easting, northing, zone


class CoordinateAlignChecker(object):
    def __init__(self):
        self.uav_topic = rospy.get_param("~uav_gps_topic", "/mavros/global_position/global")
        self.ugv_topic = rospy.get_param("~ugv_gps_topic", "navsat/fix")
        self.align_thr = float(rospy.get_param("~align_threshold", 0.5))   # m
        self.warn_thr = float(rospy.get_param("~warn_threshold", 2.0))     # m
        self.baseline = float(rospy.get_param("~baseline_offset", 0.0))    # 已知两天线基线(m)
        self.rate_hz = float(rospy.get_param("~rate", 1.0))
        self.fix_timeout = float(rospy.get_param("~fix_timeout", 5.0))     # 数据过期阈值

        self.uav_fix = None
        self.ugv_fix = None
        self.uav_stamp = None
        self.ugv_stamp = None

        self.pub = rospy.Publisher("/coordinate_align/status", String, queue_size=1)
        rospy.Subscriber(self.uav_topic, NavSatFix, self._uav_cb, queue_size=1)
        rospy.Subscriber(self.ugv_topic, NavSatFix, self._ugv_cb, queue_size=1)
        rospy.loginfo("[align] UAV=%s  UGV=%s  align<%.2fm warn>%.2fm baseline=%.2fm",
                      self.uav_topic, self.ugv_topic, self.align_thr, self.warn_thr, self.baseline)

    def _valid(self, msg):
        # status<0 表示无定位；NaN 直接丢弃
        return (msg.status.status >= 0
                and not math.isnan(msg.latitude) and not math.isnan(msg.longitude))

    def _uav_cb(self, msg):
        if self._valid(msg):
            self.uav_fix, self.uav_stamp = msg, rospy.Time.now()

    def _ugv_cb(self, msg):
        if self._valid(msg):
            self.ugv_fix, self.ugv_stamp = msg, rospy.Time.now()

    def _fresh(self, stamp):
        return stamp is not None and (rospy.Time.now() - stamp).to_sec() < self.fix_timeout

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def _tick(self):
        status = {"state": "WAITING", "deviation_m": None,
                  "uav_utm": None, "ugv_utm": None, "zone_match": None,
                  "stamp": rospy.Time.now().to_sec()}

        if not (self._fresh(self.uav_stamp) and self._fresh(self.ugv_stamp)):
            status["state"] = "NO_FIX"
            rospy.logwarn_throttle(5.0, "[align] 等待有效 GPS（UAV/UGV 至少一方无新数据）")
            self.pub.publish(String(data=json.dumps(status)))
            return

        eu, nu, zu = ll_to_utm(self.uav_fix.latitude, self.uav_fix.longitude)
        eg, ng, zg = ll_to_utm(self.ugv_fix.latitude, self.ugv_fix.longitude)
        raw = math.hypot(eu - eg, nu - ng)
        dev = abs(raw - self.baseline)        # 扣掉已知基线后的"对齐偏差"

        status["uav_utm"] = [round(eu, 3), round(nu, 3), zu]
        status["ugv_utm"] = [round(eg, 3), round(ng, 3), zg]
        status["zone_match"] = (zu == zg)
        status["deviation_m"] = round(dev, 3)

        if zu != zg:
            status["state"] = "ZONE_MISMATCH"
            rospy.logwarn_throttle(5.0, "[align] UTM zone 不一致 UAV=%d UGV=%d，需选同带 datum", zu, zg)
        elif dev < self.align_thr:
            status["state"] = "ALIGNED"
            rospy.loginfo_throttle(5.0, "[align] ALIGNED 偏差=%.3fm (<%.2f) ✓", dev, self.align_thr)
        elif dev > self.warn_thr:
            status["state"] = "MISALIGNED"
            rospy.logwarn_throttle(2.0, "[align] MISALIGNED 偏差=%.3fm (>%.2f) ✗ 检查两侧 datum 是否一致",
                                   dev, self.warn_thr)
        else:
            status["state"] = "MARGINAL"
            rospy.loginfo_throttle(5.0, "[align] MARGINAL 偏差=%.3fm", dev)

        self.pub.publish(String(data=json.dumps(status)))


def main():
    rospy.init_node("coordinate_align_checker")
    CoordinateAlignChecker().spin()


if __name__ == "__main__":
    main()
```

> ⚠️ **真机注意**：UGV GPS 话题按实际改 `~ugv_gps_topic`（如 `/ublox/fix`、`/gps/fix`）。做对齐自检时把 UAV 停在 UGV 旁、量好两天线水平距离填 `baseline_offset`，理想读数应 ≈ 0。

---

## 5. 集成 launch：`ugv_terrain_nav.launch`

放在 `src/mobile_manipulator/launch/ugv_terrain_nav.launch`。整合：UGV 双 EKF + navsat_transform（固定 datum）→ `include` 已有 `pcd_to_elevation.launch` → GridMap→costmap 桥 → move_base（含地形层）→ 对齐校验节点。

> ⚠️ **关于 "grid_map_costmap_2d 插件"**：本机 ROS 只装了 `grid_map_core/ros/filters/...`，**没有可直接挂进 move_base 的 grid_map costmap layer 插件**（`grid_map_costmap_2d` 在 grid_map 仓库里是一个 C++ `Costmap2DConverter` 工具库，不是 pluginlib 层）。Noetic 下**依赖最轻**的正路是：把高程图的 `slope`/`elevation` 层**阈值化成 `nav_msgs/OccupancyGrid`**，再用 move_base 自带的 `costmap_2d::StaticLayer` 吃进去。下面用一个轻量桥接节点 `terrain_costmap_bridge.py` 实现，不引入任何重型包。

### 5.1 地形栅格桥 `terrain_costmap_bridge.py`

放 `src/mobile_manipulator/scripts/terrain_costmap_bridge.py`（`chmod +x`）。订阅 `/elevation_mapping/elevation_map_postprocessed`，按坡度阈值把 GridMap 转 OccupancyGrid：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""terrain_costmap_bridge.py
GridMap(高程+坡度) -> nav_msgs/OccupancyGrid，供 move_base StaticLayer 使用。
依赖仅 grid_map_msgs（随 grid_map_ros 安装），不引入重型包。
规则：slope > slope_max -> 致命(100)；无效/空洞 -> 未知(-1)；其余 -> 自由(0)。
"""
import numpy as np
import rospy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid


class TerrainCostmapBridge(object):
    def __init__(self):
        self.layer = rospy.get_param("~slope_layer", "slope")          # 坡度层名
        self.slope_max = float(rospy.get_param("~slope_max", 0.45))    # rad，~26°，超过=不可通行
        self.unknown_as_free = bool(rospy.get_param("~unknown_as_free", False))
        self.pub = rospy.Publisher("/terrain_costmap", OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber("/elevation_mapping/elevation_map_postprocessed",
                         GridMap, self._cb, queue_size=1)
        rospy.loginfo("[terrain_bridge] layer=%s slope_max=%.2frad", self.layer, self.slope_max)

    def _cb(self, gm):
        if self.layer not in gm.layers:
            rospy.logwarn_throttle(5.0, "[terrain_bridge] 找不到层 '%s'，现有: %s",
                                   self.layer, gm.layers)
            return
        info = gm.info
        data = gm.data[gm.layers.index(self.layer)]
        rows = data.layout.dim[0].size       # grid_map: dim[0]=column(对应+x), dim[1]=row
        cols = data.layout.dim[1].size
        # grid_map 数据按列优先、原点在中心、+x 向后存储；转成行优先的 OccupancyGrid
        m = np.array(data.data, dtype=np.float32).reshape(rows, cols)

        og = OccupancyGrid()
        og.header.frame_id = info.header.frame_id      # = "map"
        og.header.stamp = rospy.Time.now()
        og.info.resolution = info.resolution
        og.info.width = cols
        og.info.height = rows
        # grid_map 中心在 info.pose.position；OccupancyGrid 原点在左下角
        og.info.origin.position.x = info.pose.position.x - info.length_x / 2.0
        og.info.origin.position.y = info.pose.position.y - info.length_y / 2.0
        og.info.origin.orientation.w = 1.0

        out = np.full(m.shape, -1 if not self.unknown_as_free else 0, dtype=np.int8)
        valid = ~np.isnan(m)
        out[valid & (m <= self.slope_max)] = 0
        out[valid & (m > self.slope_max)] = 100
        # grid_map 行列方向与 OccupancyGrid 相反，翻转对齐
        out = np.flipud(np.fliplr(out))
        og.data = out.flatten(order="C").tolist()
        self.pub.publish(og)


def main():
    rospy.init_node("terrain_costmap_bridge")
    TerrainCostmapBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
```

> 坐标轴/翻转细节按实际 RViz 叠加效果微调（grid_map 与 OccupancyGrid 的行列、原点约定不同）；上线时用 §6 的叠加检查校准一次即可。

### 5.2 地形 costmap 层参数 `config/nav/terrain_layer.yaml`

把地形 OccupancyGrid 作为一个 `StaticLayer` 注入 global / local costmap（与现有 static/obstacle/inflation 层并存）：

```yaml
# config/nav/terrain_layer.yaml
terrain_layer:                       # costmap_2d::StaticLayer
  map_topic: /terrain_costmap        # terrain_costmap_bridge 发布的地形栅格
  subscribe_to_updates: false
  track_unknown_space: true          # 高程图没覆盖的区域=未知，不当作自由空间
  use_maximum: true                  # 与其他层取最大代价（地形障碍不被覆盖）
  trinary_costmap: true              # 0/未知/100 三值
  lethal_cost_threshold: 100
```

### 5.3 完整 `ugv_terrain_nav.launch`

```xml
<launch>
  <!-- ====================================================================
       UGV 地形感知导航：UTM 对齐 + UAV 高程图 + move_base(含地形层)
       前置：UGV 已 spawn、传感器在跑（本 launch 只负责定位/建图/导航这层）。
       UAV 高程图来源：复用已有 pcd_to_elevation.launch（不改其内部）。
       ==================================================================== -->

  <!-- 公共 datum：UAV/UGV 必须一致（占位值，按作业点改）-->
  <arg name="datum_lat" default="47.3977419"/>
  <arg name="datum_lon" default="8.5455938"/>
  <arg name="pcd_file"  default="$(env HOME)/pointcloud_maps/uav_points_map_latest.pcd"/>
  <arg name="ugv_gps_topic" default="navsat/fix"/>   <!-- 真机改 /ublox/fix 等 -->
  <arg name="rviz" default="true"/>

  <!-- ── 1. UGV 局部 EKF：wheel odom + IMU -> odom->base_link ── -->
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization"
        clear_params="true">
    <rosparam command="load" file="$(find husky_control)/config/localization.yaml"/>
  </node>

  <!-- ── 2. UGV 全局 EKF：+GPS -> map->odom ── -->
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_global"
        clear_params="true">
    <rosparam command="load" file="$(find mobile_manipulator)/config/ekf_global.yaml"/>
    <remap from="odometry/filtered" to="odometry/filtered_map"/>
  </node>

  <!-- ── 3. navsat_transform：固定 datum，与 UAV 对齐 ── -->
  <node pkg="robot_localization" type="navsat_transform_node" name="navsat_transform"
        output="screen" clear_params="true">
    <rosparam command="load" file="$(find mobile_manipulator)/config/navsat_transform.yaml"/>
    <!-- 用 launch 参数覆盖 datum，保证与 UAV 同值（也可直接写死在 yaml）-->
    <rosparam param="datum" subst_value="true">[$(arg datum_lat), $(arg datum_lon), 0.0]</rosparam>
    <param name="wait_for_datum" value="true"/>
    <remap from="imu/data"          to="imu/data"/>
    <remap from="gps/fix"           to="$(arg ugv_gps_topic)"/>
    <remap from="odometry/filtered" to="odometry/filtered_map"/>
  </node>

  <!-- ── 4. UAV 高程图：复用已有 launch（机器人在跑 -> static_tf:=false）── -->
  <include file="$(find mobile_manipulator)/launch/pcd_to_elevation.launch">
    <arg name="pcd_file"  value="$(arg pcd_file)"/>
    <arg name="static_tf" value="false"/>   <!-- 用真实 map->odom->base_link，避免 TF 冲突 -->
    <arg name="rviz"      value="false"/>    <!-- 统一用本 launch 的 RViz -->
  </include>

  <!-- ── 5. GridMap -> OccupancyGrid 地形桥 ── -->
  <node pkg="mobile_manipulator" type="terrain_costmap_bridge.py"
        name="terrain_costmap_bridge" output="screen">
    <param name="slope_layer" value="slope"/>
    <param name="slope_max"   value="0.45"/>   <!-- ~26°，按 Husky 越野能力调 -->
  </node>

  <!-- ── 6. move_base：global+local costmap 各加一层 terrain StaticLayer ── -->
  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <rosparam file="$(find mobile_manipulator)/config/nav/costmap_common.yaml"  command="load" ns="global_costmap"/>
    <rosparam file="$(find mobile_manipulator)/config/nav/costmap_common.yaml"  command="load" ns="local_costmap"/>
    <rosparam file="$(find mobile_manipulator)/config/nav/global_costmap.yaml"  command="load"/>
    <rosparam file="$(find mobile_manipulator)/config/nav/local_costmap.yaml"   command="load"/>
    <rosparam file="$(find mobile_manipulator)/config/nav/dwa_local_planner.yaml" command="load"/>
    <!-- 地形层参数注入两个 costmap 命名空间 -->
    <rosparam file="$(find mobile_manipulator)/config/nav/terrain_layer.yaml" command="load" ns="global_costmap"/>
    <rosparam file="$(find mobile_manipulator)/config/nav/terrain_layer.yaml" command="load" ns="local_costmap"/>
    <!-- 在两个 costmap 的 plugins 列表追加 terrain_layer -->
    <rosparam ns="global_costmap" subst_value="true">
      plugins:
        - {name: static_layer,    type: "costmap_2d::StaticLayer"}
        - {name: terrain_layer,   type: "costmap_2d::StaticLayer"}
        - {name: obstacle_layer,  type: "costmap_2d::ObstacleLayer"}
        - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
    </rosparam>
    <rosparam ns="local_costmap" subst_value="true">
      plugins:
        - {name: terrain_layer,   type: "costmap_2d::StaticLayer"}
        - {name: obstacle_layer,  type: "costmap_2d::ObstacleLayer"}
        - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
    </rosparam>
  </node>

  <!-- ── 7. 坐标对齐校验 ── -->
  <node pkg="uav_truth_tracker" type="coordinate_align_checker.py"
        name="coordinate_align_checker" output="screen">
    <param name="uav_gps_topic" value="/mavros/global_position/global"/>
    <param name="ugv_gps_topic" value="$(arg ugv_gps_topic)"/>
    <param name="align_threshold" value="0.5"/>
    <param name="warn_threshold"  value="2.0"/>
    <param name="baseline_offset" value="0.0"/>
  </node>

  <node if="$(arg rviz)" name="terrain_nav_rviz" pkg="rviz" type="rviz"
        args="-d $(find mobile_manipulator)/rviz/elevation_mapping.rviz"/>
</launch>
```

> ⚠️ **真机注意**：
> - `static_tf:=false` 必须保证真实 `map→odom→base_link` 已由双 EKF 发布，否则 elevation_mapping 取不到位姿。
> - `local_costmap` 默认 `rolling_window: true`，地形 StaticLayer 在滚动窗口下也能用（StaticLayer 支持非全局尺寸）；若报尺寸冲突，把 local 的 terrain_layer 去掉、只在 global 用。
> - `terrain_layer.yaml` 的 `slope_max` 按真机底盘越野能力标定。

---

## 6. 快速验证清单

跑通后，按下表逐项确认。

### 6.1 TF 树

```bash
rosrun rqt_tf_tree rqt_tf_tree     # 或 rosrun tf2_tools view_frames.py
```
应看到单一连贯链：`utm → map → odom → base_link → ...`（`utm→map` 由 navsat_transform 广播；`map→odom` 由 ekf_global）。**不应**出现两个互不相连的 map。

### 6.2 关键 topic 在线

```bash
rostopic list | grep -E "odometry/gps|odometry/filtered_map|elevation_map_postprocessed|terrain_costmap|coordinate_align"
```
预期：
```
/odometry/gps
/odometry/filtered_map
/elevation_mapping/elevation_map_postprocessed
/terrain_costmap
/coordinate_align/status
```

### 6.3 对齐状态（核心）

```bash
rostopic echo -n1 /coordinate_align/status
```
对齐良好时（UAV 停 UGV 旁、baseline=0）：
```json
{"state": "ALIGNED", "deviation_m": 0.18,
 "uav_utm": [465123.451, 5248712.903, 32],
 "ugv_utm": [465123.402, 5248713.075, 32],
 "zone_match": true, "stamp": 1.71e9}
```
看到 `state: ALIGNED`、`zone_match: true`、`deviation_m < 0.5` 即对齐成功。
若 `state: MISALIGNED` 或 `deviation_m` 是几十~几百米 → 两侧 datum 没填一致，回 §2.2/§3 核对。

### 6.4 高程图与 UGV 同框

```bash
# 高程图 frame 应为 map
rostopic echo -n1 /elevation_mapping/elevation_map_postprocessed/info/header/frame_id   # -> "map"
# 地形栅格 frame 应为 map
rostopic echo -n1 /terrain_costmap/header/frame_id                                      # -> "map"
```
RViz 里同时叠加：`Grid`(map)、`Map`(/terrain_costmap)、`GridMap`(高程图)、UGV 模型——UGV 应落在高程图覆盖范围内、且地形障碍与实际地貌吻合。

### 6.5 costmap 吃到地形层

```bash
rosrun rqt_reconfigure rqt_reconfigure
# 展开 move_base/global_costmap，应看到 terrain_layer 在 plugins 列表里
rostopic echo -n1 /move_base/global_costmap/costmap | head     # 致命格出现在陡坡处
```
在 RViz 给 UGV 发 2D Nav Goal，全局路径应**绕开陡坡/障碍地形**（terrain_layer 把这些标成致命）。

### 6.6 上线前 datum 自检（一行）

```bash
# 两侧 datum 是否落在同一 UTM zone（经度同带）
python3 -c "import math;print('zone',int((8.5455938+180)/6)+1)"   # datum 经度填这里
```

---

## 附：改动文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `config/navsat_transform.yaml` | 改 | `wait_for_datum: true` + 固定 `datum`（UGV 侧）|
| `config/navsat_transform_uav.yaml` | 新增 | UAV 侧 navsat，同一 datum（机制 A）|
| `config/nav/terrain_layer.yaml` | 新增 | 地形 StaticLayer 参数 |
| `scripts/coordinate_align_checker.py` | 新增 | 对齐校验（`uav_truth_tracker`）|
| `scripts/terrain_costmap_bridge.py` | 新增 | GridMap→OccupancyGrid（`mobile_manipulator`）|
| `launch/ugv_terrain_nav.launch` | 新增 | 集成入口（`mobile_manipulator`）|
| `pcd_to_elevation.launch` | **不改** | 仅 `include` |

> 新增 Python 节点记得在对应包 `CMakeLists.txt` 的 `catkin_install_python` 里登记（或 `chmod +x` 后用 `rosrun`），并 `catkin build` 后 `source devel/setup.bash`。

---

*文档对应仓库提交 `0b36fb7`。datum 占位值 `47.3977419, 8.5455938`（苏黎世，PX4 SITL 默认 home）请按真实作业点替换，UAV/UGV 两侧务必一致。*
