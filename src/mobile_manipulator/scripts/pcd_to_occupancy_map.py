#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pcd_to_occupancy_map.py

把 UAV 建好的 PCD 点云投影成一张【静态、全局、坡度可通行】2D 栅格
(nav_msgs/OccupancyGrid)，直接当 move_base 全局 costmap 的 static_layer 用
（方案①：UGV 在 UAV 地图上做地形感知导航——能爬缓坡、避陡坡/楼）。

为什么按【坡度】而不是【绝对高度】判障碍：
  按绝对高度判，会把一座能爬的缓坡山丘整个误判成障碍，车根本不上去。
  按坡度判：缓坡(<slope_max)=可走(能爬)，陡坡/悬崖/楼的近垂直边=障碍。

判据（对每个 cell）：
  ground DEM   = cell 内最低点 z（地面高程），空洞用最近邻填、再中值滤波去重叠噪声
  slope        = DEM 梯度 -> 倾角；slope > slope_max -> 障碍（陡坡/楼边/悬崖）
  step(粗糙度) = 机器人半径窗口内 (max_z - ground)；step > step_max -> 障碍（楼/墙/坎）
  其余有数据   = 可走(0)；UAV 没覆盖 = 未知(-1)

只依赖 numpy + scipy.ndimage，离线可用。同时发布坡度可视化 /terrain_slope。
"""

import math
import os
import struct
import threading

import numpy as np
import rospy
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from scipy import ndimage
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def ll_to_utm(lat_deg, lon_deg):
    """WGS84 经纬度 -> UTM(easting, northing, zone)。离线，无依赖（GPS 锚定用）。"""
    a = 6378137.0
    f = 1.0 / 298.257223563
    k0 = 0.9996
    e2 = f * (2 - f)
    ep2 = e2 / (1 - e2)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    zone = int((lon_deg + 180) / 6) + 1
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
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
        northing += 10000000.0
    return easting, northing, zone


def load_pcd_xyz(path):
    """读 ASCII PCD 的 x y z。返回 Nx3 float 数组。"""
    with open(path, "r") as f:
        lines = f.readlines()
    data_start = None
    fmt_ascii = True
    for i, ln in enumerate(lines):
        s = ln.strip()
        if s.startswith("DATA"):
            fmt_ascii = s.split()[1].lower() == "ascii"
            data_start = i + 1
            break
    if data_start is None:
        raise ValueError("PCD: no DATA line in %s" % path)
    if not fmt_ascii:
        raise ValueError("PCD: only ASCII supported (got binary) %s" % path)
    rows = []
    for ln in lines[data_start:]:
        p = ln.split()
        if len(p) < 3:
            continue
        try:
            rows.append((float(p[0]), float(p[1]), float(p[2])))
        except ValueError:
            continue
    return np.array(rows, dtype=np.float64)


def build_traversability(points, resolution=0.25, slope_max_deg=30.0,
                         slope_free_deg=12.0, obstacle_height=0.4,
                         min_obstacle_points=3, ground_window=2.0,
                         smooth_cells=3, margin=2.0, min_valid_frac=0.2,
                         slope_cost_max=0.0, ugv_clearance=50.0):
    """点云 -> (可通行栅格, 坡度图)。纯函数，便于单测。

    设计原则（按用户要求）：【不用坡度判能不能走】——所有地面都可走，让车直接上坡；
    能不能上、稳不稳由【车本身】解决（压低质心 + 加大轮摩擦 + 缓速/缓转 DWA），
    不靠把坡标成障碍——否则容易把大片地标死、车反而无路可走。
      所有有数据的地面          : free(0)，直接走/直接上
      结构伸进【车身高度走廊】  : 致命障碍(100)        <- 唯一的障碍来源
        (走廊 = 地面+obstacle_height .. 地面+ugv_clearance；高过车顶的树冠/棚顶【不算】,
         UGV 能从底下开过去查雷;墙/楼/低梁/树干/棚柱伸进走廊才致命)
      slope_cost_max > 0 时     : 给坡一点"偏好平地"的软代价(上限 slope_cost_max<<100，
                                  永不阻断)；默认 0 = 坡完全免代价，纯靠车爬。
    坡度仍照算，只用于可视化(/terrain_slope + terrain_cloud 绿→红着色)，信息不丢。

    地面高程用【中值滤波】估计，楼顶等高点钻不进地面面 -> 坡度只反映真实地形。
    返回 (occ_int8 [H,W], slope_deg [H,W], ground[H,W], origin_x, origin_y, resolution)。
    occ: 0=可走(含各种坡), [1..slope_cost_max]=软代价坡(可选), 100=障碍(楼/墙), -1=未知。"""
    if points.shape[0] == 0:
        raise ValueError("empty point cloud")
    x, y, z = points[:, 0].copy(), points[:, 1].copy(), points[:, 2].copy()

    # 1) 全局极端离群点先粗去
    keep = z >= (np.percentile(z, 0.5) - 1.0)
    x, y, z = x[keep], y[keep], z[keep]

    xmin = float(np.floor(x.min() - margin))
    ymin = float(np.floor(y.min() - margin))
    w = int(np.ceil((x.max() + margin - xmin) / resolution))
    h = int(np.ceil((y.max() + margin - ymin) / resolution))
    n = w * h
    ix = np.clip(((x - xmin) / resolution).astype(np.int64), 0, w - 1)
    iy = np.clip(((y - ymin) / resolution).astype(np.int64), 0, h - 1)
    flat = iy * w + ix

    # 2) 逐 cell 统计离群点剔除（关键：解决"累积建图让平地高低不平"）。
    #    累积多帧后同一块地面落了很多 z 略不同的点，还混入个别偏低噪点。按每个 cell
    #    的 z 均值/标准差丢掉明显偏低的点，否则后面取 min 会被噪点拉低、再被 min 滤波
    #    扩散成一大片"假坑"。
    cnt = np.bincount(flat, minlength=n).astype(np.float64)
    s1 = np.bincount(flat, weights=z, minlength=n)
    s2 = np.bincount(flat, weights=z * z, minlength=n)
    cs = np.maximum(cnt, 1.0)
    mean = s1 / cs
    std = np.sqrt(np.clip(s2 / cs - mean * mean, 0.0, None))
    keep = z >= (mean[flat] - 2.0 * std[flat] - 0.10)
    x, y, z, ix, iy, flat = x[keep], y[keep], z[keep], ix[keep], iy[keep], flat[keep]

    count = np.bincount(flat, minlength=n).reshape(h, w)
    valid = count > 0
    dem_min = np.full(n, np.inf)
    np.minimum.at(dem_min, flat, z)          # 每 cell 最低点（地面候选，已去离群）
    dem_min = dem_min.reshape(h, w)          # 空洞处 = +inf

    # 3) 地面估计：先填洞，再【中值滤波】。关键：用中值而不是 min 滤波——
    #    min 滤波会把单个低噪点放大成一片"假坑/假坡"(低空密点云尤其严重，
    #    会让平地坡度虚高到 20°+ 全被误判成障碍)。中值滤波直接把噪点剔除 -> 平地是平的。
    bad = ~np.isfinite(dem_min)
    if bad.any() and (~bad).any():
        idx = ndimage.distance_transform_edt(bad, return_distances=False,
                                             return_indices=True)
        dem_min = dem_min[tuple(idx)]
    gwin = max(3, int(round(ground_window / resolution)) | 1)   # 奇数窗口
    ground = ndimage.median_filter(dem_min, size=gwin)

    # 4) 坡度：在【再平滑一层】的地面上按窗口算梯度，抗残余噪声 -> 只有真陡坡/楼边才大
    sw = max(3, int(smooth_cells) * 2 + 1)
    ground_s = ndimage.uniform_filter(ground, size=sw)
    gy, gx = np.gradient(ground_s, resolution)
    slope_deg = np.degrees(np.arctan(np.hypot(gx, gy)))

    # 5) 障碍 = 高出本地地面【任意高度】的结构(楼/墙/树/路灯/指示牌/树干…)一律算 —— 完备优先。
    #    ★为什么不再设"车顶以上当能钻过"的上限(ugv_clearance):细高物(路灯/指示牌/树干)从空中只在
    #      【高处】被激光扫到、底下没扫到,设上限会把它们当"高架空、可从底下钻过"漏掉 -> 高程图缺这些
    #      障碍 -> UGV 直接撞上(用户反馈)。故 ugv_clearance 默认很大(≈无上限,凡高出地面就标)。
    #      想让 UGV 钻树冠/棚下查雷再把它调小(注意:树干常被树冠遮挡、空中扫不到,调小有撞树干风险)。
    #    ★坡度补偿(修"坡面一堆不可通行小块"):ground 是窗口滤波估计,坡面实际点天然高它 ~tan(坡)*
    #      窗口半径,按地形坡度(封顶35°=可驾驶上限)补进阈值 -> 坡面 free;墙/楼/细高物远高于补偿阈值仍致命。
    gpt = ground[iy, ix]
    slope_tol = np.tan(np.radians(np.minimum(slope_deg[iy, ix], 35.0))) * (gwin * resolution * 0.5)
    inband = (z > gpt + obstacle_height + slope_tol) & (z < gpt + ugv_clearance + slope_tol)
    high_count = np.bincount(flat[inband], minlength=w * h).reshape(h, w)

    # 6) 分类：坡度【不判障碍】——所有地面 free(0)，让车直接上坡；只有楼/墙致命。
    #    稳定性靠车本身(质心+摩擦+缓速)，不靠把坡标死，否则大片地被标成不可走、车没路走。
    occ = np.full((h, w), -1, dtype=np.int8)
    occ[valid] = 0
    if slope_cost_max > 0:
        # 可选软代价：让规划器在有得选时略偏好平地；上限 slope_cost_max(<<100)，永不阻断。
        span = max(1e-6, slope_max_deg - slope_free_deg)
        cost = np.clip(np.round((slope_deg - slope_free_deg) / span * slope_cost_max),
                       0, slope_cost_max)
        band = valid & (slope_deg > slope_free_deg)
        occ[band] = cost[band].astype(np.int8)
    lethal = valid & (high_count >= max(1, min_obstacle_points))   # 唯一障碍：楼/墙
    occ[lethal] = 100

    # 7) 去噪【但务必保住细高真障碍】(路灯/指示牌/树干这种 1~2 格的,绝不能用形态学 opening 抹掉——
    #    那正是之前这些细障碍在高程图里消失、UGV 撞上去的根因)。改按【点数】区分:真细高障碍累积点
    #    多,噪声尖点点少 —— 只把【完全孤立(3x3 内无其它致命格)且点数 < 噪声阈值】的当噪声去掉;
    #    点数够(细高真障碍)或有致命邻居(成片楼/树)的一律保留。
    obs = occ == 100
    nbr = (ndimage.convolve(obs.astype(np.int16), np.ones((3, 3), np.int16), mode="constant")
           - obs.astype(np.int16))                          # 3x3 内其它致命格数
    noise = obs & (nbr == 0) & (high_count < max(5, min_obstacle_points + 2))
    occ[noise] = 0

    # 8) 去掉空地上零散的孤立有效 cell（杂散绿点/单次噪声返回）：valid 邻居占比太低 = 孤立散点 -> 标未知。
    #    ★只清【空地】格,绝不清致命格 —— 否则会把孤立的细高障碍(路灯/树干)当散点删掉,UGV 又撞上。
    valid_now = occ != -1
    frac = ndimage.uniform_filter(valid_now.astype(np.float32), size=5)
    occ[valid_now & (occ != 100) & (frac < min_valid_frac)] = -1
    return occ, slope_deg, ground, xmin, ymin, float(resolution)


def to_occupancy_msg(data2d, origin_x, origin_y, resolution, frame_id="map", stamp=None):
    og = OccupancyGrid()
    og.header.frame_id = frame_id
    og.header.stamp = stamp if stamp is not None else rospy.Time.now()
    og.info.resolution = resolution
    og.info.height, og.info.width = data2d.shape
    og.info.origin.position.x = origin_x
    og.info.origin.position.y = origin_y
    og.info.origin.orientation.w = 1.0
    og.data = np.asarray(data2d, dtype=np.int8).flatten(order="C").tolist()
    return og


def build_terrain_cloud(occ, ground, slope_deg, slope_max_deg, origin_x, origin_y,
                        resolution, frame_id="map", stamp=None, z_offset=-0.15):
    """每个有数据的 cell -> 一个 3D 点 (x, y, 地面高程z)，按【坡度】连续着色，
    让"可通行性"一眼可见：
      亮绿=平地(随便走) -> 黄=缓坡(能爬) -> 橙红=陡坡 -> 深红=致命障碍(楼/悬崖)。
    给 RViz 显示三维地形面（高度=真实地面高程）。
    z_offset: 把整张地形面整体下沉一点(默认 -0.15m)，铺在地面【之下】，这样 UGV 模型
    显示在格子【之上】不被吞没（配合 RViz 用 Flat Squares 平铺成地毯，不要 3D 方块）。"""
    ys, xs = np.where(occ != -1)
    if ys.size == 0:
        rows = []
    else:
        px = origin_x + (xs + 0.5) * resolution
        py = origin_y + (ys + 0.5) * resolution
        pz = ground[ys, xs].astype(np.float64) + z_offset
        is_lethal = occ[ys, xs] == 100
        # 坡度归一化 t∈[0,1]，绿->黄->红 渐变：t=0 绿(平地), t=0.5 黄(缓坡), t=1 红(陡)
        t = np.clip(slope_deg[ys, xs] / max(1e-6, slope_max_deg), 0.0, 1.0)
        r = np.clip(2.0 * t, 0.0, 1.0)
        g = np.clip(2.0 * (1.0 - t), 0.0, 1.0)
        ri = (r * 225).astype(np.uint32)
        gi = (g * 205).astype(np.uint32)
        bi = np.full(ys.shape, 55, dtype=np.uint32)
        # 致命障碍(楼/悬崖)统一深红，最醒目，明显区别于可走的绿
        ri = np.where(is_lethal, 200, ri).astype(np.uint32)
        gi = np.where(is_lethal, 25, gi).astype(np.uint32)
        bi = np.where(is_lethal, 25, bi).astype(np.uint32)
        rgb_int = (ri << 16) | (gi << 8) | bi
        col = rgb_int.view(np.float32).astype(np.float64)
        rows = list(zip(px.tolist(), py.tolist(), pz.tolist(), col.tolist()))
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.FLOAT32, 1)]
    header = Header(frame_id=frame_id)
    header.stamp = stamp if stamp is not None else rospy.Time(0)
    return pc2.create_cloud(header, fields, rows)


class TerrainProjector(object):
    """把点云投影成地形栅格/着色云。两种数据源：
      静态(默认): 读一次 PCD 文件(队友/离线用)。
      实时(Phase4): input_cloud_topic 非空 -> 订阅 UAV 累积点云
                    (/uav0/mapping/points_accumulated),边飞边重投影,/terrain_map
                    +/terrain_cloud 实时长出来,UGV 直接吃到刚建好的图。"""

    def __init__(self):
        self.resolution = float(rospy.get_param("~resolution", 0.25))
        self.slope_max_deg = float(rospy.get_param("~slope_max_deg", 30.0))        # 坡度着色/软代价归一化
        self.slope_free_deg = float(rospy.get_param("~slope_free_deg", 12.0))      # 软代价起点(slope_cost_max>0时)
        self.slope_cost_max = float(rospy.get_param("~slope_cost_max", 0.0))       # 0=坡免代价(直接上)
        self.obstacle_height = float(rospy.get_param("~obstacle_height", 0.4))     # 高出地面多少算障碍(矮坎能压过)
        # 障碍高度上限。默认很大(50)≈无上限:凡高出地面的结构(楼/树/路灯/指示牌/树干)都标障碍,
        # 保证高程图障碍完备、UGV 不撞细高物。调小(如 1.5=车顶高)可让 UGV 钻树冠/棚下查雷,但细高物
        # 只在高处被空中扫到时会被漏标 -> 有撞树干/立柱风险。
        self.ugv_clearance = float(rospy.get_param("~ugv_clearance", 50.0))
        self.min_obstacle_points = int(rospy.get_param("~min_obstacle_points", 3))
        self.ground_window = float(rospy.get_param("~ground_window", 2.0))
        self.min_valid_frac = float(rospy.get_param("~min_valid_frac", 0.2))
        self.smooth_cells = int(rospy.get_param("~smooth_cells", 3))
        self.margin = float(rospy.get_param("~margin", 2.0))
        self.cloud_z_offset = float(rospy.get_param("~cloud_z_offset", -0.15))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.min_points = int(rospy.get_param("~min_points", 200))   # 点太少不投影(实时刚起飞时)
        self.off_x = float(rospy.get_param("~offset_x", 0.0))
        self.off_y = float(rospy.get_param("~offset_y", 0.0))

        # The live UAV cloud is cumulative, so returns from the moving UGV
        # otherwise remain forever and turn its whole driven path into a red
        # obstacle trail.  Sample map->base_link independently of the cloud,
        # rasterize the swept chassis footprint, and remove only points above
        # the locally measured ground band.  ``map->base_link`` comes from a
        # two-dimensional navigation chain and its z does not follow hills;
        # using that TF z erased real uphill terrain after the UGV drove over
        # it.  Ground height is therefore estimated from the persistent cloud
        # in each swept cell, while TF supplies only the swept XY footprint.
        self.ugv_self_filter_enabled = bool(rospy.get_param(
            "~ugv_self_filter_enabled", True
        ))
        self.ugv_base_frame = str(rospy.get_param(
            "~ugv_self_filter_base_frame", "base_link"
        )).strip().lstrip("/")
        self.ugv_filter_radius = max(float(rospy.get_param(
            "~ugv_self_filter_radius", 0.72
        )), self.resolution)
        self.ugv_filter_cell_size = max(float(rospy.get_param(
            "~ugv_self_filter_cell_size", 0.10
        )), 0.05)
        self.ugv_filter_history_spacing = max(float(rospy.get_param(
            "~ugv_self_filter_history_spacing", 0.08
        )), 0.02)
        self.ugv_filter_z_min_relative = float(rospy.get_param(
            "~ugv_self_filter_z_min_relative", 0.08
        ))
        self.ugv_filter_z_max_relative = max(float(rospy.get_param(
            "~ugv_self_filter_z_max_relative", 2.00
        )), self.ugv_filter_z_min_relative + 0.10)
        self.ugv_filter_ground_percentile = min(max(float(rospy.get_param(
            "~ugv_self_filter_ground_percentile", 10.0
        )), 0.0), 50.0)
        self._ugv_filter_lock = threading.Lock()
        self._ugv_filter_cells = {}
        self._ugv_last_filter_pose = None
        self._ugv_filtered_points = 0
        if self.ugv_self_filter_enabled:
            self.ugv_tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
            self.ugv_tf_listener = tf2_ros.TransformListener(self.ugv_tf_buffer)
            sample_rate = max(float(rospy.get_param(
                "~ugv_self_filter_sample_rate", 5.0
            )), 1.0)
            rospy.Timer(
                rospy.Duration(1.0 / sample_rate), self._sample_ugv_footprint
            )

        topic = rospy.get_param("~topic", "/terrain_map")
        slope_topic = rospy.get_param("~slope_topic", "/terrain_slope")
        cloud_topic = rospy.get_param("~cloud_topic", "/terrain_cloud")
        self.pub = rospy.Publisher(topic, OccupancyGrid, queue_size=1, latch=True)
        self.pub_slope = rospy.Publisher(slope_topic, OccupancyGrid, queue_size=1, latch=True)
        self.pub_cloud = rospy.Publisher(cloud_topic, PointCloud2, queue_size=1, latch=True)
        # 把 UAV 点云对齐显示给 RViz：不重打包点(慢、会卡死),而是【廉价改 frame_id + 一条 TF】
        # 把整片云搬到世界/UGV 帧 —— O(1)、实时、随飞随长。aligned_frame 经 TF(map->aligned_frame
        # =出生点偏移)落到世界帧;RViz 显示 aligned_cloud_topic 即和高程图/UGV 重合。非空才发。
        self.aligned_cloud_topic = str(rospy.get_param("~aligned_cloud_topic", "")).strip()
        self.aligned_frame = rospy.get_param("~aligned_frame", "uav_aligned")
        self.pub_aligned = (rospy.Publisher(self.aligned_cloud_topic, PointCloud2,
                                            queue_size=1, latch=True)
                            if self.aligned_cloud_topic else None)
        self.tf_bc = tf2_ros.TransformBroadcaster() if self.pub_aligned else None
        self._last_aligntf_stamp = rospy.Time(0)  # 去重:同戳不重发,免 TF_REPEATED_DATA

        self._lock = threading.Lock()
        self._msg = None
        self._msg_slope = None
        self._cloud = None

        # === 自动对齐(Phase4 关键) ===
        # UAV 点云是在【MAVROS 局部系】里(原点=PX4 EKF 原点,SITL 里通常=无人机出生点),
        # 而 UGV 的 map 系原点=GPS datum=Gazebo 世界原点。两者差一个【出生点偏移】(本仓约
        # (0,-18))。不校正 -> UAV 建的图整体平移,UGV"看着走空地、实际撞墙"。
        # auto_align=true 时:offset = Gazebo真值(UAV) - MAVROS(UAV) (一个常量),把云搬到
        # 世界/UGV 帧。只在【本投影器内部】平移,不动 EGO/全局规划(它们仍跑 MAVROS 局部系)。
        self.auto_align = bool(rospy.get_param("~auto_align", False))
        self._truth_xy = None
        self._mavros_xy = None
        self._auto_off = None

        # 实时模式：订阅 UAV 累积点云,定时重投影。
        self.input_cloud_topic = str(rospy.get_param("~input_cloud_topic", "")).strip()
        if self.input_cloud_topic:
            self._latest_msg = None
            rebuild_period = float(rospy.get_param("~rebuild_period", 3.0))
            rospy.Subscriber(self.input_cloud_topic, PointCloud2, self._cloud_cb, queue_size=1)
            rospy.Timer(rospy.Duration(rebuild_period), self._rebuild_timer)
            rospy.logwarn("[pcd_to_occupancy_map] LIVE/Phase4: 订阅 %s,每 %.1fs 重投影 -> %s + %s",
                          self.input_cloud_topic, rebuild_period, topic, cloud_topic)
            if self.auto_align:
                self.align_model = rospy.get_param("~align_model_name", "iris_depth_camera")
                rospy.Subscriber(rospy.get_param("~align_truth_topic", "/gazebo/model_states"),
                                 ModelStates, self._truth_cb, queue_size=1)
                rospy.Subscriber(rospy.get_param("~align_mavros_topic", "/mavros/local_position/odom"),
                                 Odometry, self._mavros_cb, queue_size=1)
                rospy.logwarn("[pcd_to_occupancy_map] 自动对齐开:offset=Gazebo真值(%s)-MAVROS,"
                              "把 UAV 云对齐到 UGV/世界帧", self.align_model)
        else:
            self._static_load_and_build()

        # 统一以 2Hz 重发(latch 已保证晚订阅者也收到;刷新时间戳防 costmap 嫌旧)。
        rospy.Timer(rospy.Duration(2.0), self._republish)

    # --- 核心：一组点 -> 栅格/坡度/着色云,存起来等重发 ---
    def _process(self, pts):
        if pts is None or pts.shape[0] < self.min_points:
            rospy.loginfo_throttle(5.0, "[pcd_to_occupancy_map] 点数 %s < %d,暂不投影",
                                   0 if pts is None else pts.shape[0], self.min_points)
            return False
        ox_off, oy_off = self._eff_offset()
        p = pts.astype(np.float64, copy=True)
        p[:, 0] += ox_off          # 平移到 datum/世界/UGV 帧(对齐 UGV,杜绝"走空地撞墙")
        p[:, 1] += oy_off
        p, self_filtered = self._filter_ugv_swept_points(p)
        if p.shape[0] < self.min_points:
            rospy.loginfo_throttle(
                5.0,
                "[pcd_to_occupancy_map] 自车轨迹过滤后点数 %d < %d,暂不投影",
                p.shape[0], self.min_points,
            )
            return False
        try:
            occ, slope_deg, ground, ox, oy, res = build_traversability(
                p, self.resolution, self.slope_max_deg, self.slope_free_deg,
                self.obstacle_height, self.min_obstacle_points, self.ground_window,
                self.smooth_cells, self.margin, min_valid_frac=self.min_valid_frac,
                slope_cost_max=self.slope_cost_max, ugv_clearance=self.ugv_clearance)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, "[pcd_to_occupancy_map] 投影跳过: %s", exc)
            return False
        slope_viz = np.clip(slope_deg / max(1e-6, self.slope_max_deg) * 100.0, 0, 100).astype(np.int8)
        slope_viz[occ == -1] = -1
        cloud = build_terrain_cloud(occ, ground, slope_deg, self.slope_max_deg, ox, oy, res,
                                    self.frame_id, z_offset=self.cloud_z_offset)
        msg = to_occupancy_msg(occ, ox, oy, res, self.frame_id)
        msg_slope = to_occupancy_msg(slope_viz, ox, oy, res, self.frame_id)
        with self._lock:
            self._msg, self._msg_slope, self._cloud = msg, msg_slope, cloud
        rospy.loginfo_throttle(5.0, "[pcd_to_occupancy_map] 投影 %d 点(自车过滤=%d) -> %dx%d 可走=%d 障碍(楼)=%d 未知=%d",
                               p.shape[0], self_filtered,
                               occ.shape[1], occ.shape[0],
                               int((occ == 0).sum()), int((occ == 100).sum()), int((occ == -1).sum()))
        return True

    def _sample_ugv_footprint(self, _event):
        if not self.ugv_self_filter_enabled:
            return
        try:
            transform = self.ugv_tf_buffer.lookup_transform(
                self.frame_id,
                self.ugv_base_frame,
                rospy.Time(0),
                rospy.Duration(0.03),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0,
                "[pcd_to_occupancy_map] 等待 %s->%s 自车过滤TF: %s",
                self.frame_id, self.ugv_base_frame, exc,
            )
            return
        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        z = float(transform.transform.translation.z)
        with self._ugv_filter_lock:
            if self._ugv_last_filter_pose is not None:
                dx = x - self._ugv_last_filter_pose[0]
                dy = y - self._ugv_last_filter_pose[1]
                if math.hypot(dx, dy) < self.ugv_filter_history_spacing:
                    return
            self._ugv_last_filter_pose = (x, y, z)
            inv = 1.0 / self.ugv_filter_cell_size
            center_x = int(math.floor(x * inv))
            center_y = int(math.floor(y * inv))
            radius_cells = int(math.ceil(self.ugv_filter_radius * inv))
            radius_sq = self.ugv_filter_radius * self.ugv_filter_radius
            for cell_x in range(center_x - radius_cells,
                                center_x + radius_cells + 1):
                cell_center_x = (float(cell_x) + 0.5) * self.ugv_filter_cell_size
                for cell_y in range(center_y - radius_cells,
                                    center_y + radius_cells + 1):
                    cell_center_y = (
                        (float(cell_y) + 0.5) * self.ugv_filter_cell_size
                    )
                    if ((cell_center_x - x) ** 2
                            + (cell_center_y - y) ** 2 > radius_sq):
                        continue
                    # Value is kept only for backwards-compatible diagnostics;
                    # filtering below derives ground z from the cloud itself.
                    self._ugv_filter_cells[(cell_x, cell_y)] = z

    def _filter_ugv_swept_points(self, points):
        if not self.ugv_self_filter_enabled or points.size == 0:
            return points, 0
        with self._ugv_filter_lock:
            cells = dict(self._ugv_filter_cells)
        if not cells:
            return points, 0
        inv = 1.0 / self.ugv_filter_cell_size
        point_cells = np.floor(points[:, :2] * inv).astype(np.int64)
        keep = np.ones(points.shape[0], dtype=bool)
        matched = []
        heights_by_cell = {}
        for index, (cell_x, cell_y) in enumerate(point_cells):
            key = (int(cell_x), int(cell_y))
            if key not in cells:
                continue
            matched.append((index, key))
            heights_by_cell.setdefault(key, []).append(float(points[index, 2]))

        # The accumulated UAV cloud normally contains a ground return from
        # before the vehicle entered each cell.  A low robust percentile keeps
        # that terrain and removes only chassis/arm returns above it.  If a
        # cell has just one return, preserve it: retaining a possible dynamic
        # speck is safer than punching an unknown hole into the saved map.
        ground_by_cell = {}
        for key, heights in heights_by_cell.items():
            if len(heights) < 2:
                continue
            ground_by_cell[key] = float(np.percentile(
                np.asarray(heights, dtype=np.float64),
                self.ugv_filter_ground_percentile,
            ))

        for index, key in matched:
            ground_z = ground_by_cell.get(key)
            if ground_z is None:
                continue
            relative_z = float(points[index, 2]) - ground_z
            if (self.ugv_filter_z_min_relative <= relative_z
                    <= self.ugv_filter_z_max_relative):
                keep[index] = False
        removed = int((~keep).sum())
        self._ugv_filtered_points += removed
        return points[keep], removed

    def _eff_offset(self):
        """生效偏移：auto_align 测到的 Gazebo真值-MAVROS 优先,否则手填 offset_x/y。"""
        if self.auto_align and self._auto_off is not None:
            return self._auto_off
        return (self.off_x, self.off_y)

    def _cloud_cb(self, msg):
        # 只存原始 msg(便宜),点云解析放到 3s 定时器里做(避免每帧都解析拖慢)。
        self._latest_msg = msg
        # 廉价对齐显示：只改 frame_id(点数据不动)+ 配套 TF 把整片云搬到世界帧。实时随飞随长。
        if self.pub_aligned is not None:
            msg.header.frame_id = self.aligned_frame
            self.pub_aligned.publish(msg)
            self._broadcast_align_tf(msg.header.stamp)

    def _broadcast_align_tf(self, stamp):
        if self.tf_bc is None:
            return
        s = stamp if stamp != rospy.Time(0) else rospy.Time.now()
        if s == self._last_aligntf_stamp:   # 同戳跳过,免 TF_REPEATED_DATA
            return
        self._last_aligntf_stamp = s
        ox_off, oy_off = self._eff_offset()
        t = TransformStamped()
        t.header.stamp = s
        t.header.frame_id = self.frame_id        # map / 世界
        t.child_frame_id = self.aligned_frame    # UAV 云所在帧
        t.transform.translation.x = ox_off
        t.transform.translation.y = oy_off
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_bc.sendTransform(t)

    def _rebuild_timer(self, _evt):
        if self._latest_msg is None:
            return
        pts = np.array(list(pc2.read_points(self._latest_msg, field_names=("x", "y", "z"),
                                            skip_nans=True)), dtype=np.float64)
        self._process(pts)

    # --- 自动对齐回调：offset = Gazebo真值(UAV) - MAVROS(UAV)，常量，把云搬到世界帧 ---
    def _truth_cb(self, msg):
        try:
            i = msg.name.index(self.align_model)
        except ValueError:
            return
        self._truth_xy = (msg.pose[i].position.x, msg.pose[i].position.y)
        self._update_auto_off()

    def _mavros_cb(self, msg):
        self._mavros_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._update_auto_off()

    def _update_auto_off(self):
        if self._truth_xy is None or self._mavros_xy is None:
            return
        off = (self._truth_xy[0] - self._mavros_xy[0],
               self._truth_xy[1] - self._mavros_xy[1])
        if self._auto_off is None or \
                math.hypot(off[0] - self._auto_off[0], off[1] - self._auto_off[1]) > 0.3:
            rospy.logwarn_throttle(10.0, "[pcd_to_occupancy_map] 自动对齐 offset=(%.2f,%.2f)m"
                                   " (Gazebo真值-MAVROS)", off[0], off[1])
        self._auto_off = off

    def _republish(self, _evt):
        self._broadcast_align_tf(rospy.Time.now())   # 保持 map->aligned_frame 常新,RViz 总能变换
        with self._lock:
            if self._msg is None:
                return
            now = rospy.Time.now()
            self._msg.header.stamp = now
            self._msg_slope.header.stamp = now
            self._cloud.header.stamp = now
            self.pub.publish(self._msg)
            self.pub_slope.publish(self._msg_slope)
            self.pub_cloud.publish(self._cloud)

    # --- 静态文件模式(默认/离线): GPS 锚定 + 读一次 PCD + 投影一次 ---
    def _static_load_and_build(self):
        pcd_file = os.path.expanduser(rospy.get_param(
            "~pcd_file", "~/pointcloud_maps/uav_points_map_latest.pcd"))
        # === GPS 锚定：把 PCD(UAV map=出生点 帧) 平移到公共 datum 帧 ===
        #   ① 建图时自动存的 origin sidecar ② 显式 uav_origin_lat/lon ③ 直接 offset_x/y
        datum_lat = float(rospy.get_param("~datum_lat", 0.0))
        datum_lon = float(rospy.get_param("~datum_lon", 0.0))
        uav_lat = float(rospy.get_param("~uav_origin_lat", 0.0))
        uav_lon = float(rospy.get_param("~uav_origin_lon", 0.0))
        origin_file = os.path.expanduser(rospy.get_param(
            "~origin_file", "~/pointcloud_maps/uav_map_origin.yaml"))
        anchored = False
        if datum_lat != 0.0 and origin_file and os.path.isfile(origin_file):
            try:
                import yaml
                with open(origin_file) as f:
                    o = yaml.safe_load(f)
                ed, nd, zd = ll_to_utm(datum_lat, datum_lon)
                if int(o.get("utm_zone", -1)) == zd:
                    self.off_x = float(o["utm_easting"]) - ed
                    self.off_y = float(o["utm_northing"]) - nd
                    anchored = True
                    rospy.logwarn("[pcd_to_occupancy_map] 自动GPS锚定(sidecar %s) offset=(%.2f,%.2f)m",
                                  origin_file, self.off_x, self.off_y)
                else:
                    rospy.logwarn("[pcd_to_occupancy_map] origin sidecar UTM zone%d != datum zone%d，跳过",
                                  int(o.get("utm_zone", -1)), zd)
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("[pcd_to_occupancy_map] 读 origin sidecar 失败: %s", exc)
        if not anchored and datum_lat != 0.0 and uav_lat != 0.0:
            ed, nd, zd = ll_to_utm(datum_lat, datum_lon)
            eu, nu, zu = ll_to_utm(uav_lat, uav_lon)
            self.off_x, self.off_y = (eu - ed), (nu - nd)
            rospy.logwarn("[pcd_to_occupancy_map] GPS锚定(显式 uav_origin) offset=(%.2f,%.2f)m",
                          self.off_x, self.off_y)

        if not os.path.isfile(pcd_file):
            rospy.logfatal("[pcd_to_occupancy_map] PCD not found: %s", pcd_file)
            return
        rospy.loginfo("[pcd_to_occupancy_map] loading %s ...", pcd_file)
        self._process(load_pcd_xyz(pcd_file))


def main():
    rospy.init_node("pcd_to_occupancy_map")
    TerrainProjector()
    rospy.spin()


if __name__ == "__main__":
    main()
