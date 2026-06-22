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

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
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
                         slope_cost_max=0.0):
    """点云 -> (可通行栅格, 坡度图)。纯函数，便于单测。

    设计原则（按用户要求）：【不用坡度判能不能走】——所有地面都可走，让车直接上坡；
    能不能上、稳不稳由【车本身】解决（压低质心 + 加大轮摩擦 + 缓速/缓转 DWA），
    不靠把坡标成障碍——否则容易把大片地标死、车反而无路可走。
      所有有数据的地面          : free(0)，直接走/直接上
      楼/墙(成片高出本地地面)   : 致命障碍(100)        <- 唯一的障碍来源
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

    # 5) 楼/墙：cell 内高出"本地地面"的点数 >= 阈值（抗单点噪声）
    tall = z > (ground[iy, ix] + obstacle_height)
    high_count = np.bincount(flat[tall], minlength=w * h).reshape(h, w)

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

    # 7) 去掉零散的孤立【致命】障碍 cell（噪声尖点），成片的楼保留
    obs = occ == 100
    keep_obs = ndimage.binary_opening(obs, structure=np.ones((3, 3)))
    occ[obs & ~keep_obs] = 0

    # 8) 去掉空地上零散的孤立有效 cell（杂散绿点/单次噪声返回）：
    #    valid 邻居占比太低 = 孤立散点 -> 标未知，高程图空地就干净了。
    valid_now = occ != -1
    frac = ndimage.uniform_filter(valid_now.astype(np.float32), size=5)
    occ[valid_now & (frac < min_valid_frac)] = -1
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


def main():
    rospy.init_node("pcd_to_occupancy_map")
    pcd_file = os.path.expanduser(rospy.get_param(
        "~pcd_file", "~/pointcloud_maps/uav_points_map_latest.pcd"))
    resolution = float(rospy.get_param("~resolution", 0.25))
    slope_max_deg = float(rospy.get_param("~slope_max_deg", 30.0))         # 仅用于坡度着色/软代价归一化
    slope_free_deg = float(rospy.get_param("~slope_free_deg", 12.0))       # 软代价起点(slope_cost_max>0时)
    slope_cost_max = float(rospy.get_param("~slope_cost_max", 0.0))        # 0=坡完全免代价(直接上)；>0=轻微偏好平地的软代价上限
    obstacle_height = float(rospy.get_param("~obstacle_height", 0.4))      # 高出地面多少算障碍(含矮障碍)
    min_obstacle_points = int(rospy.get_param("~min_obstacle_points", 3))  # 抗噪：高点数阈值
    ground_window = float(rospy.get_param("~ground_window", 2.0))          # 地面估计窗口(m)
    min_valid_frac = float(rospy.get_param("~min_valid_frac", 0.2))        # 去空地零散散点
    smooth_cells = int(rospy.get_param("~smooth_cells", 3))
    margin = float(rospy.get_param("~margin", 2.0))
    cloud_z_offset = float(rospy.get_param("~cloud_z_offset", -0.15))      # 地形面下沉量，UGV 显示在格子之上
    frame_id = rospy.get_param("~frame_id", "map")
    topic = rospy.get_param("~topic", "/terrain_map")
    slope_topic = rospy.get_param("~slope_topic", "/terrain_slope")
    cloud_topic = rospy.get_param("~cloud_topic", "/terrain_cloud")

    # === GPS 锚定：把 PCD(UAV map=出生点 帧) 平移到公共 datum 帧 ===
    # 偏移来源优先级：① 建图时自动存的 origin sidecar（零配置，开机即用）
    #                ② 显式给的 uav_origin_lat/lon  ③ 直接给的米制 offset_x/y
    off_x = float(rospy.get_param("~offset_x", 0.0))
    off_y = float(rospy.get_param("~offset_y", 0.0))
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
                off_x = float(o["utm_easting"]) - ed
                off_y = float(o["utm_northing"]) - nd
                anchored = True
                rospy.logwarn("[pcd_to_occupancy_map] 自动GPS锚定(sidecar %s) offset=(%.2f,%.2f)m",
                              origin_file, off_x, off_y)
            else:
                rospy.logwarn("[pcd_to_occupancy_map] origin sidecar UTM zone%d != datum zone%d，跳过",
                              int(o.get("utm_zone", -1)), zd)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("[pcd_to_occupancy_map] 读 origin sidecar 失败: %s", exc)
    if not anchored and datum_lat != 0.0 and uav_lat != 0.0:
        ed, nd, zd = ll_to_utm(datum_lat, datum_lon)
        eu, nu, zu = ll_to_utm(uav_lat, uav_lon)
        off_x, off_y = (eu - ed), (nu - nd)
        rospy.logwarn("[pcd_to_occupancy_map] GPS锚定(显式 uav_origin) offset=(%.2f,%.2f)m", off_x, off_y)

    if not os.path.isfile(pcd_file):
        rospy.logfatal("[pcd_to_occupancy_map] PCD not found: %s", pcd_file)
        return
    rospy.loginfo("[pcd_to_occupancy_map] loading %s ...", pcd_file)
    pts = load_pcd_xyz(pcd_file)
    pts[:, 0] += off_x          # 平移到 datum 帧（GPS 锚定）
    pts[:, 1] += off_y
    occ, slope_deg, ground, ox, oy, res = build_traversability(
        pts, resolution, slope_max_deg, slope_free_deg, obstacle_height,
        min_obstacle_points, ground_window, smooth_cells, margin,
        min_valid_frac=min_valid_frac, slope_cost_max=slope_cost_max)
    rospy.logwarn("[pcd_to_occupancy_map] %dx%d res=%.2f origin=(%.1f,%.1f) offset=(%.1f,%.1f) "
                  "坡度不判障碍(slope_cost_max=%.0f) -> 可走free=%d 软代价坡=%d "
                  "障碍(楼/墙)100=%d 未知=%d",
                  occ.shape[1], occ.shape[0], res, ox, oy, off_x, off_y, slope_cost_max,
                  int((occ == 0).sum()), int(((occ >= 1) & (occ <= 99)).sum()),
                  int((occ == 100).sum()), int((occ == -1).sum()))

    # 坡度可视化：缩放到 0..100（>=slope_max 记 100），未知记 -1
    slope_viz = np.clip(slope_deg / max(1e-6, slope_max_deg) * 100.0, 0, 100).astype(np.int8)
    slope_viz[occ == -1] = -1

    cloud = build_terrain_cloud(occ, ground, slope_deg, slope_max_deg, ox, oy, res,
                                frame_id, z_offset=cloud_z_offset)

    pub = rospy.Publisher(topic, OccupancyGrid, queue_size=1, latch=True)
    pub_slope = rospy.Publisher(slope_topic, OccupancyGrid, queue_size=1, latch=True)
    pub_cloud = rospy.Publisher(cloud_topic, PointCloud2, queue_size=1, latch=True)
    msg = to_occupancy_msg(occ, ox, oy, res, frame_id)
    msg_slope = to_occupancy_msg(slope_viz, ox, oy, res, frame_id)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        msg.header.stamp = now
        msg_slope.header.stamp = now
        cloud.header.stamp = now
        pub.publish(msg)
        pub_slope.publish(msg_slope)
        pub_cloud.publish(cloud)
        rate.sleep()


if __name__ == "__main__":
    main()
