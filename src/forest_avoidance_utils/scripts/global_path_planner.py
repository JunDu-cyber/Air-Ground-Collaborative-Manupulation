#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""global_path_planner.py — UAV 全局规划前端(配合 EGO 局部避障)。

EGO 是局部 rebound 规划器，绕不过大体量建筑。本节点在它前面加一层全局规划：
用 360° 雷达累积一张全局地图，A* 搜一条【绕开大楼】的路，纯追踪喂给 EGO 子目标，
EGO 负责每一段的局部避障(穿树林/细障碍/动态)。—— 既能绕楼、又能穿树林。

三个关键：
  ① 障碍按【最高点 高出本地地面 > obstacle_height】判(不是绝对高度,缓坡山丘不算障碍)。
     用【最高点】对建图时楼底被遮挡鲁棒:无人机在 4m 只扫到楼上半截,最高点仍高 -> 照判障碍 -> 绕。
     (曾试过"结构最低面/变高度钻顶"那套,会把楼底没扫到的高楼误当高架空可飞过 -> 不绕楼直冲撞楼,已回退。)
  ② 只把【大体量障碍(楼/墙)】留给全局绕行；【小障碍(树)】按连通域面积滤掉、不全局绕,
     交给 EGO 局部避障穿过去 —— 否则树林里 A* 被一堆"树墙"堵成无解。
  ③ A* 有路时沿安全折线插值子目标喂给 EGO；A* 暂时无解时保持上一条安全
     指令并等待地图更新，不把最终目标直接穿楼发送，也不生成来回抢占的反向点。
"""

import collections
import heapq
import math
import threading

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from scipy import ndimage
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def metric_inflation(raw_obstacles, resolution, clearance_m):
    """Inflate by Euclidean metres and return (inflated, clearance metres)."""
    raw = np.asarray(raw_obstacles, dtype=bool)
    if raw.any():
        clearance = ndimage.distance_transform_edt(~raw) * float(resolution)
        inflated = raw | (clearance <= float(clearance_m) + 1e-9)
    else:
        clearance = np.full(raw.shape, np.inf, dtype=np.float32)
        inflated = raw.copy()
    return inflated, clearance


def nearest_mask_cell(mask, target, max_radius_cells=None):
    """Return the true Euclidean-nearest True cell, without scan-order bias."""
    candidates = np.argwhere(mask)
    if candidates.size == 0:
        return None
    target_array = np.asarray(target, dtype=np.float64)
    distance_sq = np.sum((candidates - target_array) ** 2, axis=1)
    if max_radius_cells is not None:
        within = distance_sq <= float(max_radius_cells) ** 2 + 1e-9
        if not np.any(within):
            return None
        candidates = candidates[within]
        distance_sq = distance_sq[within]
    best = candidates[int(np.argmin(distance_sq))]
    return int(best[0]), int(best[1])


def polyline_length(points):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points[:-1], points[1:])
    )


def astar(grid, start, goal, clearance=None, resolution=1.0,
          preferred_clearance=0.0, clearance_weight=0.0):
    """8 邻接 A*。grid: bool 障碍图。start/goal: (row,col)。返回 cell 列表或 None。"""
    if start is None or goal is None:
        return None
    h, w = grid.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w):
        return None
    if not (0 <= goal[0] < h and 0 <= goal[1] < w):
        return None
    if grid[goal] or grid[start]:
        return None
    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    openq = [(0.0, start)]
    came = {}
    g = {start: 0.0}

    def hh(a):
        return (math.hypot(a[0] - goal[0], a[1] - goal[1])
                * float(resolution))

    while openq:
        _, cur = heapq.heappop(openq)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for dr, dc, cost in nbrs:
            nr, nc = cur[0] + dr, cur[1] + dc
            if not (0 <= nr < h and 0 <= nc < w) or grid[nr, nc]:
                continue
            # A diagonal cannot squeeze through two touching obstacle corners.
            if dr != 0 and dc != 0 and (
                    grid[cur[0] + dr, cur[1]]
                    or grid[cur[0], cur[1] + dc]):
                continue
            penalty = 0.0
            if clearance is not None and clearance_weight > 0.0:
                shortage = max(
                    0.0, float(preferred_clearance) - float(clearance[nr, nc])
                )
                penalty = float(clearance_weight) * shortage
            ng = g[cur] + cost * float(resolution) + penalty
            if ng < g.get((nr, nc), 1e18):
                g[(nr, nc)] = ng
                came[(nr, nc)] = cur
                heapq.heappush(openq, (ng + hh((nr, nc)), (nr, nc)))
    return None


def line_clear(grid, a, b):
    """Bresenham: a->b 直线是否全程无障碍。"""
    r0, c0 = a
    r1, c1 = b
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
            return False
        if grid[r, c]:
            return False
        if r == r1 and c == c1:
            return True
        e2 = 2 * err
        old_r, old_c = r, c
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
        if r != old_r and c != old_c:
            if (grid[old_r, c] or grid[r, old_c]):
                return False


def shortcut_cells(cells, grid):
    """String-pull a cell path while preserving collision/corner checks."""
    if cells is None or len(cells) <= 2:
        return cells
    result = [cells[0]]
    index = 0
    while index < len(cells) - 1:
        next_index = len(cells) - 1
        while (next_index > index + 1
               and not line_clear(grid, cells[index], cells[next_index])):
            next_index -= 1
        result.append(cells[next_index])
        index = next_index
    return result


class GlobalPlanner(object):
    def __init__(self):
        self.res = float(rospy.get_param("~resolution", 0.5))
        self.half = float(rospy.get_param("~half_extent", 100.0))
        # 只全局绕【大体量障碍(楼)】：连通域占地 >= 此面积(m²)才当楼绕行；更小的(树)滤掉，
        # 交给 EGO 局部避障穿过去。设 0 关闭(所有突起都全局绕，树林里会被堵死)。
        self.min_obstacle_area = float(rospy.get_param("~min_obstacle_area", 10.0))
        self.ground_window = float(rospy.get_param("~ground_window", 2.0))   # 本地地面估计窗口(m)
        # RViz 2D Goal 的 z=0 只表示“没有指定高度”。此值仅在收到目标时
        # 尚无可用里程计高度时作兜底，不再作为“地面+巡航高度”叠加到每个子目标。
        self.height_above_ground = float(rospy.get_param("~goal_z", 4.0))
        self.inflate_m = float(rospy.get_param("~inflate", 1.5))
        self.reach = float(rospy.get_param("~reach_radius", 1.5))
        self.lookahead = float(rospy.get_param("~lookahead", 7.0))
        self.min_replan = float(rospy.get_param("~min_replan_interval", 0.5))
        self.safe_dist = float(rospy.get_param("~safe_dist", 0.8))
        self.retreat_dist = float(rospy.get_param("~retreat_dist", 2.0))
        # 反应式"贴楼退后"默认【关闭】：局部避障交给 EGO(用户要求"用ego避障,不要反应式")。
        # 开着会在建图新障碍刚出现/EGO抄近道贴墙时触发退后→又前进→来回振荡。
        self.enable_retreat = bool(rospy.get_param("~enable_retreat", False))
        # 子目标最低高度(m)：开阔处地面估计被低噪点拉低时不把无人机往地上压。
        self.min_z = float(rospy.get_param("~min_subgoal_z", 2.0))
        # 高出【本地地面】> 此高度算突起(楼/树)。★用【最高点-地面】判障碍,对建图时楼底被遮挡
        #   也鲁棒(无人机在4m只扫到楼上半截,只要够高就判障碍)——【之前能稳定绕楼靠的就是这个】。
        #   之前换成"结构最低面/钻顶"那套(变高度穿顶)会把【楼底没扫到的高楼】误当成"高架空、可飞过"
        #   -> 不绕楼 -> EGO 直冲撞楼,故本次回退掉那套。
        self.obstacle_height = float(rospy.get_param("~obstacle_height", 1.5))
        # 子目标高度平滑(限幅,m/tick):起飞/换目标时高度别瞬跳,EGO 跟得住。
        self.max_z_step = float(rospy.get_param("~max_z_step", 0.6))
        # 普通目标锁定收到时的高度；只有目标 XY 本身落在已确认障碍格内，
        # 才允许把目标抬到已观测障碍顶部以上。路径中间的高地/楼体不得改写目标 z。
        self.zero_goal_z_epsilon = max(
            0.0, float(rospy.get_param("~zero_goal_z_epsilon", 0.05)))
        self.goal_obstacle_clearance = max(
            0.0, float(rospy.get_param("~goal_obstacle_clearance", 1.5)))
        self.goal_z_reach_tolerance = max(
            0.1, float(rospy.get_param("~goal_z_reach_tolerance", 0.5)))
        self.cmd_z = None
        # 障碍图重算周期(s)：tick 5Hz 复用缓存,真正重算降到这个频率。
        self.obst_period = float(rospy.get_param("~obst_update_period", 0.5))
        # Per-cell robust vertical columns replace lifetime min/max extrema.
        # A few delayed/misaligned scans must not turn a flat road into a
        # multi-metre wall, while repeated building facade returns remain.
        self.column_sample_capacity = max(
            16, int(rospy.get_param("~column_sample_capacity", 96)))
        self.column_samples_per_frame = max(
            1, int(rospy.get_param("~column_samples_per_frame", 5)))
        self.column_min_samples = max(
            3, int(rospy.get_param("~column_min_samples", 5)))
        self.column_low_quantile = min(
            0.45, max(0.0, float(rospy.get_param("~column_low_quantile", 0.10))))
        self.column_high_quantile = max(
            0.55, min(1.0, float(rospy.get_param("~column_high_quantile", 0.90))))
        self.start_clearance_radius = max(
            0.0, float(rospy.get_param("~start_clearance_radius", 0.60)))

        self.W = max(2, int(2 * self.half / self.res))
        self.origin = -self.half
        self.gwin = max(1, int(round(self.ground_window / self.res)) | 1)
        self.inflate_cells = max(1, int(round(self.inflate_m / self.res)))
        self.min_obstacle_cells = max(0, int(round(self.min_obstacle_area / (self.res * self.res))))
        self.ground = np.full((self.W, self.W), np.inf)
        self.height = np.full((self.W, self.W), -np.inf)
        self._column_samples = {}
        self._obst_cache = None        # (obst, local_ground) 缓存,tick 5Hz 复用,obst_period 才重算
        self._ground_cache = None
        self._inflated_cache = None
        self._clearance_cache = None
        self._obst_stamp = rospy.Time(0)

        self.cur = None
        self.cur_z = self.height_above_ground
        self.goal = None
        self.requested_goal_z = None
        self.base_goal_z = self.height_above_ground
        self.resolved_goal_z = self.height_above_ground
        self.goal_inside_obstacle = False
        self.path = []
        self.idx = 0
        self.last_replan = rospy.Time(0)
        self.last_sent = None
        self.retreating = False
        self.retreat_target = None
        # rospy 的订阅回调与 Timer 回调运行在不同线程。goal_cb/replan/tick 会共同修改
        # goal/path/idx；若 tick 判定到达并把 goal 清空时 replan 正在解包目标，会出现
        # "self.goal=None" 竞态。RLock 允许 tick/goal_cb 在持锁时调用 replan。
        self._state_lock = threading.RLock()
        self._map_lock = threading.RLock()

        # 合并世界坐标对齐：用户在【世界/UGV 帧】点 UAV 目标,但本规划器+EGO 跑在【MAVROS
        # 局部帧】(原点=无人机出生点)。auto_align=true 时用 Gazebo 真值(UAV)−MAVROS(UAV)
        # 测出常量偏移,把世界目标换算到局部帧,否则目标偏一个出生点距离 -> 无人机飞错/绕圈。
        # 单飞(无 UGV)时 false:本就同一帧,不换算。
        self.auto_align = bool(rospy.get_param("~auto_align", False))
        # 世界->局部 的【固定出生点偏移】(已知且精确,比 truth-/odom 实测更稳:/odom 可能卡住
        # 给出错偏移)。合并世界从世界原点起飞时传 (0,0)。
        self.fallback_off = (float(rospy.get_param("~fallback_off_x", 0.0)),
                             float(rospy.get_param("~fallback_off_y", 0.0)))

        self.sub_pub = rospy.Publisher(
            rospy.get_param("~output_topic", "/goal_elevated"), PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher("/global_path", Path, queue_size=1, latch=True)
        rospy.Subscriber(rospy.get_param("~cloud_topic", "/uav0/mapping/points_world"),
                         PointCloud2, self.cloud_cb, queue_size=2)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/mavros/local_position/odom"),
                         Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~goal_topic", "/move_base_simple/goal"),
                         PoseStamped, self.goal_cb, queue_size=1)
        if self.auto_align:
            rospy.logwarn("[global_planner] auto_align 开:世界目标按固定出生点偏移 (%.1f,%.1f) 换算到局部帧",
                          self.fallback_off[0], self.fallback_off[1])
        rospy.Timer(rospy.Duration(0.2), self.tick)
        rospy.Timer(rospy.Duration(3.0), self._status)
        rospy.loginfo("[global_planner] %dx%d res=%.2f 巡航%.1fm | 障碍=高出本地地面>%.1fm(楼/树),"
                      "A* 全局分段 + EGO 局部 rebound(原稳定纯追踪)",
                      self.W, self.W, self.res, self.height_above_ground, self.obstacle_height)

    def w2c(self, x, y):
        return (int((y - self.origin) / self.res), int((x - self.origin) / self.res))

    def c2w(self, rc):
        return (self.origin + (rc[1] + 0.5) * self.res, self.origin + (rc[0] + 0.5) * self.res)

    def in_grid(self, rc):
        return 0 <= rc[0] < self.W and 0 <= rc[1] < self.W

    def cloud_cb(self, msg):
        pts = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                       dtype=np.float64)
        if pts.size == 0:
            return
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        ix = ((x - self.origin) / self.res).astype(np.int64)
        iy = ((y - self.origin) / self.res).astype(np.int64)
        m = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.W)
        ixm, iym, zm = ix[m], iy[m], z[m]
        if zm.size == 0:
            return
        keys = iym * self.W + ixm
        order = np.argsort(keys, kind="mergesort")
        sorted_keys = keys[order]
        unique_keys, starts, counts = np.unique(
            sorted_keys, return_index=True, return_counts=True
        )
        with self._map_lock:
            for key, start, count in zip(unique_keys, starts, counts):
                values = np.sort(zm[order[start:start + count]])
                take = min(self.column_samples_per_frame, values.size)
                if take < values.size:
                    sample_indices = np.linspace(
                        0, values.size - 1, take, dtype=np.int64
                    )
                    values = values[sample_indices]
                flat_key = int(key)
                samples = self._column_samples.get(flat_key)
                if samples is None:
                    samples = collections.deque(
                        maxlen=self.column_sample_capacity
                    )
                    self._column_samples[flat_key] = samples
                samples.extend(float(value) for value in values)
                column = np.fromiter(samples, dtype=np.float64)
                if column.size >= self.column_min_samples:
                    low, high = np.quantile(
                        column,
                        [self.column_low_quantile, self.column_high_quantile],
                    )
                else:
                    low, high = float(np.min(column)), float(np.max(column))
                self.ground.flat[flat_key] = float(low)
                self.height.flat[flat_key] = float(high)

    def odom_cb(self, msg):
        with self._state_lock:
            self.cur = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.cur_z = msg.pose.pose.position.z

    def _off(self):
        """世界->局部 的偏移。用【固定出生点偏移】(已知且精确),不用 truth-/odom 实测值:
        合并世界里 /odom 可能卡住/滞后,实测会得到错的偏移,把全局路径/目标偏到一边。单飞为 0。"""
        if not self.auto_align:
            return (0.0, 0.0)
        return self.fallback_off

    def goal_cb(self, msg):
        with self._state_lock:
            gx, gy = msg.pose.position.x, msg.pose.position.y
            wx, wy = gx, gy
            # 世界/UGV 帧目标 -> MAVROS 局部帧(减去出生点偏移),否则飞错地方/绕圈。
            ox, oy = self._off()
            gx -= ox
            gy -= oy
            self.goal = (gx, gy)
            incoming_z = float(msg.pose.position.z)
            self.requested_goal_z = incoming_z
            if abs(incoming_z) > self.zero_goal_z_epsilon:
                # 3D/Survey 目标有明确 z：原样保留。
                self.base_goal_z = incoming_z
                altitude_source = "MESSAGE_Z"
            elif math.isfinite(self.cur_z) and \
                    abs(self.cur_z) > self.zero_goal_z_epsilon:
                # RViz 2D Goal 的 z=0：保持点击时的实际飞行高度，不爬高。
                self.base_goal_z = self.cur_z
                altitude_source = "CURRENT_Z"
            else:
                self.base_goal_z = self.height_above_ground
                altitude_source = "FALLBACK_Z"
            self.resolved_goal_z = self.base_goal_z
            self.goal_inside_obstacle = False
            self.retreating = False
            self.cmd_z = None          # 新目标:高度限幅从当前高度起步
            self.last_sent = None
            rospy.logwarn("[global_planner] 收到目标 世界(%.1f,%.1f,z=%.2f)->局部(%.1f,%.1f,z=%.2f) "
                          "高度源=%s 偏移=(%.1f,%.1f) 已扫cell=%d",
                          wx, wy, incoming_z, gx, gy, self.base_goal_z,
                          altitude_source, ox, oy, int(np.isfinite(self.height).sum()))
            self.replan()

    def obstacle_and_ground(self):
        """障碍图 + 本地地面高程。
        判据(之前能稳定绕楼的版本)：cell【最高点 - 本地地面 > obstacle_height】= 突起(楼/树)。
        用【最高点】而非"结构最低面",对建图时楼底被遮挡【鲁棒】:无人机在 4m 只扫到楼上半截,
        最高点仍很高 -> 照样判成障碍 -> A* 绕行。再按连通域面积把【大块(楼/墙)】留给全局绕,
        【小块(树)】滤掉交给 EGO 局部穿。缓坡山丘(相对本地地面不突起)本就不算障碍。
        tick 5Hz 频繁调用,这里缓存,真正重算降到 obst_period。"""
        now = rospy.Time.now()
        if self._obst_cache is not None and \
                (now - self._obst_stamp).to_sec() < self.obst_period:
            return self._obst_cache, self._ground_cache

        with self._map_lock:
            ground = self.ground.copy()
            height = self.height.copy()
        # Same-column robust vertical span distinguishes a facade from a
        # sloped/flat surface.  The old neighbourhood minimum joined mapping
        # smear and ordinary grade changes into one giant false obstacle.
        local_ground = ground
        obst = np.zeros((self.W, self.W), dtype=bool)
        valid = np.isfinite(height) & np.isfinite(local_ground)
        obst[valid] = (
            height[valid] - local_ground[valid]
        ) > self.obstacle_height
        if obst.any():
            obst = ndimage.binary_closing(
                obst, structure=np.ones((3, 3), dtype=bool), iterations=1
            )
        # 删掉小连通域(树/细杆)：恢复原稳定版的面积门，避免树林被
        # 连成全局“障碍墙”后 A* 在可达/不可达之间抖动。
        if self.min_obstacle_cells > 1 and obst.any():
            lbl, n = ndimage.label(obst)
            if n > 0:
                sizes = np.bincount(lbl.ravel())
                sizes[0] = 0                       # 背景不计
                small = sizes < self.min_obstacle_cells
                obst[small[lbl]] = False
        # Facade points describe the building boundary; fill its interior so a
        # click inside the building is recognized and routed/elevated safely.
        if obst.any():
            obst = ndimage.binary_fill_holes(obst)
        self._obst_cache = obst
        self._ground_cache = local_ground
        self._inflated_cache, self._clearance_cache = metric_inflation(
            obst, self.res, self.inflate_m
        )
        self._obst_stamp = now
        return obst, local_ground

    def _resolve_goal_altitude(self, obst):
        """固定本次目标高度。

        普通目标始终使用接收时冻结的 base_goal_z，不查沿途地面、不随 carrot
        抬高。只有目标的原始（未膨胀）障碍格为真时，才抬到已观测顶部之上。
        使用未膨胀格避免把“离楼还有1.5m”的正常空地误判为楼内。
        """
        self.resolved_goal_z = self.base_goal_z
        self.goal_inside_obstacle = False
        if self.goal is None:
            return
        rc = self.w2c(*self.goal)
        if not self.in_grid(rc):
            return
        # 高度例外看原始几何，不受“只把大楼留给全局 A*”的面积过滤影响；
        # 因此目标点落在树/细柱上也能被识别，但邻近空地仍不会被障碍膨胀误抬高。
        raw_inside = bool(obst[rc])
        with self._map_lock:
            cell_height = float(self.height[rc])
        if (not raw_inside and self._ground_cache is not None
                and math.isfinite(cell_height)
                and math.isfinite(float(self._ground_cache[rc]))):
            raw_inside = (
                cell_height - float(self._ground_cache[rc])
                > self.obstacle_height
            )
        if not raw_inside:
            return
        self.goal_inside_obstacle = True
        with self._map_lock:
            observed_top = float(self.height[rc])
            if not math.isfinite(observed_top):
                radius = max(1, int(math.ceil(3.0 / self.res)))
                r, c = rc
                nearby = self.height[
                    max(0, r - radius):min(self.W, r + radius + 1),
                    max(0, c - radius):min(self.W, c + radius + 1),
                ]
                finite = nearby[np.isfinite(nearby)]
                if finite.size:
                    observed_top = float(np.max(finite))
        if math.isfinite(observed_top):
            self.resolved_goal_z = max(
                self.base_goal_z,
                observed_top + self.goal_obstacle_clearance,
            )
        rospy.logwarn_throttle(
            2.0,
            "[global_planner] 目标XY位于原始障碍格：z %.2f -> %.2f "
            "(observed_top=%.2f clearance=%.2f)",
            self.base_goal_z, self.resolved_goal_z,
            observed_top, self.goal_obstacle_clearance,
        )

    def ground_z(self, xy, _local_ground=None):
        """该处地面高程:用本格附近 3x3 的【中值】(self.ground 是每格最低点)。
        ★不用大窗口 min 滤波(local_ground):坡上 min 滤波会取到下坡最低点、估计偏低,
        导致子目标高度偏低、无人机贴坡甚至扎进坡里。中值跟随坡面、抗单点噪声。
        未探索 -> 0(起飞地面);不返回 cur_z-AGL(否则起飞锁死在贴地、永不爬升)。"""
        rc = self.w2c(*xy)
        if self.in_grid(rc):
            r, c = rc
            win = self.ground[max(0, r - 1):r + 2, max(0, c - 1):c + 2]
            fin = win[np.isfinite(win)]
            if fin.size > 0:
                return float(np.median(fin))
        return 0.0

    def nearest_free(self, grid, rc, radius=24):
        if self.in_grid(rc) and not grid[rc]:
            return rc
        nearest = nearest_mask_cell(~grid, rc, radius)
        return nearest if nearest is not None else nearest_mask_cell(~grid, rc)

    def _shortcut(self, pts, grid):
        """串拉(string-pulling)：A* 的 8 邻接栅格路是锯齿状的，纯追踪沿它走会弯弯曲曲。
        这里把相邻【可直连(无碰)】的点抽稀成几段长直线，只留转折点 -> EGO 轨迹更直更顺。"""
        if len(pts) <= 2:
            return pts
        # 任一点出界(目标可能在 ±half 外)就不抽稀、直接用原路 —— 否则 w2c 越界、
        # line_clear 里 grid[r,c] 会 IndexError(正越界)或负索引回绕(取错格)。
        cells = [self.w2c(*p) for p in pts]
        if not all(self.in_grid(c) for c in cells):
            return pts
        reduced = shortcut_cells(cells, grid)
        index_by_cell = {cell: index for index, cell in enumerate(cells)}
        return [pts[index_by_cell[cell]] for cell in reduced]

    def _routing_grid(self, obst):
        if self._inflated_cache is None:
            self._inflated_cache, self._clearance_cache = metric_inflation(
                obst, self.res, self.inflate_m
            )
        grid = self._inflated_cache.copy()
        # Clear only inflation (never raw geometry) around the measured UAV
        # position so its first planning state cannot be trapped by map smear.
        if self.cur is not None and self.start_clearance_radius > 0.0:
            rc = self.w2c(*self.cur)
            radius = int(math.ceil(self.start_clearance_radius / self.res))
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    point = (rc[0] + dr, rc[1] + dc)
                    if (self.in_grid(point)
                            and math.hypot(dr, dc) * self.res
                            <= self.start_clearance_radius
                            and not obst[point]):
                        grid[point] = False
        return grid

    def _select_carrot(self, grid, uav_cell):
        """Interpolate lookahead along the safe polyline, never at the goal."""
        if self.cur is None or not self.path:
            return None, None
        remaining = self.lookahead
        anchor = self.cur
        last_clear = None
        exact_index = None
        start_index = min(max(self.idx, 0), len(self.path) - 1)
        raw_grid = getattr(self, "_obst_cache", None)

        for index in range(start_index, len(self.path)):
            target = self.path[index]
            dx = target[0] - anchor[0]
            dy = target[1] - anchor[1]
            segment = math.hypot(dx, dy)
            if segment < 1e-9:
                exact_index = index
                anchor = target
                continue
            if segment <= remaining + 1e-9:
                candidate = target
                candidate_exact = index
            else:
                ratio = remaining / segment
                candidate = (anchor[0] + ratio * dx, anchor[1] + ratio * dy)
                candidate_exact = None

            candidate_cell = self.w2c(*candidate)
            final_target = index == len(self.path) - 1
            clear = (
                self.in_grid(uav_cell)
                and self.in_grid(candidate_cell)
                and line_clear(grid, uav_cell, candidate_cell)
            )
            # The last short segment may lie inside the conservative inflation
            # ring. Admit it only when raw geometry is clear, or when the user
            # explicitly clicked inside an obstacle and its target was raised.
            if (not clear and final_target
                    and (getattr(self, "goal_inside_obstacle", False)
                         or (raw_grid is not None
                             and self.in_grid(candidate_cell)
                             and line_clear(raw_grid, uav_cell, candidate_cell)))):
                clear = True
            if not clear:
                break
            last_clear = candidate
            exact_index = candidate_exact
            if candidate_exact is None:
                break
            remaining -= segment
            if remaining <= 1e-9:
                break
            anchor = target

        return last_clear, exact_index

    def replan(self):
        with self._state_lock:
            self._replan_locked()

    def _replan_locked(self):
        if self.cur is None or self.goal is None:
            return
        self.last_replan = rospy.Time.now()
        obst, _ = self.obstacle_and_ground()
        self._resolve_goal_altitude(obst)
        grid = self._routing_grid(obst)
        sc = self.nearest_free(grid, self.w2c(*self.cur))
        gc = self.nearest_free(grid, self.w2c(*self.goal))
        cells = astar(
            grid, sc, gc, self._clearance_cache, self.res,
            self.inflate_m, 0.12,
        ) if (sc is not None and gc is not None) else None
        if cells:
            cells = shortcut_cells(cells, grid)
            pts = [self.c2w(c) for c in cells]
            if not pts or math.hypot(
                    pts[-1][0] - self.goal[0],
                    pts[-1][1] - self.goal[1]) > 0.25 * self.res:
                pts.append(self.goal)
            else:
                pts[-1] = self.goal
            self.path = self._shortcut(pts, grid)   # 抽稀成几段直线 -> 轨迹更直
            rospy.loginfo_throttle(2.0, "[global_planner] 全局绕楼路 %d 点(原稳定纯追踪)", len(self.path))
        else:
            # Never replace a failed global route with a direct goal through a
            # building.  Hold the last safe command and wait for the next map
            # update; preserving last_sent also avoids resetting EGO every
            # 0.5 s with the same unreachable target.
            self.path = []
            start_blocked = bool(
                self.in_grid(self.w2c(*self.cur)) and grid[self.w2c(*self.cur)]
            )
            goal_blocked = bool(
                self.in_grid(self.w2c(*self.goal)) and grid[self.w2c(*self.goal)]
            )
            rospy.logwarn_throttle(
                3.0,
                "[global_planner] A*暂时无解，保持悬停等待地图更新 "
                "(start_blocked=%s goal_blocked=%s obstacle_cells=%d)",
                start_blocked, goal_blocked, int(np.count_nonzero(obst)),
            )
        self.idx = 0
        self._publish_path()

    def tick(self, _evt):
        with self._state_lock:
            self._tick_locked(_evt)

    def _tick_locked(self, _evt):
        if self.cur is None:
            return
        # ★到达最终目标 → 清空目标&路径、停发子目标,悬停等下一个目标。
        #   配合 _publish_subgoal 的 reach 守卫,杜绝给 EGO 喂 start==goal —— EGO 的
        #   REPLAN_TRAJ 收到退化目标会 final_plan_success=0 无限死循环且【无法自恢复】
        #   (那个状态没有"够近就放弃→WAIT_TARGET"的出口),只能靠不发退化目标来避免。
        altitude_reached = (
            not self.goal_inside_obstacle
            or abs(self.cur_z - self.resolved_goal_z) <= self.goal_z_reach_tolerance
        )
        if self.goal is not None and altitude_reached and \
                math.hypot(self.cur[0] - self.goal[0], self.cur[1] - self.goal[1]) < self.reach:
            rospy.loginfo_throttle(3.0, "[global_planner] 已到达目标(%.1f,%.1f),悬停等待新目标",
                                   self.goal[0], self.goal[1])
            self.goal = None
            self.path = []
            self.idx = 0
            self.last_sent = None
            self.cmd_z = None          # 重置高度限幅,下个目标从当前高度起步
            self.goal_inside_obstacle = False
            return
        obst, _ = self.obstacle_and_ground()
        # 1) 反应式贴楼退后——默认关闭(enable_retreat=false)，局部避障交给 EGO。
        #    开着会"前进→离障碍0.5m→退后2m→再前进"来回振荡(正是用户看到的现象)。
        if self.enable_retreat and obst.any():
            dist = ndimage.distance_transform_edt(~obst)
            rc = self.w2c(*self.cur)
            d = dist[rc] * self.res if self.in_grid(rc) else 1e9
            if self.retreating:
                if d > self.safe_dist + 1.0:
                    self.retreating = False
                    self.replan()
                else:
                    self._publish_subgoal(self.retreat_target, self.resolved_goal_z)
                    return
            elif d < self.safe_dist:
                r, c = rc
                dx = dist[r, min(c + 1, self.W - 1)] - dist[r, max(c - 1, 0)]
                dy = dist[min(r + 1, self.W - 1), c] - dist[max(r - 1, 0), c]
                nrm = math.hypot(dx, dy)
                if nrm > 1e-6:
                    self.retreat_target = (self.cur[0] + self.retreat_dist * dx / nrm,
                                           self.cur[1] + self.retreat_dist * dy / nrm)
                    self.retreating = True
                    self._publish_subgoal(self.retreat_target, self.resolved_goal_z)
                    rospy.logwarn_throttle(1.0, "[global_planner] 离楼太近(%.1fm)，退后再绕", d)
                    return

        if self.goal is None:
            return
        if not self.path:
            if (rospy.Time.now() - self.last_replan).to_sec() > self.min_replan:
                self.replan()
            if not self.path:
                return
        grid = self._routing_grid(obst)
        uav_cell = self.w2c(*self.cur)
        while self.idx < len(self.path) - 1 and \
                math.hypot(self.cur[0] - self.path[self.idx][0],
                           self.cur[1] - self.path[self.idx][1]) < self.reach:
            self.idx += 1
        # Interpolate lookahead along the globally safe polyline.  The former
        # vertex-only fallback aimed directly at the final goal whenever the
        # next corner was farther than lookahead, cutting through buildings.
        carrot, _exact_index = self._select_carrot(grid, uav_cell)
        if carrot is None:
            if (rospy.Time.now() - self.last_replan).to_sec() > self.min_replan:
                self.replan()
            return
        # 整条路径使用冻结的目标高度，不再因沿途地面/障碍而主动爬高。
        # 只有 _resolve_goal_altitude 确认“最终目标本身在障碍格内”才会抬高。
        z_target = self.resolved_goal_z
        # 显式 3D 目标或楼内目标可能需要改高，仍做每 tick 限幅。
        if self.cmd_z is None:
            self.cmd_z = self.cur_z
        self.cmd_z += max(-self.max_z_step, min(self.max_z_step, z_target - self.cmd_z))
        self._publish_subgoal(carrot, self.cmd_z)

    def _publish_subgoal(self, xy, z):
        # z 已由收到目标时冻结，这里不作地形叠加或最低高度夹取。
        # ★防卡死(关键):绝不发"≈无人机当前位置"的子目标。EGO 一旦收到 start==goal,会在
        #   REPLAN_TRAJ 里 final_plan_success=0 无限死循环、且没有自恢复出口。萝卜点落进
        #   0.8m 内就【不发】,让 EGO 用上一个有效目标把最后这点飞完、轨迹自然结束→悬停。
        if self.cur is not None and \
                math.hypot(xy[0] - self.cur[0], xy[1] - self.cur[1]) < 0.8:
            return
        # 子目标变化小于 1m 不重发：避免萝卜点微抖一直让 EGO 重规划，轨迹更稳不弯弯曲曲。
        if self.last_sent is not None and \
                math.hypot(xy[0] - self.last_sent[0], xy[1] - self.last_sent[1]) < 1.0 and \
                abs(z - self.last_sent[2]) < 0.5:
            return
        self.last_sent = (xy[0], xy[1], z)
        m = PoseStamped()
        m.header = Header(frame_id="map", stamp=rospy.Time.now())
        m.pose.position.x = xy[0]
        m.pose.position.y = xy[1]
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        self.sub_pub.publish(m)

    def _publish_path(self):
        # /global_path 仅用于 RViz 显示：把局部帧路点 +偏移 显示到世界帧,和对齐后的点云/
        # UGV/高程图重合(规划/喂 EGO 仍用局部帧的 /goal_elevated,不受影响)。
        ox, oy = self._off()
        path = Path()
        path.header = Header(frame_id="map", stamp=rospy.Time.now())
        for xy in self.path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = xy[0] + ox
            ps.pose.position.y = xy[1] + oy
            ps.pose.position.z = self.resolved_goal_z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _status(self, _evt):
        with self._state_lock:
            rospy.loginfo("[global_planner] 已扫cell=%d odom=%s 目标=%s 路点=%d",
                          int(np.isfinite(self.height).sum()),
                          "有" if self.cur is not None else "无",
                          "有" if self.goal is not None else "无", len(self.path))


def main():
    rospy.init_node("global_path_planner")
    GlobalPlanner()
    rospy.spin()


if __name__ == "__main__":
    main()
