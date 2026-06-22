#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""global_path_planner.py — UAV 全局规划前端(配合 EGO 局部避障)。

EGO 是局部 rebound 规划器，绕不过大体量建筑。本节点在它前面加一层全局规划：
用 360° 雷达累积一张全局地图，A* 搜一条【绕开大楼】的路，纯追踪喂给 EGO 子目标，
EGO 负责每一段的局部避障(穿树林/细障碍/动态)。—— 既能绕楼、又能穿树林。

三个关键：
  ① 障碍按【高出本地地面】判，不是绝对高度 → 缓坡山丘(地面整体抬高)不算障碍，能飞越/爬坡。
  ② 只把【大体量障碍(楼/墙)】留给全局绕行；【小障碍(树)】按连通域面积滤掉、不全局绕，
     交给 EGO 局部避障穿过去 —— 否则树林里 A* 被一堆"树墙"堵成无解、无人机原地不动不规划。
  ③ 子目标高度 = 本地地面 + height_above_ground → 无人机【贴着地形爬坡】，扫得到坡、建得了坡的图。
"""

import heapq
import math

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from scipy import ndimage
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def astar(grid, start, goal):
    """8 邻接 A*。grid: bool 障碍图。start/goal: (row,col)。返回 cell 列表或 None。"""
    h, w = grid.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w):
        return None
    if grid[goal] or grid[start]:
        return None
    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    openq = [(0.0, start)]
    came = {}
    g = {start: 0.0}

    def hh(a):
        return math.hypot(a[0] - goal[0], a[1] - goal[1])

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
            ng = g[cur] + cost
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
        if grid[r, c]:
            return False
        if r == r1 and c == c1:
            return True
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc


class GlobalPlanner(object):
    def __init__(self):
        self.res = float(rospy.get_param("~resolution", 0.5))
        self.half = float(rospy.get_param("~half_extent", 100.0))
        # 高出"本地地面"多少算障碍(楼/树/悬崖)。缓坡山丘地面整体抬高、相对本地地面不突起，不算障碍。
        self.obstacle_height = float(rospy.get_param("~obstacle_height", 1.5))
        # 只全局绕【大体量障碍(楼)】：连通域占地 >= 此面积(m²)才当楼绕行；更小的(树)滤掉，
        # 交给 EGO 局部避障穿过去。设 0 关闭(所有突起都全局绕，树林里会被堵死)。
        self.min_obstacle_area = float(rospy.get_param("~min_obstacle_area", 10.0))
        self.ground_window = float(rospy.get_param("~ground_window", 2.0))   # 本地地面估计窗口(m)
        self.height_above_ground = float(rospy.get_param("~goal_z", 4.0))    # 贴地形飞这么高 → 爬坡
        self.inflate_m = float(rospy.get_param("~inflate", 1.5))
        self.reach = float(rospy.get_param("~reach_radius", 1.5))
        self.lookahead = float(rospy.get_param("~lookahead", 7.0))
        self.min_replan = float(rospy.get_param("~min_replan_interval", 0.5))
        self.safe_dist = float(rospy.get_param("~safe_dist", 0.8))
        self.retreat_dist = float(rospy.get_param("~retreat_dist", 2.0))
        # 反应式"贴楼退后"默认【关闭】：局部避障交给 EGO(用户要求"用ego避障,不要反应式")。
        # 开着会在建图新障碍刚出现/EGO抄近道贴墙时触发退后→又前进→来回振荡。
        self.enable_retreat = bool(rospy.get_param("~enable_retreat", False))
        # 子目标最低高度(m)：地面估计被低噪点拉低时,子目标高度别跟着沉到地里把无人机往地上压。
        self.min_z = float(rospy.get_param("~min_subgoal_z", 2.0))

        self.W = max(2, int(2 * self.half / self.res))
        self.origin = -self.half
        self.gwin = max(1, int(round(self.ground_window / self.res)) | 1)
        self.inflate_cells = max(1, int(round(self.inflate_m / self.res)))
        self.min_obstacle_cells = max(0, int(round(self.min_obstacle_area / (self.res * self.res))))
        self.ground = np.full((self.W, self.W), np.inf)    # 每 cell 最低点(地面候选)
        self.height = np.full((self.W, self.W), -np.inf)   # 每 cell 最高点

        self.cur = None
        self.cur_z = self.height_above_ground
        self.goal = None
        self.path = []
        self.idx = 0
        self.last_replan = rospy.Time(0)
        self.last_sent = None
        self.retreating = False
        self.retreat_target = None

        self.sub_pub = rospy.Publisher(
            rospy.get_param("~output_topic", "/goal_elevated"), PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher("/global_path", Path, queue_size=1, latch=True)
        rospy.Subscriber(rospy.get_param("~cloud_topic", "/uav0/mapping/points_world"),
                         PointCloud2, self.cloud_cb, queue_size=2)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/mavros/local_position/odom"),
                         Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~goal_topic", "/move_base_simple/goal"),
                         PoseStamped, self.goal_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.tick)
        rospy.Timer(rospy.Duration(3.0), self._status)
        rospy.loginfo("[global_planner] %dx%d res=%.2f obstacle=高出本地地面%.1fm 贴地飞%.1fm",
                      self.W, self.W, self.res, self.obstacle_height, self.height_above_ground)

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
        np.minimum.at(self.ground, (iy[m], ix[m]), z[m])   # 地面 = 累积最低
        np.maximum.at(self.height, (iy[m], ix[m]), z[m])   # 最高点

    def odom_cb(self, msg):
        self.cur = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.cur_z = msg.pose.pose.position.z

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.retreating = False
        rospy.logwarn("[global_planner] 收到目标(%.1f,%.1f) 已扫cell=%d odom=%s -> 规划绕楼(缓坡不绕)",
                      self.goal[0], self.goal[1], int(np.isfinite(self.height).sum()),
                      "有" if self.cur is not None else "无")
        self.replan()

    def obstacle_and_ground(self):
        """障碍图 + 本地地面高程。
        高出本地地面>obstacle_height 的算突起；再按连通域面积把【大块(楼/墙)】留下给全局
        绕行，【小块(树)】滤掉交给 EGO 局部避障穿过去。缓坡山丘(相对地面不突起)本就不算障碍。"""
        local_ground = ndimage.minimum_filter(self.ground, size=self.gwin)
        obst = np.zeros((self.W, self.W), dtype=bool)
        valid = np.isfinite(self.height) & np.isfinite(local_ground)
        obst[valid] = (self.height[valid] - local_ground[valid]) > self.obstacle_height
        # 删掉小连通域(树/细杆)：只保留占地够大的楼/墙做全局绕行，其余让 EGO 局部穿。
        if self.min_obstacle_cells > 1 and obst.any():
            lbl, n = ndimage.label(obst)
            if n > 0:
                sizes = np.bincount(lbl.ravel())
                sizes[0] = 0                       # 背景不计
                small = sizes < self.min_obstacle_cells
                obst[small[lbl]] = False           # 小块(树)从全局障碍图移除
        return obst, local_ground

    def ground_z(self, xy, local_ground):
        rc = self.w2c(*xy)
        if self.in_grid(rc) and np.isfinite(local_ground[rc]):
            return float(local_ground[rc])
        # 地面未知 -> 按起飞地面(0)，子目标抬到巡航高度 height_above_ground。
        # 不能返回 cur_z-AGL：那会让 z 恒等于 cur_z，起飞没爬上去就把高度锁死在贴地、永远不爬。
        return 0.0

    def nearest_free(self, grid, rc, radius=12):
        if self.in_grid(rc) and not grid[rc]:
            return rc
        for r in range(1, radius):
            for dr in range(-r, r + 1):
                for dc in range(-r, r + 1):
                    p = (rc[0] + dr, rc[1] + dc)
                    if self.in_grid(p) and not grid[p]:
                        return p
        return None

    def _shortcut(self, pts, grid):
        """串拉(string-pulling)：A* 的 8 邻接栅格路是锯齿状的，纯追踪沿它走会弯弯曲曲。
        这里把相邻【可直连(无碰)】的点抽稀成几段长直线，只留转折点 -> EGO 轨迹更直更顺。"""
        if len(pts) <= 2:
            return pts
        out = [pts[0]]
        i, n = 0, len(pts)
        while i < n - 1:
            j = n - 1
            while j > i + 1 and not line_clear(grid, self.w2c(*pts[i]), self.w2c(*pts[j])):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def replan(self):
        if self.cur is None or self.goal is None:
            return
        self.last_replan = rospy.Time.now()
        self.last_sent = None
        obst, _ = self.obstacle_and_ground()
        grid = ndimage.binary_dilation(obst, iterations=self.inflate_cells)
        sc = self.nearest_free(grid, self.w2c(*self.cur))
        gc = self.nearest_free(grid, self.w2c(*self.goal))
        cells = astar(grid, sc, gc) if (sc is not None and gc is not None) else None
        if cells:
            pts = [self.c2w(c) for c in cells]
            pts[-1] = self.goal
            self.path = self._shortcut(pts, grid)   # 抽稀成几段直线 -> 轨迹更直不弯弯曲曲
            rospy.loginfo_throttle(2.0, "[global_planner] 绕楼路 %d 点(已串拉抽稀,缓坡直接飞越)", len(self.path))
        else:
            self.path = [self.goal]
            rospy.logwarn_throttle(3.0, "[global_planner] A* 未找到路(未探索/被堵)，直发目标")
        self.idx = 0
        self._publish_path()

    def tick(self, _evt):
        if self.cur is None:
            return
        obst, local_ground = self.obstacle_and_ground()
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
                    self._publish_subgoal(self.retreat_target, self.ground_z(self.retreat_target, local_ground) + self.height_above_ground)
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
                    self._publish_subgoal(self.retreat_target, self.ground_z(self.retreat_target, local_ground) + self.height_above_ground)
                    rospy.logwarn_throttle(1.0, "[global_planner] 离楼太近(%.1fm)，退后再绕", d)
                    return

        if self.goal is None:
            return
        if not self.path:
            self.replan()
            if not self.path:
                return
        grid = ndimage.binary_dilation(obst, iterations=self.inflate_cells)
        uav_cell = self.w2c(*self.cur)
        while self.idx < len(self.path) - 1 and \
                math.hypot(self.cur[0] - self.path[self.idx][0],
                           self.cur[1] - self.path[self.idx][1]) < self.reach:
            self.idx += 1
        # 纯追踪萝卜点：从当前位置出发，沿全局路找直线无碰且前瞻内的最远点
        best = -1
        for j in range(self.idx, len(self.path)):
            if math.hypot(self.cur[0] - self.path[j][0],
                          self.cur[1] - self.path[j][1]) > self.lookahead:
                break
            jc = self.w2c(*self.path[j])
            if self.in_grid(uav_cell) and self.in_grid(jc) and line_clear(grid, uav_cell, jc):
                best = j
        if best < 0:
            if (rospy.Time.now() - self.last_replan).to_sec() > self.min_replan:
                self.replan()   # 可能重建 path/idx，必须在取 best 之前，否则索引越界
            best = min(self.idx, len(self.path) - 1)   # 没干净萝卜点也至少朝下一个路点
        if not self.path:
            return
        best = min(best, len(self.path) - 1)
        carrot = self.path[best]
        # 防退化(关键)：萝卜点≈当前位置但还没到最终目标 → 朝目标方向迈 lookahead，给 EGO 一个
        # 真目标。否则 EGO 收到"原地"目标(start≈goal)就一直 Close to goal / plan_success=0 卡死。
        if (math.hypot(self.cur[0] - carrot[0], self.cur[1] - carrot[1]) < self.reach and
                math.hypot(self.cur[0] - self.goal[0], self.cur[1] - self.goal[1]) > self.reach):
            dx, dy = self.goal[0] - self.cur[0], self.goal[1] - self.cur[1]
            d = math.hypot(dx, dy)
            carrot = (self.cur[0] + min(self.lookahead, d) * dx / d,
                      self.cur[1] + min(self.lookahead, d) * dy / d)
        # 子目标高度 = 本地地面 + 巡航高度(地面未知则按起飞地面=巡航高度) → 爬到飞行高度/贴地形
        z = self.ground_z(carrot, local_ground) + self.height_above_ground
        self._publish_subgoal(carrot, z)

    def _publish_subgoal(self, xy, z):
        z = max(z, self.min_z)   # 高度下限：地面噪点拉低估计时也不把无人机往地上压
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
        path = Path()
        path.header = Header(frame_id="map", stamp=rospy.Time.now())
        for xy in self.path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = xy[0]
            ps.pose.position.y = xy[1]
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _status(self, _evt):
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
