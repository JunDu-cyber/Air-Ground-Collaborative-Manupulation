#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""terrain_costmap_bridge.py

GridMap(高程+坡度) -> nav_msgs/OccupancyGrid，供 move_base StaticLayer 使用
（方案①：把 UAV 高程图接入 UGV 导航）。

为什么不用 grid_map_costmap_2d 插件：本机只装了 grid_map_core/ros/filters，没有可
直接挂进 move_base 的 grid_map costmap layer 插件；grid_map_costmap_2d 本身是个 C++
转换库，不是 pluginlib 层。这里用最轻依赖(仅 grid_map_msgs)把 slope 层阈值化成
OccupancyGrid，再用 move_base 自带 StaticLayer 吃进去，不引入任何重型包。

规则：slope > slope_max -> 致命(100)；无效/空洞(NaN) -> 未知(-1)；其余 -> 自由(0)。
"""

import numpy as np
import rospy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid


def gridmap_layer_to_occupancy(gm, layer, slope_max, unknown_as_free=False, stamp=None):
    """把 GridMap 的某一层按阈值转成 nav_msgs/OccupancyGrid。纯函数，便于单测。

    stamp 留空则用 rospy.Time.now()；单测时可注入固定 stamp 避免依赖 roscore。
    返回 (OccupancyGrid, None) 成功；(None, reason) 失败。"""
    if layer not in gm.layers:
        return None, "layer '%s' not in %s" % (layer, list(gm.layers))

    data = gm.data[gm.layers.index(layer)]
    dims = data.layout.dim
    if len(dims) < 2:
        return None, "unexpected MultiArray layout (need 2 dims)"
    rows = dims[0].size       # grid_map: dim[0] -> +x (columns of cells along x)
    cols = dims[1].size       # dim[1] -> +y
    if rows * cols == 0 or len(data.data) < rows * cols:
        return None, "empty/short data %dx%d vs %d" % (rows, cols, len(data.data))

    m = np.array(data.data[:rows * cols], dtype=np.float32).reshape(rows, cols)

    info = gm.info
    og = OccupancyGrid()
    og.header.frame_id = info.header.frame_id      # = "map"
    og.header.stamp = stamp if stamp is not None else rospy.Time.now()
    og.info.resolution = info.resolution
    og.info.width = cols
    og.info.height = rows
    # grid_map 中心在 info.pose.position；OccupancyGrid 原点在左下角。
    og.info.origin.position.x = info.pose.position.x - info.length_x / 2.0
    og.info.origin.position.y = info.pose.position.y - info.length_y / 2.0
    og.info.origin.orientation.w = 1.0

    fill = 0 if unknown_as_free else -1
    out = np.full(m.shape, fill, dtype=np.int8)
    valid = ~np.isnan(m)
    out[valid & (m <= slope_max)] = 0
    out[valid & (m > slope_max)] = 100
    # grid_map 行列方向与 OccupancyGrid 相反，翻转对齐（上线时按 RViz 叠加校准一次）。
    out = np.flipud(np.fliplr(out))
    og.data = out.flatten(order="C").astype(np.int8).tolist()
    return og, None


class TerrainCostmapBridge(object):
    def __init__(self):
        self.layer = rospy.get_param("~slope_layer", "slope")
        self.slope_max = float(rospy.get_param("~slope_max", 0.45))   # rad, ~26deg
        self.unknown_as_free = bool(rospy.get_param("~unknown_as_free", False))
        self.in_topic = rospy.get_param(
            "~grid_map_topic", "/elevation_mapping/elevation_map_postprocessed")
        self.pub = rospy.Publisher("/terrain_costmap", OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber(self.in_topic, GridMap, self._cb, queue_size=1)
        rospy.loginfo("[terrain_bridge] in=%s layer=%s slope_max=%.2frad",
                      self.in_topic, self.layer, self.slope_max)

    def _cb(self, gm):
        og, reason = gridmap_layer_to_occupancy(
            gm, self.layer, self.slope_max, self.unknown_as_free)
        if og is None:
            rospy.logwarn_throttle(5.0, "[terrain_bridge] skip: %s", reason)
            return
        self.pub.publish(og)


def main():
    rospy.init_node("terrain_costmap_bridge")
    TerrainCostmapBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
