#!/usr/bin/env python3
"""从全局地形点云中提取无人机周围的局部地形/障碍点云，直接供给 EGO-Planner。

绕过 pcl_render_node (GPU CUDA depth render)，避免 CUDA 兼容性问题。
始终采用最新的全局点云，并按 XY 栅格剔除明显低于表面的地下填充点，
避免旧节点或旧 latched 消息把地形下方堆成竖直障碍柱。
"""
import math
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def main():
    rospy.init_node('local_terrain_cloud')

    sensing_horizon = rospy.get_param('~sensing_horizon', 30.0)
    publish_rate = rospy.get_param('~publish_rate', 10.0)
    xy_cell = rospy.get_param('~xy_cell', 0.5)
    top_band = rospy.get_param('~top_band', 6.0)
    subsurface_depth = rospy.get_param('~subsurface_depth', 6.0)
    fill_step = rospy.get_param('~fill_step', 0.5)

    global_points = None    # Nx3 numpy array (float32)
    drone_xyz = None        # (x, y, z)

    def remove_subterranean_fill(points):
        top_z = {}
        for x, y, z in points:
            key = (int(math.floor(x / xy_cell)), int(math.floor(y / xy_cell)))
            if key not in top_z or z > top_z[key]:
                top_z[key] = float(z)

        filtered = []
        for x, y, z in points:
            key = (int(math.floor(x / xy_cell)), int(math.floor(y / xy_cell)))
            if z >= top_z[key] - top_band:
                filtered.append([x, y, z])
        return np.array(filtered, dtype=np.float32)

    def global_cb(msg):
        nonlocal global_points
        pts = []
        for pt in pc2.read_points(msg, field_names=("x", "y", "z"),
                                  skip_nans=True):
            pts.append([pt[0], pt[1], pt[2]])

        if not pts:
            return

        raw = np.array(pts, dtype=np.float32)
        global_points = remove_subterranean_fill(raw)
        rospy.loginfo_throttle(
            5,
            "[LocalTerrain] Updated global cloud: %d -> %d points",
            len(raw), len(global_points),
        )

    def odom_cb(msg):
        nonlocal drone_xyz
        p = msg.pose.position
        drone_xyz = (p.x, p.y, p.z)

    rospy.Subscriber('/map_generator/global_cloud', PointCloud2, global_cb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, odom_cb)

    pub = rospy.Publisher('/local_terrain_cloud/filtered', PointCloud2, queue_size=1)
    rate = rospy.Rate(publish_rate)

    rospy.loginfo("[LocalTerrain] Waiting for global cloud and odom...")
    while not rospy.is_shutdown() and (global_points is None or drone_xyz is None):
        rate.sleep()

    rospy.loginfo("[LocalTerrain] Running: horizon=%.1fm, rate=%.1fHz",
                  sensing_horizon, publish_rate)

    while not rospy.is_shutdown():
        dx = global_points[:, 0] - drone_xyz[0]
        dy = global_points[:, 1] - drone_xyz[1]
        dist_xy = np.sqrt(dx * dx + dy * dy)

        local = global_points[dist_xy < sensing_horizon]

        surface = {}
        for x, y, z in local:
            key = (int(math.floor(x / xy_cell)), int(math.floor(y / xy_cell)))
            if key not in surface or z > surface[key][2]:
                surface[key] = (float(x), float(y), float(z))

        thick = []
        for x, y, z in surface.values():
            zz = z
            z_min = z - subsurface_depth
            while zz >= z_min:
                thick.append([x, y, zz])
                zz -= fill_step

        local = np.array(thick, dtype=np.float32) if thick else np.empty((0, 3), dtype=np.float32)

        header = Header(stamp=rospy.Time.now(), frame_id="map")
        cloud_msg = pc2.create_cloud_xyz32(header, local.tolist())
        pub.publish(cloud_msg)

        rospy.loginfo_throttle(5, "[LocalTerrain] Published %d local thickened points", len(local))
        rate.sleep()


if __name__ == '__main__':
    main()
