#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_truth_cloud_projector.py — 用 Gazebo 真值位姿直接投影 UAV 激光到世界系。

为什么不用 TF：合并世界重负载下 RTF<<1 -> sim 时间抖/跳(Time jump) -> TF 时间戳重复被丢
-> map->uav_base_link 冻住 -> 每帧激光投到同一处(点云只堆一个圆)。本节点【完全绕开 TF、
/odom、时间戳】：直接订阅 /gazebo/model_states 拿无人机【当前真值位姿】,用它把【激光帧】里的
点转到世界系。真值一直在动 -> 点云必然随无人机铺开,锁步/时间抖动都影响不了它。

输入：过滤后的激光点云(激光frame, 同 uav_pointcloud_to_world 的 input)。
输出：points_world。offset 减去出生点 xy(z 保留)-> 落在 MAVROS 局部系,与下游 relay/地形
      投影/global_planner 一致(它们再用固定偏移搬到世界帧显示)。offset=0 则直接输出世界系。
"""

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def quat_to_rot(x, y, z, w):
    """四元数 -> 3x3 旋转矩阵。"""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


class TruthCloudProjector(object):
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "iris_depth_camera")
        self.input_topic = rospy.get_param("~input_topic", "/uav0/mapping/velodyne_points_sensor")
        self.output_topic = rospy.get_param("~output_topic", "/uav0/mapping/points_world")
        self.world_frame = rospy.get_param("~world_frame", "map")
        # 激光相对机体的安装偏移(SDF: velodyne_link 在 body 上方 0.06m, 无旋转)。
        self.mount = np.array([
            float(rospy.get_param("~mount_x", 0.0)),
            float(rospy.get_param("~mount_y", 0.0)),
            float(rospy.get_param("~mount_z", 0.06)),
        ])
        # 输出帧偏移：减去出生点 xy -> MAVROS 局部系(z 保留真值)。默认 0 -> 世界系。
        self.off = np.array([
            float(rospy.get_param("~offset_x", 0.0)),
            float(rospy.get_param("~offset_y", 0.0)),
            float(rospy.get_param("~offset_z", 0.0)),
        ])
        self.max_points = int(rospy.get_param("~max_points", 0))   # >0: 随机降采样上限,控负载

        # (位置(3,), 旋转(3,3)) 一个元组,_truth_cb 一次写、_cloud_cb 一次读 -> 避免跨线程
        # 读到位置/姿态来自不同时刻的撕裂值(否则单帧投影会有 ~分米级抖动)。
        # ★只用【最新】真值位姿投影,【刻意不看 cloud 时间戳】:本节点存在的全部意义就是绕开
        #   这个重负载合并世界里不可靠的 /clock·时间戳(stamp 卡顿/抖动会让按戳查到的位姿严重错位
        #   -> 整片点云"不知道在哪"、全局图建错、无人机不再绕楼)。最新真值一直在动、永远对得上当前
        #   扫描,只有"最新与本帧扫描时刻"之间的微小延迟带来轻微拖影 -> 靠【慢飞】压住,不靠对时间戳。
        self._pose = None
        self._n = 0
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._truth_cb, queue_size=1)
        rospy.Subscriber(self.input_topic, PointCloud2, self._cloud_cb, queue_size=1)
        rospy.Timer(rospy.Duration(3.0), self._status)
        rospy.logwarn("[TruthCloudProjector] 真值投影: %s + model_states(%s) -> %s | mount=%s off=%s",
                      self.input_topic, self.model_name, self.output_topic,
                      self.mount.tolist(), self.off.tolist())

    def _truth_cb(self, msg):
        try:
            i = msg.name.index(self.model_name)
        except ValueError:
            return
        p = msg.pose[i].position
        o = msg.pose[i].orientation
        self._pose = (np.array([p.x, p.y, p.z]),
                      quat_to_rot(o.x, o.y, o.z, o.w))

    def _cloud_cb(self, msg):
        pose = self._pose          # 一次读取(原子):用最新真值位姿,不查 cloud 时间戳(见上注释)
        if pose is None:
            rospy.logwarn_throttle(5.0, "[TruthCloudProjector] 还没收到 model_states(%s),等真值...",
                                   self.model_name)
            return
        P, R = pose
        pts = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                       dtype=np.float64)
        if pts.shape[0] == 0:
            return
        if self.max_points > 0 and pts.shape[0] > self.max_points:
            idx = np.random.choice(pts.shape[0], self.max_points, replace=False)
            pts = pts[idx]
        # 世界系 = 真值位置 + 真值姿态 *(安装偏移 + 激光帧点);再减输出偏移落到目标帧。
        world = (P + (pts + self.mount) @ R.T - self.off).astype(np.float32)
        # 直接用 numpy 缓冲构造 PointCloud2(O(N) 内存拷贝),不走 create_cloud 的逐点 Python
        # 打包(热路径慢)。当前时间戳,不沿用输入里可能卡住的(/odom重打的)戳。
        cloud = PointCloud2()
        cloud.header = Header(frame_id=self.world_frame, stamp=rospy.Time.now())
        cloud.height = 1
        cloud.width = world.shape[0]
        cloud.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1)]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * world.shape[0]
        cloud.is_dense = True
        cloud.data = np.ascontiguousarray(world).tobytes()
        self.pub.publish(cloud)
        self._n += 1

    def _status(self, _evt):
        pose = self._pose
        if pose is not None:
            P = pose[0]
            rospy.loginfo_throttle(3.0, "[TruthCloudProjector] UAV真值=(%.1f,%.1f,%.1f) 已发布=%d帧"
                                   " (这个坐标一直变=点云会随飞行铺开)",
                                   P[0], P[1], P[2], self._n)


def main():
    rospy.init_node("uav_truth_cloud_projector")
    TruthCloudProjector()
    rospy.spin()


if __name__ == "__main__":
    main()
