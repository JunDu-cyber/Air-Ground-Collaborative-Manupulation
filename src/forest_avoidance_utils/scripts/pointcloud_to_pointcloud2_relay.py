#!/usr/bin/env python3
"""Relay Gazebo block-laser PointCloud output as filtered PointCloud2."""

import math
import time
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2


def bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class PointCloud2Relay(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/uav0/velodyne_points_raw")
        self.output_topic = rospy.get_param("~output_topic", "/uav0/velodyne_points")
        self.frame_id = rospy.get_param("~frame_id", "")
        self.output_frame_id = rospy.get_param("~output_frame_id", self.frame_id)
        self.publish_in_world_frame = bool_param("~publish_in_world_frame", False)
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.stamp_with_odom = bool_param("~stamp_with_odom", False)
        self.include_intensity = rospy.get_param("~include_intensity", True)
        self.min_range = float(rospy.get_param("~min_range", 0.0))
        self.max_range = float(rospy.get_param("~max_range", 0.0))
        self.sensor_max_range = float(rospy.get_param("~sensor_max_range", 0.0))
        self.max_range_margin = float(rospy.get_param("~max_range_margin", 0.0))
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.0))
        self.local_min_z = float(rospy.get_param("~local_min_z", -999.0))
        self.local_max_z = float(rospy.get_param("~local_max_z", 999.0))
        self.neighbor_voxel_size = float(rospy.get_param("~neighbor_voxel_size", 0.0))
        self.min_neighbors = int(rospy.get_param("~min_neighbors", 1))
        self.min_publish_interval = float(rospy.get_param("~min_publish_interval", 0.0))
        self.last_publish_time = 0.0
        self.imu_topic = rospy.get_param("~imu_topic", "")
        self.max_angular_velocity = float(rospy.get_param("~max_angular_velocity", 0.0))
        self.max_imu_age = float(rospy.get_param("~max_imu_age", 0.25))
        self.last_imu = None
        self.odom_topic = rospy.get_param("~odom_topic", "")
        self.max_linear_velocity = float(rospy.get_param("~max_linear_velocity", 0.0))
        self.max_odom_angular_velocity = float(
            rospy.get_param("~max_odom_angular_velocity", self.max_angular_velocity))
        self.max_odom_age = float(rospy.get_param("~max_odom_age", 0.25))
        self.ground_filter_enabled = bool_param("~ground_filter_enabled", False)
        self.ground_z = float(rospy.get_param("~ground_z", 0.0))
        self.ground_keep_above = float(rospy.get_param("~ground_keep_above", 0.35))
        self.ground_column_filter_enabled = bool_param("~ground_column_filter_enabled", True)
        self.ground_column_size = float(rospy.get_param("~ground_column_size", 0.08))
        self.ground_column_min_height = float(rospy.get_param("~ground_column_min_height", 0.20))
        self.ground_column_support_radius = int(rospy.get_param("~ground_column_support_radius", 0))
        self.self_filter_enabled = bool_param("~self_filter_enabled", True)
        self.self_filter_body_radius = float(rospy.get_param("~self_filter_body_radius", 0.28))
        self.self_filter_body_z_min = float(rospy.get_param("~self_filter_body_z_min", -0.16))
        self.self_filter_body_z_max = float(rospy.get_param("~self_filter_body_z_max", 0.16))
        self.self_filter_rotor_radius = float(rospy.get_param("~self_filter_rotor_radius", 0.17))
        self.self_filter_rotor_z_min = float(rospy.get_param("~self_filter_rotor_z_min", -0.14))
        self.self_filter_rotor_z_max = float(rospy.get_param("~self_filter_rotor_z_max", 0.06))
        self.lidar_mount_x = float(rospy.get_param("~lidar_mount_x", 0.0))
        self.lidar_mount_y = float(rospy.get_param("~lidar_mount_y", 0.0))
        self.lidar_mount_z = float(rospy.get_param("~lidar_mount_z", 0.06))
        self.last_odom = None
        self.input_frames = 0
        self.published_frames = 0
        self.input_points = 0
        self.published_points = 0
        self.dropped_no_imu = 0
        self.dropped_old_imu = 0
        self.dropped_imu_angular = 0
        self.dropped_no_odom = 0
        self.dropped_old_odom = 0
        self.dropped_linear = 0
        self.dropped_odom_angular = 0
        self.dropped_ground_points = 0
        self.kept_supported_ground_points = 0
        self.dropped_self_points = 0
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        rospy.Subscriber(self.input_topic, PointCloud, self.cloud_cb, queue_size=2)
        if self.imu_topic:
            rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=20)
        if self.odom_topic:
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=20)
        rospy.Timer(rospy.Duration(5.0), self.status_cb)
        self.min_range_sq = self.min_range * self.min_range
        self.max_range_sq = self.max_range * self.max_range
        self.max_valid_sensor_range = self.sensor_max_range - self.max_range_margin
        self.max_valid_sensor_range_sq = self.max_valid_sensor_range * self.max_valid_sensor_range
        rospy.logwarn(
            "[PointCloud2Relay] %s PointCloud -> %s PointCloud2 frame_override='%s' "
            "output_frame='%s' publish_world=%s world_frame='%s' stamp_with_odom=%s "
            "intensity=%s min_range=%.2f max_range=%.2f sensor_max=%.2f margin=%.2f "
            "voxel=%.3f local_z=[%.2f,%.2f] neighbor_voxel=%.3f min_neighbors=%d "
            "imu='%s' max_imu_ang=%.2f max_imu_age=%.2f "
            "odom='%s' max_lin=%.2f max_odom_ang=%.2f max_odom_age=%.2f "
            "ground_filter=%s ground_z=%.2f keep_above=%.2f "
            "ground_column=%s column_size=%.2f column_min_height=%.2f column_support_radius=%d "
            "self_filter=%s body_radius=%.2f body_z=[%.2f,%.2f] "
            "rotor_radius=%.2f rotor_z=[%.2f,%.2f] "
            "mount=[%.2f,%.2f,%.2f]",
            self.input_topic, self.output_topic, self.frame_id,
            self.output_frame_id, self.publish_in_world_frame, self.world_frame_id,
            self.stamp_with_odom,
            self.include_intensity, self.min_range, self.max_range,
            self.sensor_max_range, self.max_range_margin, self.voxel_size,
            self.local_min_z, self.local_max_z,
            self.neighbor_voxel_size, self.min_neighbors, self.imu_topic,
            self.max_angular_velocity, self.max_imu_age, self.odom_topic,
            self.max_linear_velocity, self.max_odom_angular_velocity, self.max_odom_age,
            self.ground_filter_enabled, self.ground_z, self.ground_keep_above,
            self.ground_column_filter_enabled, self.ground_column_size,
            self.ground_column_min_height, self.ground_column_support_radius,
            self.self_filter_enabled, self.self_filter_body_radius,
            self.self_filter_body_z_min, self.self_filter_body_z_max,
            self.self_filter_rotor_radius, self.self_filter_rotor_z_min,
            self.self_filter_rotor_z_max,
            self.lidar_mount_x, self.lidar_mount_y, self.lidar_mount_z)

    def imu_cb(self, msg):
        self.last_imu = msg

    def odom_cb(self, msg):
        self.last_odom = msg

    def status_cb(self, _event):
        rospy.logwarn(
            "[PointCloud2Relay] frames in=%d pub=%d drop(no_imu=%d old_imu=%d imu_ang=%d "
            "no_odom=%d old_odom=%d linear=%d odom_ang=%d) points in=%d pub=%d "
            "drop_ground=%d keep_supported_ground=%d drop_self=%d",
            self.input_frames, self.published_frames,
            self.dropped_no_imu, self.dropped_old_imu, self.dropped_imu_angular,
            self.dropped_no_odom, self.dropped_old_odom, self.dropped_linear,
            self.dropped_odom_angular, self.input_points, self.published_points,
            self.dropped_ground_points, self.kept_supported_ground_points,
            self.dropped_self_points)

    def stamp_is_old(self, stamp, reference_stamp, max_age):
        if max_age <= 0.0 or not stamp or not reference_stamp:
            return False
        if stamp.to_sec() <= 0.0 or reference_stamp.to_sec() <= 0.0:
            return False
        return abs((stamp - reference_stamp).to_sec()) > max_age

    def motion_is_stable(self, stamp):
        if self.imu_topic and self.max_angular_velocity > 0.0:
            if self.last_imu is None:
                self.dropped_no_imu += 1
            elif self.stamp_is_old(stamp, self.last_imu.header.stamp, self.max_imu_age):
                self.dropped_old_imu += 1
            else:
                angular = self.last_imu.angular_velocity
                angular_norm = math.sqrt(
                    angular.x * angular.x + angular.y * angular.y + angular.z * angular.z)
                if angular_norm > self.max_angular_velocity:
                    self.dropped_imu_angular += 1
                    return False

        if self.odom_topic and (
                self.max_linear_velocity > 0.0 or self.max_odom_angular_velocity > 0.0):
            if self.last_odom is None:
                self.dropped_no_odom += 1
                return False
            if self.stamp_is_old(stamp, self.last_odom.header.stamp, self.max_odom_age):
                self.dropped_old_odom += 1
                return False
            linear = self.last_odom.twist.twist.linear
            linear_norm = math.sqrt(
                linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
            if self.max_linear_velocity > 0.0 and linear_norm > self.max_linear_velocity:
                self.dropped_linear += 1
                return False
            angular = self.last_odom.twist.twist.angular
            angular_norm = math.sqrt(
                angular.x * angular.x + angular.y * angular.y + angular.z * angular.z)
            if self.max_odom_angular_velocity > 0.0 and angular_norm > self.max_odom_angular_velocity:
                self.dropped_odom_angular += 1
                return False
        return True

    def rotate_vector(self, x, y, z):
        q = self.last_odom.pose.pose.orientation
        return (
            (1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * x
            + 2.0 * (q.x * q.y - q.z * q.w) * y
            + 2.0 * (q.x * q.z + q.y * q.w) * z,
            2.0 * (q.x * q.y + q.z * q.w) * x
            + (1.0 - 2.0 * (q.x * q.x + q.z * q.z)) * y
            + 2.0 * (q.y * q.z - q.x * q.w) * z,
            2.0 * (q.x * q.z - q.y * q.w) * x
            + 2.0 * (q.y * q.z + q.x * q.w) * y
            + (1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * z,
        )

    def transform_point_to_world(self, point):
        if self.last_odom is None:
            return None
        rx, ry, rz = self.rotate_vector(
            point.x + self.lidar_mount_x,
            point.y + self.lidar_mount_y,
            point.z + self.lidar_mount_z,
        )
        position = self.last_odom.pose.pose.position
        return (position.x + rx, position.y + ry, position.z + rz)

    def is_ground_point(self, world_point):
        if not self.ground_filter_enabled or world_point is None:
            return False
        return world_point[2] <= self.ground_z + self.ground_keep_above

    def ground_column_key(self, world_point):
        if world_point is None or self.ground_column_size <= 0.0:
            return None
        inv_column = 1.0 / self.ground_column_size
        return (
            int(math.floor(world_point[0] * inv_column)),
            int(math.floor(world_point[1] * inv_column)),
        )

    def column_supports_obstacle(self, world_point, column_max_z):
        if not self.ground_column_filter_enabled:
            return False
        key = self.ground_column_key(world_point)
        if key is None:
            return False
        min_supported_z = self.ground_z + max(
            self.ground_keep_above, self.ground_column_min_height)
        support_radius = max(0, self.ground_column_support_radius)
        for dx in range(-support_radius, support_radius + 1):
            for dy in range(-support_radius, support_radius + 1):
                if column_max_z.get((key[0] + dx, key[1] + dy), -999.0) >= min_supported_z:
                    return True
        return False

    def is_self_point(self, point):
        if not self.self_filter_enabled:
            return False
        body_radius_sq = self.self_filter_body_radius * self.self_filter_body_radius
        if (
                point.x * point.x + point.y * point.y <= body_radius_sq
                and self.self_filter_body_z_min <= point.z <= self.self_filter_body_z_max):
            return True

        rotor_centers = (
            (0.13, -0.22, -0.037),
            (-0.13, 0.20, -0.037),
            (0.13, 0.22, -0.037),
            (-0.13, -0.20, -0.037),
        )
        rotor_radius_sq = self.self_filter_rotor_radius * self.self_filter_rotor_radius
        for cx, cy, cz in rotor_centers:
            dx = point.x - cx
            dy = point.y - cy
            if dx * dx + dy * dy > rotor_radius_sq:
                continue
            dz = point.z - cz
            if self.self_filter_rotor_z_min <= dz <= self.self_filter_rotor_z_max:
                return True
        return False

    def output_point(self, point, world_point):
        if self.publish_in_world_frame and world_point is not None:
            return world_point
        return (point.x, point.y, point.z)

    def active_output_frame(self, header):
        if self.publish_in_world_frame:
            return self.world_frame_id
        if self.output_frame_id:
            return self.output_frame_id
        return header.frame_id

    def active_output_stamp(self, header):
        if self.stamp_with_odom and self.last_odom is not None:
            return self.last_odom.header.stamp
        return header.stamp

    def can_transform_points(self):
        if not self.publish_in_world_frame and not self.ground_filter_enabled:
            return True
        return self.last_odom is not None

    def zero_output_point(self):
        if self.publish_in_world_frame and self.last_odom is not None:
            position = self.last_odom.pose.pose.position
            return (position.x, position.y, position.z)
        return (0.0, 0.0, 0.0)

    def point_coords(self, point):
        return point

    def point_key(self, point, inv_voxel):
        return (
            int(math.floor(point[0] * inv_voxel)),
            int(math.floor(point[1] * inv_voxel)),
            int(math.floor(point[2] * inv_voxel)),
        )

    def point_z(self, point):
        return point[2]

    def point_tuple(self, point):
        return point

    def point_with_intensity(self, point):
        return (point[0], point[1], point[2], 1.0)

    def update_ground_stats(self, world_point):
        return

    def should_transform_with_odom(self):
        return self.publish_in_world_frame or self.ground_filter_enabled

    def odom_is_required(self):
        return self.should_transform_with_odom() or self.max_linear_velocity > 0.0 or self.max_odom_angular_velocity > 0.0

    def ensure_odom_available(self):
        return not self.odom_is_required() or self.last_odom is not None

    def ground_filter_world_z(self, point):
        odom_z = self.last_odom.pose.pose.position.z
        return odom_z + self.rotate_vector(
            point.x + self.lidar_mount_x,
            point.y + self.lidar_mount_y,
            point.z + self.lidar_mount_z,
        )[2]

    def remove_isolated_points(self, filtered):
        if self.neighbor_voxel_size <= 0.0 or self.min_neighbors <= 1:
            return filtered

        inv_neighbor_voxel = 1.0 / self.neighbor_voxel_size
        occupancy = {}
        point_keys = []
        for _, point in filtered:
            key = self.point_key(point, inv_neighbor_voxel)
            point_keys.append(key)
            occupancy[key] = occupancy.get(key, 0) + 1

        kept = []
        for entry, key in zip(filtered, point_keys):
            neighbors = 0
            keep = False
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        neighbors += occupancy.get((key[0] + dx, key[1] + dy, key[2] + dz), 0)
                        if neighbors >= self.min_neighbors:
                            keep = True
                            break
                    if keep:
                        break
                if keep:
                    break
            if keep:
                kept.append(entry)
        return kept

    def filter_points(self, points):
        candidates = []
        column_max_z = {}
        use_local_z = self.local_min_z > -999.0 or self.local_max_z < 999.0
        transform_with_odom = self.should_transform_with_odom()
        for point in points:
            if not (math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)):
                continue
            if self.is_self_point(point):
                self.dropped_self_points += 1
                continue
            # Sensor-frame Z filter: remove ground hits (below) and sky noise (above)
            if use_local_z and (point.z < self.local_min_z or point.z > self.local_max_z):
                continue
            range_sq = point.x * point.x + point.y * point.y + point.z * point.z
            if self.min_range > 0.0 and range_sq < self.min_range_sq:
                continue
            if self.max_range > 0.0 and range_sq > self.max_range_sq:
                continue
            if self.max_valid_sensor_range > 0.0 and range_sq >= self.max_valid_sensor_range_sq:
                continue
            world_point = self.transform_point_to_world(point) if transform_with_odom else None
            output_point = self.output_point(point, world_point)
            candidates.append((range_sq, output_point, world_point))
            if self.ground_filter_enabled and self.ground_column_filter_enabled and world_point is not None:
                support_z = self.ground_z + max(
                    self.ground_keep_above, self.ground_column_min_height)
                if world_point[2] > support_z:
                    key = self.ground_column_key(world_point)
                    if key is not None:
                        column_max_z[key] = max(column_max_z.get(key, -999.0), world_point[2])

        filtered = []
        for range_sq, output_point, world_point in candidates:
            if self.is_ground_point(world_point):
                if self.column_supports_obstacle(world_point, column_max_z):
                    self.kept_supported_ground_points += 1
                else:
                    self.dropped_ground_points += 1
                    continue
            filtered.append((range_sq, output_point))

        filtered = self.remove_isolated_points(filtered)
        if self.voxel_size <= 0.0:
            return [point for _, point in filtered]

        inv_voxel = 1.0 / self.voxel_size
        voxels = {}
        for range_sq, point in filtered:
            key = self.point_key(point, inv_voxel)
            existing = voxels.get(key)
            if existing is None or range_sq < existing[0]:
                voxels[key] = (range_sq, point)
        return [point for _, point in voxels.values()]

    def cloud_cb(self, msg):
        self.input_frames += 1
        self.input_points += len(msg.points)
        if not self.motion_is_stable(msg.header.stamp):
            return
        if not self.can_transform_points():
            self.dropped_no_odom += 1
            return
        # Rate limiter: skip frames that arrive too fast
        now = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0 else time.time()
        if self.min_publish_interval > 0.0 and (now - self.last_publish_time) < self.min_publish_interval:
            return
        header = msg.header
        header.frame_id = self.active_output_frame(msg.header)
        header.stamp = self.active_output_stamp(msg.header)
        raw_points = self.filter_points(msg.points)
        self.published_points += len(raw_points)
        if self.include_intensity:
            fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
                PointField("intensity", 12, PointField.FLOAT32, 1),
            ]
            points = [self.point_with_intensity(p) for p in raw_points]
            self.pub.publish(pc2.create_cloud(header, fields, points))
        else:
            self.pub.publish(pc2.create_cloud_xyz32(header, raw_points))
        self.last_publish_time = now
        self.published_frames += 1


def main():
    rospy.init_node("pointcloud_to_pointcloud2_relay")
    PointCloud2Relay()
    rospy.spin()


if __name__ == "__main__":
    main()
