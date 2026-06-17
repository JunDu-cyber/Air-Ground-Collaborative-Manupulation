#!/usr/bin/env python3
"""Transform a UAV LiDAR PointCloud2 into a fixed world frame for UGV mapping."""

import copy
import math
from collections import OrderedDict

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from sensor_msgs.msg import PointCloud2

try:
    from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
except ImportError:
    do_transform_cloud = None


def bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def clean_frame(frame_id):
    return str(frame_id or "").strip().lstrip("/")


class UavPointCloudToWorld(object):
    def __init__(self):
        if do_transform_cloud is None:
            rospy.logfatal(
                "[UavPointCloudToWorld] Missing Python module tf2_sensor_msgs. "
                "Install ros-noetic-tf2-sensor-msgs before running this node."
            )
            raise ImportError("tf2_sensor_msgs")

        self.input_topic = rospy.get_param("~input_topic", "/uav0/lidar/points")
        self.output_topic = rospy.get_param(
            "~output_topic", "/uav0/mapping/points_world"
        )
        self.target_frame = clean_frame(rospy.get_param("~target_frame", "map"))
        self.throttle_rate = float(
            rospy.get_param("~throttle_rate", rospy.get_param("~publish_rate", 3.0))
        )
        self.tf_lookup_timeout = float(rospy.get_param("~tf_lookup_timeout", 0.10))
        self.allow_latest_tf_fallback = bool_param("~allow_latest_tf_fallback", True)

        self.enable_range_filter = bool_param("~enable_range_filter", True)
        self.min_range = max(float(rospy.get_param("~min_range", 0.0)), 0.0)
        self.max_range = max(float(rospy.get_param("~max_range", 30.0)), 0.0)
        self.drop_max_range_returns = bool_param("~drop_max_range_returns", True)
        self.sensor_max_range = max(float(rospy.get_param("~sensor_max_range", 30.0)), 0.0)
        self.max_range_margin = max(float(rospy.get_param("~max_range_margin", 0.5)), 0.0)
        self.drop_far_boundary_band = bool_param("~drop_far_boundary_band", True)
        self.range_edge_margin = max(float(rospy.get_param("~range_edge_margin", 1.5)), 0.0)
        self.enable_voxel_filter = bool_param("~enable_voxel_filter", True)
        self.leaf_size = max(float(rospy.get_param("~leaf_size", 0.1)), 0.0)

        self.last_publish_time = rospy.Time(0)
        self.min_publish_period = (
            1.0 / self.throttle_rate if self.throttle_rate > 0.0 else 0.0
        )
        self.input_frames = 0
        self.published_frames = 0
        self.dropped_tf = 0
        self.dropped_rate = 0
        self.dropped_max_range_points = 0
        self.dropped_boundary_points = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber(
            self.input_topic, PointCloud2, self.cloud_cb, queue_size=1
        )
        rospy.Timer(rospy.Duration(10.0), self.status_cb)

        rospy.loginfo(
            "[UavPointCloudToWorld] input=%s output=%s target_frame=%s "
            "rate=%.2fHz tf_timeout=%.2fs latest_tf_fallback=%s "
            "range_filter=%s min=%.2f max=%.2f drop_max_range=%s "
            "sensor_max=%.2f margin=%.2f drop_boundary=%s edge_margin=%.2f "
            "voxel_filter=%s leaf=%.3f",
            self.input_topic,
            self.output_topic,
            self.target_frame,
            self.throttle_rate,
            self.tf_lookup_timeout,
            self.allow_latest_tf_fallback,
            self.enable_range_filter,
            self.min_range,
            self.max_range,
            self.drop_max_range_returns,
            self.sensor_max_range,
            self.max_range_margin,
            self.drop_far_boundary_band,
            self.range_edge_margin,
            self.enable_voxel_filter,
            self.leaf_size,
        )

    def status_cb(self, _event):
        rospy.loginfo(
            "[UavPointCloudToWorld] frames in=%d pub=%d dropped_tf=%d "
            "dropped_rate=%d dropped_max_range_points=%d dropped_boundary_points=%d",
            self.input_frames,
            self.published_frames,
            self.dropped_tf,
            self.dropped_rate,
            self.dropped_max_range_points,
            self.dropped_boundary_points,
        )

    def should_publish_now(self):
        if self.min_publish_period <= 0.0 or self.last_publish_time == rospy.Time(0):
            return True
        return (rospy.Time.now() - self.last_publish_time).to_sec() >= self.min_publish_period

    def lookup_transform(self, source_frame, stamp):
        timeout = rospy.Duration(self.tf_lookup_timeout)
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, stamp, timeout
            )
        except tf2_ros.ExtrapolationException as exc:
            if not self.allow_latest_tf_fallback:
                raise
            rospy.logwarn_throttle(
                5.0,
                "[UavPointCloudToWorld] TF extrapolation for %s <- %s at %.3f, "
                "trying latest transform: %s",
                self.target_frame,
                source_frame,
                stamp.to_sec(),
                str(exc),
            )
            return self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rospy.Time(0), timeout
            )

    def cloud_cb(self, msg):
        self.input_frames += 1
        if not self.should_publish_now():
            self.dropped_rate += 1
            return

        source_frame = clean_frame(msg.header.frame_id)
        if not source_frame:
            self.dropped_tf += 1
            rospy.logwarn_throttle(
                5.0,
                "[UavPointCloudToWorld] input cloud has empty header.frame_id; "
                "cannot transform to %s",
                self.target_frame,
            )
            return

        cloud_in = msg
        if source_frame != msg.header.frame_id:
            cloud_in = copy.deepcopy(msg)
            cloud_in.header.frame_id = source_frame

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time(0)
        try:
            if source_frame == self.target_frame:
                transform = None
                cloud_world = copy.deepcopy(cloud_in)
            else:
                transform = self.lookup_transform(source_frame, stamp)
                cloud_world = do_transform_cloud(cloud_in, transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.dropped_tf += 1
            rospy.logwarn_throttle(
                5.0,
                "[UavPointCloudToWorld] Missing TF %s <- %s; not publishing this cloud. "
                "Check the TF chain from the LiDAR frame to the fixed frame. Error: %s",
                self.target_frame,
                source_frame,
                str(exc),
            )
            return

        cloud_world.header.frame_id = self.target_frame
        cloud_world.header.stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        )

        if transform is None:
            origin = (0.0, 0.0, 0.0)
        else:
            origin_t = transform.transform.translation
            origin = (origin_t.x, origin_t.y, origin_t.z)
        filtered = self.filter_cloud(cloud_world, origin)
        self.pub.publish(filtered)
        self.last_publish_time = rospy.Time.now()
        self.published_frames += 1

    def field_tuple_indices(self, fields):
        indices = {}
        index = 0
        for field in fields:
            indices[field.name] = index
            index += max(field.count, 1)
        return indices

    def filter_cloud(self, cloud, origin):
        if (
            not self.enable_range_filter
            and (not self.enable_voxel_filter or self.leaf_size <= 0.0)
        ):
            return cloud

        field_names = [field.name for field in cloud.fields]
        field_indices = self.field_tuple_indices(cloud.fields)
        if not all(name in field_indices for name in ("x", "y", "z")):
            rospy.logwarn_throttle(
                5.0,
                "[UavPointCloudToWorld] PointCloud2 lacks x/y/z fields; "
                "publishing transformed cloud without filters.",
            )
            return cloud

        x_idx = field_indices["x"]
        y_idx = field_indices["y"]
        z_idx = field_indices["z"]
        min_range_sq = self.min_range * self.min_range
        max_range_sq = self.max_range * self.max_range
        max_valid_range = self.sensor_max_range - self.max_range_margin
        max_valid_range_sq = max_valid_range * max_valid_range
        far_boundary_range = self.max_range if self.max_range > 0.0 else max_valid_range
        boundary_limit = far_boundary_range - self.range_edge_margin
        boundary_limit_sq = boundary_limit * boundary_limit
        rows = []
        dropped_max_range = 0
        dropped_boundary = 0

        for point in pc2.read_points(cloud, field_names=field_names, skip_nans=True):
            x = point[x_idx]
            y = point[y_idx]
            z = point[z_idx]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if self.enable_range_filter:
                dx = x - origin[0]
                dy = y - origin[1]
                dz = z - origin[2]
                dist_sq = dx * dx + dy * dy + dz * dz
                if self.min_range > 0.0 and dist_sq < min_range_sq:
                    continue
                if self.max_range > 0.0 and dist_sq > max_range_sq:
                    continue
                if (
                    self.drop_max_range_returns
                    and max_valid_range > 0.0
                    and dist_sq >= max_valid_range_sq
                ):
                    dropped_max_range += 1
                    continue
                if (
                    self.drop_far_boundary_band
                    and boundary_limit > self.min_range
                    and dist_sq >= boundary_limit_sq
                ):
                    dropped_boundary += 1
                    continue
            rows.append(tuple(point))

        self.dropped_max_range_points += dropped_max_range
        self.dropped_boundary_points += dropped_boundary
        if dropped_max_range:
            rospy.logdebug(
                "[UavPointCloudToWorld] dropped %d max-range boundary points",
                dropped_max_range,
            )
        if dropped_boundary:
            rospy.logdebug(
                "[UavPointCloudToWorld] dropped %d far boundary-band points",
                dropped_boundary,
            )

        if self.enable_voxel_filter and self.leaf_size > 0.0:
            rows = self.voxel_downsample(rows, x_idx, y_idx, z_idx)

        header = copy.deepcopy(cloud.header)
        header.frame_id = self.target_frame
        return pc2.create_cloud(header, cloud.fields, rows)

    def voxel_downsample(self, rows, x_idx, y_idx, z_idx):
        inv_leaf = 1.0 / self.leaf_size
        voxels = OrderedDict()
        for point in rows:
            key = (
                int(math.floor(point[x_idx] * inv_leaf)),
                int(math.floor(point[y_idx] * inv_leaf)),
                int(math.floor(point[z_idx] * inv_leaf)),
            )
            entry = voxels.get(key)
            if entry is None:
                voxels[key] = {
                    "count": 1,
                    "sum_x": point[x_idx],
                    "sum_y": point[y_idx],
                    "sum_z": point[z_idx],
                    "point": list(point),
                }
            else:
                entry["count"] += 1
                entry["sum_x"] += point[x_idx]
                entry["sum_y"] += point[y_idx]
                entry["sum_z"] += point[z_idx]

        downsampled = []
        for entry in voxels.values():
            point = entry["point"]
            count = float(entry["count"])
            point[x_idx] = entry["sum_x"] / count
            point[y_idx] = entry["sum_y"] / count
            point[z_idx] = entry["sum_z"] / count
            downsampled.append(tuple(point))
        return downsampled


def main():
    rospy.init_node("uav_pointcloud_to_world")
    UavPointCloudToWorld()
    rospy.spin()


if __name__ == "__main__":
    main()
