#!/usr/bin/env python3
"""Accumulate world-frame UAV point clouds and save a persistent PCD map."""

import math
import os
from datetime import datetime

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse


def bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def clean_frame(frame_id):
    return str(frame_id or "").strip().lstrip("/")


class UavPointCloudMapRecorder(object):
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/uav0/mapping/points_world"
        )
        self.accumulated_topic = rospy.get_param(
            "~accumulated_topic", "/uav0/mapping/points_accumulated"
        )
        self.frame_id = clean_frame(rospy.get_param("~frame_id", "map"))
        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/pointcloud_maps")
        )
        self.map_name = rospy.get_param("~map_name", "uav_points_map")
        self.voxel_leaf_size = max(
            float(rospy.get_param("~voxel_leaf_size", 0.15)), 1e-4
        )
        self.min_observations = max(int(rospy.get_param("~min_observations", 1)), 1)
        self.min_z = float(rospy.get_param("~min_z", -999.0))
        self.max_z = float(rospy.get_param("~max_z", 999.0))
        if self.min_z > self.max_z:
            rospy.logwarn(
                "[UavPointCloudMapRecorder] min_z %.3f is greater than max_z %.3f; "
                "disabling map z filter.",
                self.min_z,
                self.max_z,
            )
            self.min_z = -999.0
            self.max_z = 999.0
        self.use_z_filter = self.min_z > -999.0 or self.max_z < 999.0
        self.drop_flat_ground = bool_param("~drop_flat_ground", False)
        self.ground_filter_mode = str(
            rospy.get_param(
                "~ground_filter_mode", "flat" if self.drop_flat_ground else "none"
            )
        ).strip().lower()
        if self.ground_filter_mode not in ("none", "flat", "local"):
            rospy.logwarn(
                "[UavPointCloudMapRecorder] unknown ground_filter_mode '%s'; "
                "using 'none'.",
                self.ground_filter_mode,
            )
            self.ground_filter_mode = "none"
        self.ground_z = float(rospy.get_param("~ground_z", 0.0))
        self.ground_clearance = max(
            float(rospy.get_param("~ground_clearance", 0.18)), 0.0
        )
        self.obstacle_height_above_ground = max(
            float(
                rospy.get_param(
                    "~obstacle_height_above_ground", self.ground_clearance
                )
            ),
            0.0,
        )
        self.ground_xy_cell_size = max(
            float(rospy.get_param("~ground_xy_cell_size", 0.35)),
            self.voxel_leaf_size,
        )
        self.keep_terrain_points = bool_param("~keep_terrain_points", False)
        self.terrain_cell_size = max(
            float(rospy.get_param("~terrain_cell_size", 0.80)),
            self.ground_xy_cell_size,
        )
        self.publish_rate = max(float(rospy.get_param("~publish_rate", 1.0)), 0.0)
        self.publish_on_update = bool_param("~publish_on_update", True)
        self.min_update_publish_period = max(
            float(rospy.get_param("~min_update_publish_period", 0.5)), 0.0
        )
        self.ray_clearing_enabled = bool_param("~ray_clearing_enabled", False)
        self.sensor_frame = clean_frame(
            rospy.get_param("~sensor_frame", "uav0/velodyne_link")
        )
        self.ray_clear_max_range = max(
            float(rospy.get_param("~ray_clear_max_range", 0.0)), 0.0
        )
        self.ray_clear_min_range = max(
            float(rospy.get_param("~ray_clear_min_range", self.voxel_leaf_size)), 0.0
        )
        self.ray_clear_step_size = max(
            float(
                rospy.get_param(
                    "~ray_clear_step_size", self.voxel_leaf_size * 2.0
                )
            ),
            self.voxel_leaf_size,
        )
        self.ray_clear_keep_last_voxels = max(
            int(rospy.get_param("~ray_clear_keep_last_voxels", 2)), 0
        )
        self.ray_clear_decimation = max(
            int(rospy.get_param("~ray_clear_decimation", 6)), 1
        )
        self.tf_lookup_timeout = max(
            float(rospy.get_param("~tf_lookup_timeout", 0.10)), 0.0
        )
        self.save_interval = max(float(rospy.get_param("~save_interval", 30.0)), 0.0)
        self.save_latest = bool(rospy.get_param("~save_latest", True))
        self.save_timestamped = bool(rospy.get_param("~save_timestamped", True))
        self.save_on_shutdown = bool(rospy.get_param("~save_on_shutdown", True))
        self.min_points_to_save = max(int(rospy.get_param("~min_points_to_save", 1)), 1)

        self.inv_leaf = 1.0 / self.voxel_leaf_size
        self.voxels = {}
        self.input_frames = 0
        self.input_points = 0
        self.accepted_points = 0
        self.dropped_z_points = 0
        self.cleared_voxels = 0
        self.ray_clear_no_tf = 0
        self.last_ground_filter_stats = {
            "input": 0,
            "kept_obstacle": 0,
            "kept_terrain": 0,
            "dropped_ground": 0,
        }
        self.last_save_time = rospy.Time.now()
        self.last_accumulated_publish_time = rospy.Time(0)
        self.last_cloud_stamp = rospy.Time(0)

        os.makedirs(self.output_dir, exist_ok=True)

        self.pub = rospy.Publisher(
            self.accumulated_topic, PointCloud2, queue_size=1, latch=True
        )
        self.sub = rospy.Subscriber(
            self.input_topic, PointCloud2, self.cloud_cb, queue_size=1
        )
        self.save_srv = rospy.Service("~save", Trigger, self.save_service_cb)
        self.clear_srv = rospy.Service("~clear", Trigger, self.clear_service_cb)
        if self.ray_clearing_enabled:
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        if self.publish_rate > 0.0:
            rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_timer_cb)
        if self.save_interval > 0.0:
            rospy.Timer(rospy.Duration(self.save_interval), self.save_timer_cb)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(
            "[UavPointCloudMapRecorder] input=%s accumulated=%s frame=%s "
            "output_dir=%s map_name=%s voxel=%.3f publish_rate=%.2f "
            "publish_on_update=%s update_publish_period=%.2f "
            "save_interval=%.1f min_observations=%d z_filter=[%.2f,%.2f] "
            "ground_mode=%s ground_z=%.2f ground_clearance=%.2f "
            "obstacle_above_ground=%.2f ground_cell=%.2f keep_terrain=%s "
            "terrain_cell=%.2f ray_clear=%s sensor_frame=%s clear_range=[%.2f,%.2f] "
            "clear_step=%.2f keep_tail=%d decimation=%d "
            "latest=%s timestamped=%s",
            self.input_topic,
            self.accumulated_topic,
            self.frame_id,
            self.output_dir,
            self.map_name,
            self.voxel_leaf_size,
            self.publish_rate,
            self.publish_on_update,
            self.min_update_publish_period,
            self.save_interval,
            self.min_observations,
            self.min_z,
            self.max_z,
            self.ground_filter_mode,
            self.ground_z,
            self.ground_clearance,
            self.obstacle_height_above_ground,
            self.ground_xy_cell_size,
            self.keep_terrain_points,
            self.terrain_cell_size,
            self.ray_clearing_enabled,
            self.sensor_frame,
            self.ray_clear_min_range,
            self.ray_clear_max_range,
            self.ray_clear_step_size,
            self.ray_clear_keep_last_voxels,
            self.ray_clear_decimation,
            self.save_latest,
            self.save_timestamped,
        )

    def cloud_cb(self, msg):
        input_frame = clean_frame(msg.header.frame_id)
        if input_frame and input_frame != self.frame_id:
            rospy.logwarn_throttle(
                5.0,
                "[UavPointCloudMapRecorder] input frame is '%s', expected '%s'. "
                "This recorder assumes the cloud is already in the map frame.",
                input_frame,
                self.frame_id,
            )

        added = 0
        accepted = 0
        dropped_z = 0
        cleared = 0
        frame_index = self.input_frames + 1
        accepted_points = []
        endpoint_keys = set()
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if self.use_z_filter and (z < self.min_z or z > self.max_z):
                dropped_z += 1
                continue
            key = (
                int(math.floor(x * self.inv_leaf)),
                int(math.floor(y * self.inv_leaf)),
                int(math.floor(z * self.inv_leaf)),
            )
            accepted_points.append((x, y, z, key))
            endpoint_keys.add(key)
            accepted += 1

        origin = self.lookup_sensor_origin(msg.header.stamp)
        if origin is not None and accepted_points:
            cleared = self.clear_free_space(origin, accepted_points, endpoint_keys)

        for x, y, z, key in accepted_points:
            entry = self.voxels.get(key)
            if entry is None:
                self.voxels[key] = [x, y, z, 1, 1, frame_index]
                added += 1
            else:
                entry[0] += x
                entry[1] += y
                entry[2] += z
                entry[3] += 1
                if entry[5] != frame_index:
                    entry[4] += 1
                    entry[5] = frame_index

        self.input_frames += 1
        self.input_points += msg.width * msg.height
        self.accepted_points += accepted
        self.dropped_z_points += dropped_z
        self.cleared_voxels += cleared
        self.last_cloud_stamp = msg.header.stamp
        rospy.loginfo_throttle(
            5.0,
            "[UavPointCloudMapRecorder] frames=%d input_points=%d accepted=%d "
            "voxels=%d confirmed=%d last_new=%d cleared=%d/%d output=%d "
            "kept_obstacle=%d kept_terrain=%d dropped_ground=%d dropped_z=%d",
            self.input_frames,
            self.input_points,
            self.accepted_points,
            len(self.voxels),
            self.confirmed_voxel_count(),
            added,
            cleared,
            self.cleared_voxels,
            self.last_ground_filter_stats["kept_obstacle"]
            + self.last_ground_filter_stats["kept_terrain"],
            self.last_ground_filter_stats["kept_obstacle"],
            self.last_ground_filter_stats["kept_terrain"],
            self.last_ground_filter_stats["dropped_ground"],
            self.dropped_z_points,
        )
        if self.publish_on_update:
            self.publish_accumulated_if_due()

    def lookup_sensor_origin(self, stamp):
        if not self.ray_clearing_enabled:
            return None
        if not self.sensor_frame:
            rospy.logwarn_throttle(
                5.0,
                "[UavPointCloudMapRecorder] ray clearing is enabled but sensor_frame is empty.",
            )
            return None
        query_stamp = stamp if stamp != rospy.Time() else rospy.Time(0)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.sensor_frame,
                query_stamp,
                rospy.Duration(self.tf_lookup_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.ray_clear_no_tf += 1
            rospy.logwarn_throttle(
                5.0,
                "[UavPointCloudMapRecorder] ray clearing disabled for this frame: "
                "missing TF %s <- %s at %.3f (%s). Endpoints are still recorded.",
                self.frame_id,
                self.sensor_frame,
                query_stamp.to_sec(),
                str(exc),
            )
            return None

        t = transform.transform.translation
        return (t.x, t.y, t.z)

    def clear_free_space(self, origin, accepted_points, endpoint_keys):
        clear_keys = set()
        ox, oy, oz = origin
        max_range = self.ray_clear_max_range
        min_range = self.ray_clear_min_range
        for index, (x, y, z, endpoint_key) in enumerate(accepted_points):
            if index % self.ray_clear_decimation != 0:
                continue
            dx = x - ox
            dy = y - oy
            dz = z - oz
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance <= min_range:
                continue
            if max_range > 0.0 and distance > max_range:
                continue

            steps = max(int(math.floor(distance / self.ray_clear_step_size)), 1)
            last_clear_step = max(0, steps - self.ray_clear_keep_last_voxels)
            for step in range(1, last_clear_step + 1):
                ratio = float(step) / float(steps)
                key = (
                    int(math.floor((ox + dx * ratio) * self.inv_leaf)),
                    int(math.floor((oy + dy * ratio) * self.inv_leaf)),
                    int(math.floor((oz + dz * ratio) * self.inv_leaf)),
                )
                if key == endpoint_key or key in endpoint_keys:
                    continue
                clear_keys.add(key)

        cleared = 0
        for key in clear_keys:
            if key in self.voxels:
                del self.voxels[key]
                cleared += 1
        return cleared

    def confirmed_voxel_count(self):
        return sum(
            1 for entry in self.voxels.values() if entry[4] >= self.min_observations
        )

    def accumulated_points(self):
        points = []
        for sx, sy, sz, sample_count, observation_count, _last_frame in self.voxels.values():
            if observation_count < self.min_observations:
                continue
            inv_count = 1.0 / float(sample_count)
            points.append((sx * inv_count, sy * inv_count, sz * inv_count))
        return self.apply_ground_filter(points)

    def point_xy_key(self, point, cell_size):
        inv_cell = 1.0 / cell_size
        return (
            int(math.floor(point[0] * inv_cell)),
            int(math.floor(point[1] * inv_cell)),
        )

    def ground_percentile(self, values):
        if not values:
            return None
        ordered = sorted(values)
        if len(ordered) <= 2:
            return ordered[0]
        index = int(math.floor(0.20 * float(len(ordered) - 1)))
        return ordered[index]

    def sparse_terrain_points(self, ground_points):
        if not self.keep_terrain_points:
            return []
        cells = {}
        for point in ground_points:
            key = self.point_xy_key(point, self.terrain_cell_size)
            existing = cells.get(key)
            if existing is None or point[2] < existing[2]:
                cells[key] = point
        return list(cells.values())

    def apply_ground_filter(self, points):
        if not points or self.ground_filter_mode == "none":
            self.last_ground_filter_stats = {
                "input": len(points),
                "kept_obstacle": len(points),
                "kept_terrain": 0,
                "dropped_ground": 0,
            }
            return points

        obstacle_points = []
        ground_points = []
        if self.ground_filter_mode == "flat":
            ground_limit = self.ground_z + self.ground_clearance
            for point in points:
                if point[2] <= ground_limit:
                    ground_points.append(point)
                else:
                    obstacle_points.append(point)
        else:
            columns = {}
            for point in points:
                key = self.point_xy_key(point, self.ground_xy_cell_size)
                columns.setdefault(key, []).append(point[2])
            ground_by_column = {
                key: self.ground_percentile(values)
                for key, values in columns.items()
            }
            for point in points:
                key = self.point_xy_key(point, self.ground_xy_cell_size)
                local_ground_z = ground_by_column.get(key)
                if local_ground_z is None:
                    obstacle_points.append(point)
                    continue
                if point[2] <= local_ground_z + self.obstacle_height_above_ground:
                    ground_points.append(point)
                else:
                    obstacle_points.append(point)

        terrain_points = self.sparse_terrain_points(ground_points)
        self.last_ground_filter_stats = {
            "input": len(points),
            "kept_obstacle": len(obstacle_points),
            "kept_terrain": len(terrain_points),
            "dropped_ground": max(0, len(ground_points) - len(terrain_points)),
        }
        return obstacle_points + terrain_points

    def publish_timer_cb(self, _event):
        self.publish_accumulated()

    def publish_accumulated_if_due(self):
        now = rospy.Time.now()
        if (
            self.last_accumulated_publish_time != rospy.Time(0)
            and self.min_update_publish_period > 0.0
            and (now - self.last_accumulated_publish_time).to_sec()
            < self.min_update_publish_period
        ):
            return
        self.publish_accumulated()

    def publish_accumulated(self):
        points = self.accumulated_points()
        if not points:
            return
        stamp = self.last_cloud_stamp if self.last_cloud_stamp != rospy.Time() else rospy.Time.now()
        header = Header(stamp=stamp, frame_id=self.frame_id)
        self.pub.publish(pc2.create_cloud_xyz32(header, points))
        self.last_accumulated_publish_time = rospy.Time.now()

    def save_timer_cb(self, _event):
        self.save_map(reason="timer")

    def save_service_cb(self, _req):
        path = self.save_map(reason="service")
        if path:
            return TriggerResponse(True, path)
        return TriggerResponse(False, "not enough accumulated points to save")

    def clear_service_cb(self, _req):
        voxel_count = len(self.voxels)
        self.voxels.clear()
        self.last_ground_filter_stats = {
            "input": 0,
            "kept_obstacle": 0,
            "kept_terrain": 0,
            "dropped_ground": 0,
        }
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        self.pub.publish(pc2.create_cloud_xyz32(header, []))
        self.last_accumulated_publish_time = rospy.Time.now()
        message = "cleared %d accumulated voxels" % voxel_count
        rospy.logwarn("[UavPointCloudMapRecorder] %s", message)
        return TriggerResponse(True, message)

    def on_shutdown(self):
        if self.save_on_shutdown:
            self.save_map(reason="shutdown")

    def save_map(self, reason="manual"):
        points = self.accumulated_points()
        if len(points) < self.min_points_to_save:
            rospy.logwarn(
                "[UavPointCloudMapRecorder] skip save on %s: only %d points",
                reason,
                len(points),
            )
            return ""

        saved_paths = []
        if self.save_latest:
            latest_path = os.path.join(self.output_dir, "%s_latest.pcd" % self.map_name)
            self.write_ascii_pcd(latest_path, points)
            saved_paths.append(latest_path)
        if self.save_timestamped:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            timestamped_path = os.path.join(
                self.output_dir, "%s_%s.pcd" % (self.map_name, stamp)
            )
            self.write_ascii_pcd(timestamped_path, points)
            saved_paths.append(timestamped_path)

        self.last_save_time = rospy.Time.now()
        rospy.loginfo(
            "[UavPointCloudMapRecorder] saved %d points on %s: %s",
            len(points),
            reason,
            ", ".join(saved_paths),
        )
        return saved_paths[0] if saved_paths else ""

    def write_ascii_pcd(self, path, points):
        tmp_path = path + ".tmp"
        with open(tmp_path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write("WIDTH %d\n" % len(points))
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write("POINTS %d\n" % len(points))
            f.write("DATA ascii\n")
            for x, y, z in points:
                f.write("%.6f %.6f %.6f\n" % (x, y, z))
        os.replace(tmp_path, path)


def main():
    rospy.init_node("uav_pointcloud_map_recorder")
    UavPointCloudMapRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
