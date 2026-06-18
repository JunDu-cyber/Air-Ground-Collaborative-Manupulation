#!/usr/bin/env python3
"""Publish a predicted intercept goal from dual-UAV truth odometry."""

import math

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class TruthTracker:
    def __init__(self):
        self.prediction_mode = rospy.get_param("~prediction_mode", "fixed").lower()
        self.prediction_time = rospy.get_param("~prediction_time", 1.5)
        self.goal_update_rate = max(rospy.get_param("~goal_update_rate", 2.0), 0.1)
        self.use_prediction = rospy.get_param("~use_prediction", True)
        self.use_twist_velocity = rospy.get_param("~use_twist_velocity", False)
        self.velocity_source = rospy.get_param("~velocity_source", "diff").lower()
        self.min_twist_speed = rospy.get_param("~min_twist_speed", 0.03)
        self.max_goal_jump = rospy.get_param("~max_goal_jump", 3.0)
        self.goal_frame_id = rospy.get_param("~frame_id", "map")
        self.marker_scale = rospy.get_param("~marker_scale", 0.55)
        self.smooth_prediction = rospy.get_param("~smooth_prediction", True)
        self.t_go_smoothing_alpha = min(
            max(rospy.get_param("~t_go_smoothing_alpha", 0.25), 0.0), 1.0
        )
        self.goal_smoothing_alpha = min(
            max(rospy.get_param("~goal_smoothing_alpha", 0.35), 0.0), 1.0
        )
        self.max_t_go_rate = max(rospy.get_param("~max_t_go_rate", 0.8), 0.0)
        self.max_goal_speed = max(rospy.get_param("~max_goal_speed", 1.2), 0.0)
        self.keep_goal_on_circle = rospy.get_param("~keep_goal_on_circle", True)

        self.assumed_chaser_speed = max(
            rospy.get_param("~assumed_chaser_speed", 1.8), 0.05
        )
        self.min_prediction_time = max(
            rospy.get_param("~min_prediction_time", 0.5), 0.0
        )
        self.max_prediction_time = max(
            rospy.get_param("~max_prediction_time", 4.0), self.min_prediction_time
        )
        self.intercept_iterations = max(
            int(rospy.get_param("~intercept_iterations", 5)), 1
        )
        self.target_motion_mode = rospy.get_param("~target_motion_mode", "circle").lower()
        requested_prediction_model = rospy.get_param("~prediction_model", "auto").lower()
        self.prediction_model = self._resolve_prediction_model(
            requested_prediction_model, self.target_motion_mode
        )
        self.target_center_x = rospy.get_param("~target_center_x", 2.0)
        self.target_center_y = rospy.get_param("~target_center_y", 0.0)
        self.target_radius = max(rospy.get_param("~target_radius", 3.0), 0.05)
        self.target_direction = rospy.get_param("~target_direction", 1.0)
        self.infer_target_direction = rospy.get_param("~infer_target_direction", True)
        self.target_speed = max(rospy.get_param("~target_speed", 0.6), 0.0)
        self.target_height = rospy.get_param("~target_height", 0.0)

        target_odom_topic = rospy.get_param(
            "~target_odom_topic", "/uav1/mavros/local_position/odom"
        )
        chaser_odom_topic = rospy.get_param(
            "~chaser_odom_topic", "/uav0/mavros/local_position/odom"
        )
        goal_topic = rospy.get_param("~goal_topic", "/uav0/tracker/goal")
        marker_topic = rospy.get_param("~marker_topic", "/uav0/tracker/marker")

        self.last_stamp = None
        self.last_pos = None
        self.latest_odom = None
        self.latest_velocity = (0.0, 0.0, 0.0)
        self.latest_chaser_odom = None
        self.last_goal_pos = None
        self.filtered_t_go = None
        self.filtered_goal_pos = None
        self.last_filter_stamp = None

        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=10)
        self.odom_sub = rospy.Subscriber(
            target_odom_topic, Odometry, self.odom_callback, queue_size=10
        )
        self.chaser_odom_sub = rospy.Subscriber(
            chaser_odom_topic, Odometry, self.chaser_odom_callback, queue_size=10
        )
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.goal_update_rate), self.publish_goal
        )

        rospy.loginfo(
            "[TruthTracker] target=%s chaser=%s goal=%s marker=%s mode=%s requested_model=%s resolved_model=%s target_motion=%s update=%.2fHz fixed_T=%.2fs speed=%.2fm/s window=[%.2f %.2f] iters=%d smooth=%s t_alpha=%.2f goal_alpha=%.2f max_t_rate=%.2f max_goal_speed=%.2f",
            target_odom_topic,
            chaser_odom_topic,
            goal_topic,
            marker_topic,
            self.prediction_mode,
            requested_prediction_model,
            self.prediction_model,
            self.target_motion_mode,
            self.goal_update_rate,
            self.prediction_time,
            self.assumed_chaser_speed,
            self.min_prediction_time,
            self.max_prediction_time,
            self.intercept_iterations,
            self.smooth_prediction,
            self.t_go_smoothing_alpha,
            self.goal_smoothing_alpha,
            self.max_t_go_rate,
            self.max_goal_speed,
        )

    @staticmethod
    def _speed(vx, vy, vz):
        return math.sqrt(vx * vx + vy * vy + vz * vz)

    @staticmethod
    def _distance(a, b):
        return math.sqrt(
            (a[0] - b[0]) * (a[0] - b[0])
            + (a[1] - b[1]) * (a[1] - b[1])
            + (a[2] - b[2]) * (a[2] - b[2])
        )

    @staticmethod
    def _yaw_to_quat(yaw):
        half = 0.5 * yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))

    @staticmethod
    def _point_tuple(position):
        return (position.x, position.y, position.z)

    @staticmethod
    def _resolve_prediction_model(prediction_model, target_motion_mode):
        if prediction_model in ("auto", "target", "configured"):
            if target_motion_mode == "circle":
                return "circle"
            if target_motion_mode == "line":
                return "line"
            return "linear"
        return prediction_model

    @staticmethod
    def _clamp(value, low, high):
        return min(max(value, low), high)

    def _clamp_time(self, t_go):
        return min(max(t_go, self.min_prediction_time), self.max_prediction_time)

    def _estimate_velocity(self, msg):
        pos = msg.pose.pose.position
        twist = msg.twist.twist.linear
        twist_speed = self._speed(twist.x, twist.y, twist.z)

        if self.velocity_source == "twist" and twist_speed >= self.min_twist_speed:
            return twist.x, twist.y, twist.z

        if (
            self.velocity_source == "auto"
            and self.use_twist_velocity
            and twist_speed >= self.min_twist_speed
        ):
            return twist.x, twist.y, twist.z

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        if self.last_stamp is None or self.last_pos is None:
            return twist.x, twist.y, twist.z

        dt = (stamp - self.last_stamp).to_sec()
        if dt <= 1e-3:
            return twist.x, twist.y, twist.z

        return (
            (pos.x - self.last_pos[0]) / dt,
            (pos.y - self.last_pos[1]) / dt,
            (pos.z - self.last_pos[2]) / dt,
        )

    def _linear_intercept_time(self, target_pos, target_vel, chaser_pos):
        rx = target_pos[0] - chaser_pos[0]
        ry = target_pos[1] - chaser_pos[1]
        rz = target_pos[2] - chaser_pos[2]
        vx, vy, vz = target_vel
        s = self.assumed_chaser_speed

        a = vx * vx + vy * vy + vz * vz - s * s
        b = 2.0 * (rx * vx + ry * vy + rz * vz)
        c = rx * rx + ry * ry + rz * rz

        if abs(a) < 1e-6:
            if abs(b) < 1e-6:
                return self._clamp_time(math.sqrt(c) / s)
            root = -c / b
            return self._clamp_time(root if root > 0.0 else math.sqrt(c) / s)

        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return self._clamp_time(math.sqrt(c) / s)

        sqrt_disc = math.sqrt(disc)
        roots = [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)]
        positive_roots = [root for root in roots if root > 0.0]
        if not positive_roots:
            return self._clamp_time(math.sqrt(c) / s)
        return self._clamp_time(min(positive_roots))

    def _linear_future(self, target_pos, target_vel, t_go):
        return (
            target_pos[0] + target_vel[0] * t_go,
            target_pos[1] + target_vel[1] * t_go,
            target_pos[2] + target_vel[2] * t_go,
        )

    def _circle_future(self, target_pos, target_vel, t_go):
        theta = math.atan2(
            target_pos[1] - self.target_center_y,
            target_pos[0] - self.target_center_x,
        )
        xy_speed = self._speed(target_vel[0], target_vel[1], 0.0)
        target_speed = self.target_speed if self.target_speed > 1e-6 else xy_speed
        direction = 1.0 if self.target_direction >= 0.0 else -1.0
        if self.infer_target_direction and xy_speed > 1e-3:
            rx = target_pos[0] - self.target_center_x
            ry = target_pos[1] - self.target_center_y
            cross_z = rx * target_vel[1] - ry * target_vel[0]
            if abs(cross_z) > 1e-4:
                direction = 1.0 if cross_z > 0.0 else -1.0
        omega = target_speed / self.target_radius
        theta_future = theta + direction * omega * t_go
        z = self.target_height if self.target_height > 0.0 else target_pos[2]
        return (
            self.target_center_x + self.target_radius * math.cos(theta_future),
            self.target_center_y + self.target_radius * math.sin(theta_future),
            z,
        )

    def _line_future(self, target_pos, target_vel, t_go):
        x_min = self.target_center_x - self.target_radius
        x_max = self.target_center_x + self.target_radius
        segment_len = x_max - x_min
        z = self.target_height if self.target_height > 0.0 else target_pos[2]
        if segment_len <= 1e-6:
            return self.target_center_x, self.target_center_y, z

        measured_speed = self._speed(target_vel[0], target_vel[1], 0.0)
        target_speed = self.target_speed if self.target_speed > 1e-6 else measured_speed
        if target_speed <= 1e-4:
            return self._linear_future(target_pos, target_vel, t_go)

        direction = 1.0 if self.target_direction >= 0.0 else -1.0
        if abs(target_vel[0]) > 1e-3:
            direction = 1.0 if target_vel[0] > 0.0 else -1.0

        s_now = self._clamp(target_pos[0] - x_min, 0.0, segment_len)
        phase = (s_now + direction * target_speed * t_go) % (2.0 * segment_len)
        if phase <= segment_len:
            x = x_min + phase
        else:
            x = x_max - (phase - segment_len)
        return x, self.target_center_y, z

    def _future_position(self, target_pos, target_vel, t_go):
        if self.prediction_model == "circle":
            return self._circle_future(target_pos, target_vel, t_go)
        if self.prediction_model == "line":
            return self._line_future(target_pos, target_vel, t_go)
        if self.prediction_model != "linear":
            rospy.logwarn_throttle(
                5.0,
                "[TruthTracker] unknown prediction_model '%s', using linear",
                self.prediction_model,
            )
        return self._linear_future(target_pos, target_vel, t_go)

    def _goal_for_time(self, target_pos, target_vel, t_go):
        if not self.use_prediction:
            return target_pos
        if self.prediction_mode == "fixed" or self.prediction_mode == "adaptive":
            return self._linear_future(target_pos, target_vel, t_go)
        return self._future_position(target_pos, target_vel, t_go)

    def _project_goal_to_circle(self, goal_pos):
        dx = goal_pos[0] - self.target_center_x
        dy = goal_pos[1] - self.target_center_y
        norm = self._speed(dx, dy, 0.0)
        if norm <= 1e-6:
            return goal_pos
        scale = self.target_radius / norm
        z = self.target_height if self.target_height > 0.0 else goal_pos[2]
        return (
            self.target_center_x + dx * scale,
            self.target_center_y + dy * scale,
            z,
        )

    def _limit_vector_step(self, previous, current, max_step):
        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
        dz = current[2] - previous[2]
        dist = self._speed(dx, dy, dz)
        if max_step <= 0.0 or dist <= max_step or dist <= 1e-6:
            return current
        scale = max_step / dist
        return (
            previous[0] + dx * scale,
            previous[1] + dy * scale,
            previous[2] + dz * scale,
        )

    def _smooth_goal(self, target_pos, target_vel, raw_goal_pos, raw_t_go, stamp):
        if not self.smooth_prediction:
            return raw_goal_pos, raw_t_go

        if self.last_filter_stamp is None:
            dt = 1.0 / self.goal_update_rate
        else:
            dt = max((stamp - self.last_filter_stamp).to_sec(), 1.0 / self.goal_update_rate)
            dt = min(dt, 0.5)
        self.last_filter_stamp = stamp

        raw_t_go = self._clamp_time(raw_t_go)
        if self.filtered_t_go is None:
            filtered_t_go = raw_t_go
        else:
            limited_t_go = raw_t_go
            max_t_delta = self.max_t_go_rate * dt
            if max_t_delta > 0.0:
                delta = raw_t_go - self.filtered_t_go
                if delta > max_t_delta:
                    limited_t_go = self.filtered_t_go + max_t_delta
                elif delta < -max_t_delta:
                    limited_t_go = self.filtered_t_go - max_t_delta

            filtered_t_go = (
                self.filtered_t_go
                + self.t_go_smoothing_alpha * (limited_t_go - self.filtered_t_go)
            )
            filtered_t_go = self._clamp_time(filtered_t_go)
        self.filtered_t_go = filtered_t_go

        time_goal_pos = self._goal_for_time(target_pos, target_vel, filtered_t_go)
        if self.filtered_goal_pos is None:
            filtered_goal_pos = time_goal_pos
        else:
            filtered_goal_pos = (
                self.filtered_goal_pos[0]
                + self.goal_smoothing_alpha * (time_goal_pos[0] - self.filtered_goal_pos[0]),
                self.filtered_goal_pos[1]
                + self.goal_smoothing_alpha * (time_goal_pos[1] - self.filtered_goal_pos[1]),
                self.filtered_goal_pos[2]
                + self.goal_smoothing_alpha * (time_goal_pos[2] - self.filtered_goal_pos[2]),
            )
            filtered_goal_pos = self._limit_vector_step(
                self.filtered_goal_pos, filtered_goal_pos, self.max_goal_speed * dt
            )

        if (
            self.keep_goal_on_circle
            and self.prediction_mode == "intercept"
            and self.prediction_model in ("circle", "line")
        ):
            if self.prediction_model == "circle":
                filtered_goal_pos = self._project_goal_to_circle(filtered_goal_pos)
            else:
                filtered_goal_pos = self._project_goal_to_line(filtered_goal_pos)

        self.filtered_goal_pos = filtered_goal_pos
        return filtered_goal_pos, filtered_t_go

    def _project_goal_to_line(self, goal_pos):
        z = self.target_height if self.target_height > 0.0 else goal_pos[2]
        return (
            self._clamp(
                goal_pos[0],
                self.target_center_x - self.target_radius,
                self.target_center_x + self.target_radius,
            ),
            self.target_center_y,
            z,
        )

    def _compute_prediction(self, target_pos, target_vel):
        if not self.use_prediction:
            return target_pos, 0.0

        chaser_pos = None
        if self.latest_chaser_odom is not None:
            chaser_pos = self._point_tuple(self.latest_chaser_odom.pose.pose.position)

        mode = self.prediction_mode
        if mode == "fixed":
            return self._linear_future(target_pos, target_vel, self.prediction_time), self.prediction_time

        if chaser_pos is None:
            rospy.logwarn_throttle(
                3.0,
                "[TruthTracker] %s mode needs chaser odom; falling back to fixed prediction",
                mode,
            )
            return self._linear_future(target_pos, target_vel, self.prediction_time), self.prediction_time

        if mode == "adaptive":
            distance = self._distance(target_pos, chaser_pos)
            t_go = self._clamp_time(distance / self.assumed_chaser_speed)
            return self._linear_future(target_pos, target_vel, t_go), t_go

        if mode != "intercept":
            rospy.logwarn_throttle(
                5.0, "[TruthTracker] unknown prediction_mode '%s', using intercept", mode
            )

        if self.prediction_model == "linear":
            t_go = self._linear_intercept_time(target_pos, target_vel, chaser_pos)
            return self._linear_future(target_pos, target_vel, t_go), t_go

        t_go = self._clamp_time(
            self._distance(target_pos, chaser_pos) / self.assumed_chaser_speed
        )
        p_future = target_pos
        for _ in range(self.intercept_iterations):
            p_future = self._future_position(target_pos, target_vel, t_go)
            t_go = self._clamp_time(
                self._distance(p_future, chaser_pos) / self.assumed_chaser_speed
            )
        p_future = self._future_position(target_pos, target_vel, t_go)
        return p_future, t_go

    def _clamp_goal_jump(self, goal_pos):
        if self.max_goal_jump <= 0.0:
            return goal_pos

        if self.last_goal_pos is None:
            return goal_pos

        dx = goal_pos[0] - self.last_goal_pos[0]
        dy = goal_pos[1] - self.last_goal_pos[1]
        dz = goal_pos[2] - self.last_goal_pos[2]
        dist = self._speed(dx, dy, dz)
        if dist <= self.max_goal_jump:
            return goal_pos

        scale = self.max_goal_jump / dist
        rospy.logwarn_throttle(
            2.0,
            "[TruthTracker] goal jump %.2fm exceeds max_goal_jump %.2fm, clamping",
            dist,
            self.max_goal_jump,
        )
        return (
            self.last_goal_pos[0] + dx * scale,
            self.last_goal_pos[1] + dy * scale,
            self.last_goal_pos[2] + dz * scale,
        )

    def _build_sphere_marker(self, header, marker_id, ns, pose, color, scale):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration(1.0 / self.goal_update_rate * 1.5)
        return marker

    def _build_prediction_line_marker(self, header, target_pos, target_vel, goal_pos, t_go):
        marker = Marker()
        marker.header = header
        marker.ns = "uav_truth_tracker_prediction_path"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.65
        marker.color.b = 0.05
        marker.color.a = 0.9
        samples = (
            12
            if self.prediction_model in ("circle", "line") and t_go > 1e-3
            else 1
        )
        for i in range(samples + 1):
            t = t_go * float(i) / float(samples)
            p = self._future_position(target_pos, target_vel, t)
            marker.points.append(Point(x=p[0], y=p[1], z=p[2]))
        if marker.points[-1].x != goal_pos[0] or marker.points[-1].y != goal_pos[1]:
            marker.points.append(Point(x=goal_pos[0], y=goal_pos[1], z=goal_pos[2]))
        marker.lifetime = rospy.Duration(1.0 / self.goal_update_rate * 1.5)
        return marker

    def _build_chaser_goal_line_marker(self, header, chaser_pos, goal_pos):
        marker = Marker()
        marker.header = header
        marker.ns = "uav_truth_tracker_chaser_to_goal"
        marker.id = 3
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.75
        marker.points.append(Point(x=chaser_pos[0], y=chaser_pos[1], z=chaser_pos[2]))
        marker.points.append(Point(x=goal_pos[0], y=goal_pos[1], z=goal_pos[2]))
        marker.lifetime = rospy.Duration(1.0 / self.goal_update_rate * 1.5)
        return marker

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        vx, vy, vz = self._estimate_velocity(msg)
        self.latest_odom = msg
        self.latest_velocity = (vx, vy, vz)

        msg_stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        self.last_stamp = msg_stamp
        self.last_pos = (pos.x, pos.y, pos.z)

    def chaser_odom_callback(self, msg):
        self.latest_chaser_odom = msg

    def publish_goal(self, _event):
        if self.latest_odom is None:
            rospy.loginfo_throttle(3.0, "[TruthTracker] waiting for target odom")
            return

        msg = self.latest_odom
        target_pos = self._point_tuple(msg.pose.pose.position)
        target_vel = self.latest_velocity

        frame_id = self.goal_frame_id or msg.header.frame_id or "map"
        stamp = rospy.Time.now()

        raw_goal_pos, raw_t_go = self._compute_prediction(target_pos, target_vel)
        goal_pos, t_go = self._smooth_goal(target_pos, target_vel, raw_goal_pos, raw_t_go, stamp)
        goal_pos = self._clamp_goal_jump(goal_pos)

        goal = PoseStamped()
        goal.header.stamp = stamp
        goal.header.frame_id = frame_id
        goal.pose.position.x = goal_pos[0]
        goal.pose.position.y = goal_pos[1]
        goal.pose.position.z = goal_pos[2]

        yaw = (
            math.atan2(target_vel[1], target_vel[0])
            if self._speed(target_vel[0], target_vel[1], 0.0) > 1e-3
            else 0.0
        )
        qx, qy, qz, qw = self._yaw_to_quat(yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self.goal_pub.publish(goal)

        current_pose = PoseStamped()
        current_pose.header = goal.header
        current_pose.pose = msg.pose.pose
        current_marker = self._build_sphere_marker(
            goal.header,
            0,
            "uav_truth_tracker_current",
            current_pose.pose,
            (0.0, 0.8, 1.0, 0.85),
            self.marker_scale * 0.75,
        )
        predicted_marker = self._build_sphere_marker(
            goal.header,
            1,
            "uav_truth_tracker_prediction",
            goal.pose,
            (1.0, 0.2, 0.05, 0.9),
            self.marker_scale,
        )
        line_marker = self._build_prediction_line_marker(
            goal.header, target_pos, target_vel, goal_pos, t_go
        )
        self.marker_pub.publish(current_marker)
        self.marker_pub.publish(predicted_marker)
        self.marker_pub.publish(line_marker)
        self.last_goal_pos = goal_pos

        distance_to_goal = float("nan")
        if self.latest_chaser_odom is not None:
            chaser_pos = self._point_tuple(self.latest_chaser_odom.pose.pose.position)
            distance_to_goal = self._distance(goal_pos, chaser_pos)
            self.marker_pub.publish(
                self._build_chaser_goal_line_marker(goal.header, chaser_pos, goal_pos)
            )

        rospy.loginfo_throttle(
            2.0,
            "[TruthTracker] mode=%s model=%s raw_t=%.2f t_go=%.2f dist_goal=%.2f target=(%.2f %.2f %.2f) vel=(%.2f %.2f %.2f) raw_goal=(%.2f %.2f %.2f) goal=(%.2f %.2f %.2f)",
            self.prediction_mode,
            self.prediction_model,
            raw_t_go,
            t_go,
            distance_to_goal,
            target_pos[0],
            target_pos[1],
            target_pos[2],
            target_vel[0],
            target_vel[1],
            target_vel[2],
            raw_goal_pos[0],
            raw_goal_pos[1],
            raw_goal_pos[2],
            goal_pos[0],
            goal_pos[1],
            goal_pos[2],
        )


def main():
    rospy.init_node("truth_tracker_node")
    TruthTracker()
    rospy.spin()


if __name__ == "__main__":
    main()
