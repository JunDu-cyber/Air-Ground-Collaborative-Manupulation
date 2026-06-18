#!/usr/bin/env python3
"""Publish simple local-position setpoints for the target UAV."""

import math
import random

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry


class Uav1Motion:
    def __init__(self):
        ns = rospy.get_param("~mavros_ns", "/uav1/mavros").rstrip("/")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.mode = rospy.get_param("~mode", "circle").lower()
        self.height = rospy.get_param("~height", 3.0)
        self.radius = max(rospy.get_param("~radius", 3.0), 0.05)
        self.speed = max(rospy.get_param("~speed", 0.6), 0.0)
        self.center_x = rospy.get_param("~center_x", 2.0)
        self.center_y = rospy.get_param("~center_y", 0.0)
        self.z_amplitude = max(rospy.get_param("~z_amplitude", 0.5), 0.0)
        self.z_period = max(rospy.get_param("~z_period", 12.0), 0.1)
        self.z_min = rospy.get_param("~z_min", 2.0)
        self.z_max = rospy.get_param("~z_max", 5.0)
        if self.z_max < self.z_min:
            self.z_min, self.z_max = self.z_max, self.z_min
        self.figure8_x_amp = max(rospy.get_param("~figure8_x_amp", 3.0), 0.05)
        self.figure8_y_amp = max(rospy.get_param("~figure8_y_amp", 2.0), 0.05)
        self.x_min = rospy.get_param("~x_min", self.center_x - self.radius)
        self.x_max = rospy.get_param("~x_max", self.center_x + self.radius)
        self.y_min = rospy.get_param("~y_min", self.center_y - self.radius)
        self.y_max = rospy.get_param("~y_max", self.center_y + self.radius)
        if self.x_max < self.x_min:
            self.x_min, self.x_max = self.x_max, self.x_min
        if self.y_max < self.y_min:
            self.y_min, self.y_max = self.y_max, self.y_min
        self.waypoint_accept_radius = max(
            rospy.get_param("~waypoint_accept_radius", 0.5), 0.01
        )
        self.waypoint_hold_time = max(rospy.get_param("~waypoint_hold_time", 1.0), 0.0)
        self.auto_arm = rospy.get_param("~auto_arm", False)
        self.auto_offboard = rospy.get_param("~auto_offboard", False)
        self.start_delay = max(rospy.get_param("~start_delay", 0.0), 0.0)

        self.state = State()
        self.current_position = None
        self.current_waypoint = None
        self.waypoint_inside_since = None
        self.start_time = rospy.Time.now()
        self.motion_start_time = None

        self.state_sub = rospy.Subscriber(ns + "/state", State, self.state_cb, queue_size=10)
        self.odom_sub = rospy.Subscriber(
            ns + "/local_position/odom", Odometry, self.odom_cb, queue_size=10
        )
        self.setpoint_pub = rospy.Publisher(
            ns + "/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.set_mode_srv = rospy.ServiceProxy(ns + "/set_mode", SetMode)
        self.arming_srv = rospy.ServiceProxy(ns + "/cmd/arming", CommandBool)

        rospy.loginfo(
            "[Uav1Motion] ns=%s mode=%s center=(%.2f %.2f) height=%.2f radius=%.2f speed=%.2f z_amp=%.2f z_period=%.2f z_range=[%.2f %.2f] figure8_amp=(%.2f %.2f) waypoint_box=[%.2f %.2f]x[%.2f %.2f] accept=%.2f hold=%.2f start_delay=%.2f auto_offboard=%s auto_arm=%s",
            ns,
            self.mode,
            self.center_x,
            self.center_y,
            self.height,
            self.radius,
            self.speed,
            self.z_amplitude,
            self.z_period,
            self.z_min,
            self.z_max,
            self.figure8_x_amp,
            self.figure8_y_amp,
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.waypoint_accept_radius,
            self.waypoint_hold_time,
            self.start_delay,
            self.auto_offboard,
            self.auto_arm,
        )

    def state_cb(self, msg):
        self.state = msg

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.current_position = (p.x, p.y, p.z)

    @staticmethod
    def yaw_to_quat(yaw):
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def clamp_z(self, z):
        return min(max(z, self.z_min), self.z_max)

    def make_z_sine(self, elapsed):
        omega_z = 2.0 * math.pi / self.z_period
        return self.clamp_z(self.height + self.z_amplitude * math.sin(omega_z * elapsed))

    def make_circle_pose(self, elapsed):
        angular_speed = self.speed / self.radius
        angle = angular_speed * elapsed
        x = self.center_x + self.radius * math.cos(angle)
        y = self.center_y + self.radius * math.sin(angle)
        yaw = angle + math.pi / 2.0
        return x, y, yaw

    def make_line_pose(self, elapsed):
        if self.speed <= 1e-6:
            return self.center_x - self.radius, self.center_y, 0.0

        period = 4.0 * self.radius / self.speed
        phase = (elapsed % period) / period
        if phase < 0.5:
            alpha = phase * 2.0
            x = self.center_x - self.radius + alpha * 2.0 * self.radius
            yaw = 0.0
        else:
            alpha = (phase - 0.5) * 2.0
            x = self.center_x + self.radius - alpha * 2.0 * self.radius
            yaw = math.pi
        return x, self.center_y, yaw

    def make_circle_z_sine_pose(self, elapsed):
        x, y, yaw = self.make_circle_pose(elapsed)
        return x, y, self.make_z_sine(elapsed), yaw

    def make_figure8_3d_pose(self, elapsed):
        omega = self.speed / max(self.figure8_x_amp, self.figure8_y_amp, 0.05)
        phase = omega * elapsed
        x = self.center_x + self.figure8_x_amp * math.sin(phase)
        y = self.center_y + self.figure8_y_amp * math.sin(2.0 * phase)
        z = self.make_z_sine(elapsed)

        dx = self.figure8_x_amp * omega * math.cos(phase)
        dy = 2.0 * self.figure8_y_amp * omega * math.cos(2.0 * phase)
        yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-6 else 0.0
        return x, y, z, yaw

    def random_waypoint(self):
        return (
            random.uniform(self.x_min, self.x_max),
            random.uniform(self.y_min, self.y_max),
            random.uniform(self.z_min, self.z_max),
        )

    def make_random_waypoint_3d_pose(self):
        now = rospy.Time.now()
        if self.current_waypoint is None:
            self.current_waypoint = self.random_waypoint()

        if self.current_position is not None:
            dx = self.current_waypoint[0] - self.current_position[0]
            dy = self.current_waypoint[1] - self.current_position[1]
            dz = self.current_waypoint[2] - self.current_position[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance <= self.waypoint_accept_radius:
                if self.waypoint_inside_since is None:
                    self.waypoint_inside_since = now
                elif (now - self.waypoint_inside_since).to_sec() >= self.waypoint_hold_time:
                    self.current_waypoint = self.random_waypoint()
                    self.waypoint_inside_since = None
            else:
                self.waypoint_inside_since = None

        x, y, z = self.current_waypoint
        yaw = 0.0
        if self.current_position is not None:
            yaw = math.atan2(y - self.current_position[1], x - self.current_position[0])
        return x, y, self.clamp_z(z), yaw

    def build_setpoint(self):
        now = rospy.Time.now()
        if self.start_delay > 0.0:
            delay_elapsed = (now - self.start_time).to_sec()
            if delay_elapsed < self.start_delay:
                pose = PoseStamped()
                pose.header.stamp = now
                pose.header.frame_id = self.frame_id
                if self.current_position is not None:
                    pose.pose.position.x = self.current_position[0]
                    pose.pose.position.y = self.current_position[1]
                    pose.pose.position.z = self.current_position[2]
                else:
                    pose.pose.position.x = 0.0
                    pose.pose.position.y = 0.0
                    pose.pose.position.z = self.clamp_z(self.height)
                pose.pose.orientation.w = 1.0
                rospy.loginfo_throttle(
                    1.0,
                    "[Uav1Motion] holding current target position during start_delay %.2f/%.2f",
                    delay_elapsed,
                    self.start_delay,
                )
                return pose

        if self.motion_start_time is None:
            self.motion_start_time = now
            rospy.loginfo(
                "[Uav1Motion] target motion started after %.2fs delay",
                self.start_delay,
            )

        elapsed = (now - self.motion_start_time).to_sec()
        if self.mode == "line":
            x, y, yaw = self.make_line_pose(elapsed)
            z = self.clamp_z(self.height)
        elif self.mode == "circle":
            x, y, yaw = self.make_circle_pose(elapsed)
            z = self.clamp_z(self.height)
        elif self.mode == "circle_z_sine":
            x, y, z, yaw = self.make_circle_z_sine_pose(elapsed)
        elif self.mode == "figure8_3d":
            x, y, z, yaw = self.make_figure8_3d_pose(elapsed)
        elif self.mode == "random_waypoint_3d":
            x, y, z, yaw = self.make_random_waypoint_3d_pose()
        else:
            rospy.logwarn_throttle(
                5.0, "[Uav1Motion] unknown mode '%s', using circle", self.mode
            )
            x, y, yaw = self.make_circle_pose(elapsed)
            z = self.clamp_z(self.height)

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        qx, qy, qz, qw = self.yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def publish_setpoint(self):
        self.setpoint_pub.publish(self.build_setpoint())

    def maybe_request_offboard_and_arm(self):
        if self.auto_offboard:
            try:
                self.set_mode_srv(custom_mode="OFFBOARD")
                rospy.loginfo("[Uav1Motion] OFFBOARD requested")
            except rospy.ServiceException as exc:
                rospy.logerr("[Uav1Motion] OFFBOARD request failed: %s", exc)

        if self.auto_arm:
            try:
                self.arming_srv(True)
                rospy.loginfo("[Uav1Motion] arm requested")
            except rospy.ServiceException as exc:
                rospy.logerr("[Uav1Motion] arm request failed: %s", exc)

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and not self.state.connected:
            self.publish_setpoint()
            rate.sleep()

        for _ in range(60):
            if rospy.is_shutdown():
                return
            self.publish_setpoint()
            rate.sleep()

        self.maybe_request_offboard_and_arm()

        while not rospy.is_shutdown():
            self.publish_setpoint()
            rate.sleep()


def main():
    rospy.init_node("uav1_motion_node")
    Uav1Motion().run()


if __name__ == "__main__":
    main()
