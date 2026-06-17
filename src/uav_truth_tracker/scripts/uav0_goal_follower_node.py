#!/usr/bin/env python3
"""Follow tracker goals with bounded local-position setpoint steps."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry


class Uav0GoalFollower:
    def __init__(self):
        ns = rospy.get_param("~mavros_ns", "/uav0/mavros").rstrip("/")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.goal_topic = rospy.get_param("~goal_topic", "/uav0/tracker/goal")
        self.odom_topic = rospy.get_param("~odom_topic", ns + "/local_position/odom")
        self.setpoint_topic = rospy.get_param(
            "~setpoint_topic", ns + "/setpoint_position/local"
        )
        self.max_speed = max(rospy.get_param("~max_speed", 1.5), 0.01)
        self.max_accel = max(rospy.get_param("~max_accel", 1.0), 0.01)
        self.accept_radius = max(rospy.get_param("~accept_radius", 0.5), 0.01)
        self.stop_at_goal = rospy.get_param("~stop_at_goal", False)
        self.min_pursuit_speed = max(rospy.get_param("~min_pursuit_speed", 0.35), 0.0)
        self.pursuit_slow_radius = max(
            rospy.get_param("~pursuit_slow_radius", 1.2), self.accept_radius
        )
        self.max_setpoint_lead = max(rospy.get_param("~max_setpoint_lead", 2.0), 0.1)
        self.keep_setpoint_between_uav_and_goal = rospy.get_param(
            "~keep_setpoint_between_uav_and_goal", True
        )
        self.command_yaw = rospy.get_param("~command_yaw", True)
        self.face_goal_before_move = rospy.get_param("~face_goal_before_move", True)
        self.yaw_align_threshold = max(rospy.get_param("~yaw_align_threshold", 0.35), 0.01)
        self.max_step = max(rospy.get_param("~max_step", 0.3), 0.01)
        self.max_tracking_distance = max(
            rospy.get_param("~max_tracking_distance", 20.0), self.max_step
        )
        self.hold_height_from_goal = rospy.get_param("~hold_height_from_goal", True)
        self.default_hover_x = rospy.get_param("~default_hover_x", 0.0)
        self.default_hover_y = rospy.get_param("~default_hover_y", 0.0)
        self.default_hover_z = rospy.get_param("~default_hover_z", 3.0)
        self.goal_timeout = rospy.get_param("~goal_timeout", 2.0)
        self.auto_arm = rospy.get_param("~auto_arm", False)
        self.auto_offboard = rospy.get_param("~auto_offboard", False)

        self.state = State()
        self.current_pose = None
        self.current_stamp = None
        self.last_goal = None
        self.last_goal_stamp = None
        self.last_setpoint = None
        self.last_publish_time = None
        self.virtual_setpoint = None
        self.command_velocity = [0.0, 0.0, 0.0]

        self.state_sub = rospy.Subscriber(ns + "/state", State, self.state_cb, queue_size=10)
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_cb, queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_cb, queue_size=10
        )
        self.setpoint_pub = rospy.Publisher(self.setpoint_topic, PoseStamped, queue_size=10)
        self.set_mode_srv = rospy.ServiceProxy(ns + "/set_mode", SetMode)
        self.arming_srv = rospy.ServiceProxy(ns + "/cmd/arming", CommandBool)

        rospy.loginfo(
            "[Uav0GoalFollower] goal=%s odom=%s setpoint=%s max_speed=%.2f max_accel=%.2f accept_radius=%.2f stop_at_goal=%s min_pursuit_speed=%.2f slow_radius=%.2f max_lead=%.2f keep_between=%s command_yaw=%s face_first=%s yaw_threshold=%.2f max_dist=%.2f hold_height_from_goal=%s auto_offboard=%s auto_arm=%s",
            self.goal_topic,
            self.odom_topic,
            self.setpoint_topic,
            self.max_speed,
            self.max_accel,
            self.accept_radius,
            self.stop_at_goal,
            self.min_pursuit_speed,
            self.pursuit_slow_radius,
            self.max_setpoint_lead,
            self.keep_setpoint_between_uav_and_goal,
            self.command_yaw,
            self.face_goal_before_move,
            self.yaw_align_threshold,
            self.max_tracking_distance,
            self.hold_height_from_goal,
            self.auto_offboard,
            self.auto_arm,
        )

    def state_cb(self, msg):
        self.state = msg

    def goal_cb(self, msg):
        self.last_goal = msg
        self.last_goal_stamp = rospy.Time.now()

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose
        self.current_stamp = msg.header.stamp

    @staticmethod
    def yaw_to_quat(yaw):
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    @staticmethod
    def quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def vector_norm(x, y, z):
        return math.sqrt(x * x + y * y + z * z)

    def get_dt(self, now):
        if self.last_publish_time is None:
            self.last_publish_time = now
            return 1.0 / 20.0

        dt = (now - self.last_publish_time).to_sec()
        self.last_publish_time = now
        if dt <= 1e-3:
            return 1.0 / 20.0
        return min(dt, 0.25)

    def reset_velocity(self):
        self.command_velocity = [0.0, 0.0, 0.0]

    def reset_virtual_setpoint_to_current(self):
        if self.current_pose is None:
            self.virtual_setpoint = None
            return
        pos = self.current_pose.position
        self.virtual_setpoint = [pos.x, pos.y, pos.z]

    def ensure_virtual_setpoint(self):
        if self.virtual_setpoint is None:
            self.reset_virtual_setpoint_to_current()

    def clamp_virtual_setpoint_lead(self, target=None):
        if self.current_pose is None or self.virtual_setpoint is None:
            return 0.0

        current = self.current_pose.position
        dx = self.virtual_setpoint[0] - current.x
        dy = self.virtual_setpoint[1] - current.y
        dz = self.virtual_setpoint[2] - current.z
        lead = self.vector_norm(dx, dy, dz)

        if self.keep_setpoint_between_uav_and_goal and target is not None:
            gx = target[0] - current.x
            gy = target[1] - current.y
            gz = target[2] - current.z
            goal_distance = self.vector_norm(gx, gy, gz)
            if goal_distance <= 1e-6:
                self.virtual_setpoint = [current.x, current.y, current.z]
                return 0.0

            ux = gx / goal_distance
            uy = gy / goal_distance
            uz = gz / goal_distance
            along = dx * ux + dy * uy + dz * uz
            along = min(max(along, 0.0), min(self.max_setpoint_lead, goal_distance))
            self.virtual_setpoint = [
                current.x + ux * along,
                current.y + uy * along,
                current.z + uz * along,
            ]
            return along

        if lead > self.max_setpoint_lead and lead > 1e-6:
            scale = self.max_setpoint_lead / lead
            self.virtual_setpoint = [
                current.x + dx * scale,
                current.y + dy * scale,
                current.z + dz * scale,
            ]
            return self.max_setpoint_lead
        return lead

    def has_fresh_goal(self):
        if self.last_goal is None or self.last_goal_stamp is None:
            return False
        return (rospy.Time.now() - self.last_goal_stamp).to_sec() <= self.goal_timeout

    def build_hold_pose(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id

        if self.current_pose is not None:
            pose.pose.position.x = self.current_pose.position.x
            pose.pose.position.y = self.current_pose.position.y
            pose.pose.position.z = self.current_pose.position.z
            pose.pose.orientation = self.current_pose.orientation
            self.reset_virtual_setpoint_to_current()
        elif self.last_setpoint is not None:
            pose.pose = self.last_setpoint.pose
        else:
            pose.pose.position.x = self.default_hover_x
            pose.pose.position.y = self.default_hover_y
            pose.pose.position.z = self.default_hover_z
            pose.pose.orientation.w = 1.0

        return pose

    def limit_velocity(self, desired_velocity, dt):
        dvx = desired_velocity[0] - self.command_velocity[0]
        dvy = desired_velocity[1] - self.command_velocity[1]
        dvz = desired_velocity[2] - self.command_velocity[2]
        dv_norm = self.vector_norm(dvx, dvy, dvz)
        max_dv = self.max_accel * dt
        if dv_norm > max_dv and dv_norm > 1e-6:
            scale = max_dv / dv_norm
            dvx *= scale
            dvy *= scale
            dvz *= scale

        vx = self.command_velocity[0] + dvx
        vy = self.command_velocity[1] + dvy
        vz = self.command_velocity[2] + dvz
        speed = self.vector_norm(vx, vy, vz)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx *= scale
            vy *= scale
            vz *= scale

        self.command_velocity = [vx, vy, vz]
        return self.command_velocity

    def build_follow_pose(self):
        now = rospy.Time.now()
        dt = self.get_dt(now)

        if self.current_pose is None or not self.has_fresh_goal():
            if self.last_goal is None:
                rospy.loginfo_throttle(3.0, "[Uav0GoalFollower] waiting for tracker goal")
            else:
                rospy.logwarn_throttle(3.0, "[Uav0GoalFollower] tracker goal timed out")
            self.reset_velocity()
            return self.build_hold_pose()

        self.ensure_virtual_setpoint()
        current = self.current_pose.position
        goal = self.last_goal.pose.position

        target_x = goal.x
        target_y = goal.y
        target_z = goal.z if self.hold_height_from_goal else current.z

        dx = target_x - current.x
        dy = target_y - current.y
        dz = target_z - current.z
        distance = self.vector_norm(dx, dy, dz)
        raw_distance = distance
        horizontal_distance = self.vector_norm(dx, dy, 0.0)
        desired_yaw = (
            math.atan2(dy, dx)
            if horizontal_distance > 1e-3
            else self.quat_to_yaw(self.current_pose.orientation)
        )
        current_yaw = self.quat_to_yaw(self.current_pose.orientation)
        yaw_error = self.wrap_angle(desired_yaw - current_yaw)
        yaw_ready = abs(yaw_error) <= self.yaw_align_threshold

        if distance > self.max_tracking_distance:
            scale = self.max_tracking_distance / distance
            dx *= scale
            dy *= scale
            dz *= scale
            distance = self.max_tracking_distance
            rospy.logwarn_throttle(
                2.0,
                "[Uav0GoalFollower] tracker goal exceeds max_tracking_distance, clamping pursuit vector",
            )

        if distance <= self.accept_radius and self.stop_at_goal:
            desired_velocity = (0.0, 0.0, 0.0)
            rospy.loginfo_throttle(
                2.0,
                "[Uav0GoalFollower] inside accept_radius %.2fm, smoothing to hover",
                self.accept_radius,
            )
        elif self.command_yaw and self.face_goal_before_move and not yaw_ready:
            desired_velocity = (0.0, 0.0, 0.0)
            self.reset_velocity()
            self.reset_virtual_setpoint_to_current()
            rospy.loginfo_throttle(
                1.0,
                "[Uav0GoalFollower] aligning yaw before move: err=%.2f rad threshold=%.2f",
                yaw_error,
                self.yaw_align_threshold,
            )
        else:
            if distance <= 1e-6:
                direction = (0.0, 0.0, 0.0)
                desired_speed = 0.0
            else:
                direction = (dx / distance, dy / distance, dz / distance)
                if self.stop_at_goal:
                    desired_speed = min(
                        self.max_speed, max(0.0, distance - self.accept_radius)
                    )
                elif distance <= self.pursuit_slow_radius:
                    desired_speed = min(self.max_speed, distance)
                else:
                    desired_speed = min(
                        self.max_speed,
                        max(self.min_pursuit_speed, distance),
                    )
            desired_velocity = (
                direction[0] * desired_speed,
                direction[1] * desired_speed,
                direction[2] * desired_speed,
            )

        vx, vy, vz = self.limit_velocity(desired_velocity, dt)
        step_x = vx * dt
        step_y = vy * dt
        step_z = vz * dt

        max_step_distance = self.max_speed * dt
        step_distance = self.vector_norm(step_x, step_y, step_z)
        if step_distance > max_step_distance and step_distance > 1e-6:
            scale = max_step_distance / step_distance
            step_x *= scale
            step_y *= scale
            step_z *= scale

        if raw_distance <= self.accept_radius and self.stop_at_goal:
            self.reset_velocity()
            self.reset_virtual_setpoint_to_current()
        elif self.virtual_setpoint is not None:
            self.virtual_setpoint[0] += step_x
            self.virtual_setpoint[1] += step_y
            self.virtual_setpoint[2] += step_z
            self.clamp_virtual_setpoint_lead((target_x, target_y, target_z))

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.frame_id
        if self.virtual_setpoint is None:
            pose.pose.position.x = current.x
            pose.pose.position.y = current.y
            pose.pose.position.z = current.z
        else:
            pose.pose.position.x = self.virtual_setpoint[0]
            pose.pose.position.y = self.virtual_setpoint[1]
            pose.pose.position.z = self.virtual_setpoint[2]

        if self.command_yaw:
            qx, qy, qz, qw = self.yaw_to_quat(desired_yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
        else:
            pose.pose.orientation = self.current_pose.orientation

        rospy.loginfo_throttle(
            1.0,
            "[Uav0GoalFollower] dist=%.2f yaw_err=%.2f goal=(%.2f %.2f %.2f) setpoint=(%.2f %.2f %.2f) vel=(%.2f %.2f %.2f) lead=%.2f dt=%.3f",
            raw_distance,
            yaw_error,
            target_x,
            target_y,
            target_z,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            vx,
            vy,
            vz,
            self.clamp_virtual_setpoint_lead((target_x, target_y, target_z)),
            dt,
        )
        return pose

    def publish_setpoint(self):
        self.last_setpoint = self.build_follow_pose()
        self.setpoint_pub.publish(self.last_setpoint)

    def maybe_request_offboard_and_arm(self):
        if self.auto_offboard:
            try:
                self.set_mode_srv(custom_mode="OFFBOARD")
                rospy.loginfo("[Uav0GoalFollower] OFFBOARD requested")
            except rospy.ServiceException as exc:
                rospy.logerr("[Uav0GoalFollower] OFFBOARD request failed: %s", exc)

        if self.auto_arm:
            try:
                self.arming_srv(True)
                rospy.loginfo("[Uav0GoalFollower] arm requested")
            except rospy.ServiceException as exc:
                rospy.logerr("[Uav0GoalFollower] arm request failed: %s", exc)

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
    rospy.init_node("uav0_goal_follower_node")
    Uav0GoalFollower().run()


if __name__ == "__main__":
    main()
