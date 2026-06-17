#!/usr/bin/env python3
"""Namespaced MAVROS OFFBOARD hover helper for dual-UAV bringup checks."""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class MavrosHover:
    def __init__(self):
        ns = rospy.get_param("~mavros_ns", "/uav0/mavros").rstrip("/")
        self.takeoff_z = rospy.get_param("~takeoff_z", 3.0)
        self.x = rospy.get_param("~x", 0.0)
        self.y = rospy.get_param("~y", 0.0)
        self.auto_arm = rospy.get_param("~auto_arm", False)
        self.auto_offboard = rospy.get_param("~auto_offboard", False)

        self.state = State()
        self.state_sub = rospy.Subscriber(ns + "/state", State, self.state_cb, queue_size=10)
        self.setpoint_pub = rospy.Publisher(
            ns + "/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.set_mode_srv = rospy.ServiceProxy(ns + "/set_mode", SetMode)
        self.arming_srv = rospy.ServiceProxy(ns + "/cmd/arming", CommandBool)

        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.pose.position.x = self.x
        self.pose.pose.position.y = self.y
        self.pose.pose.position.z = self.takeoff_z
        self.pose.pose.orientation.w = 1.0

        rospy.loginfo(
            "[MavrosHover] ns=%s setpoint=(%.1f %.1f %.1f) auto_offboard=%s auto_arm=%s",
            ns,
            self.x,
            self.y,
            self.takeoff_z,
            self.auto_offboard,
            self.auto_arm,
        )

    def state_cb(self, msg):
        self.state = msg

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

        if self.auto_offboard:
            try:
                self.set_mode_srv(custom_mode="OFFBOARD")
                rospy.loginfo("[MavrosHover] OFFBOARD requested")
            except rospy.ServiceException as exc:
                rospy.logerr("[MavrosHover] OFFBOARD request failed: %s", exc)

        if self.auto_arm:
            try:
                self.arming_srv(True)
                rospy.loginfo("[MavrosHover] arm requested")
            except rospy.ServiceException as exc:
                rospy.logerr("[MavrosHover] arm request failed: %s", exc)

        while not rospy.is_shutdown():
            self.publish_setpoint()
            rate.sleep()

    def publish_setpoint(self):
        self.pose.header.stamp = rospy.Time.now()
        self.setpoint_pub.publish(self.pose)


def main():
    rospy.init_node("mavros_hover_node")
    MavrosHover().run()


if __name__ == "__main__":
    main()
