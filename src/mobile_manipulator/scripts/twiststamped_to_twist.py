#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Relay the CMU pathFollower's TwistStamped command onto the Husky twist_mux.

The CMU local_planner's pathFollower publishes a geometry_msgs/TwistStamped
(body twist, frame_id "vehicle"). The Husky stack drives off geometry_msgs/Twist
via twist_mux, whose lowest-priority "external" input is the topic `cmd_vel`
(priority 1; teleop inputs joy_teleop/cmd_vel etc. preempt it). This node strips
the stamp and republishes the bare Twist so the planner goes through twist_mux
(teleop still preempts) rather than bypassing it.

  ~input  (TwistStamped)  default /cmd_vel_stamped   <- pathFollower (remapped)
  ~output (Twist)         default /cmd_vel            -> twist_mux external input
"""

import rospy
from geometry_msgs.msg import TwistStamped, Twist


def main():
    rospy.init_node('twiststamped_to_twist')
    input_topic = rospy.get_param('~input', '/cmd_vel_stamped')
    output_topic = rospy.get_param('~output', '/cmd_vel')

    pub = rospy.Publisher(output_topic, Twist, queue_size=1)

    def cb(msg):
        pub.publish(msg.twist)

    rospy.Subscriber(input_topic, TwistStamped, cb)
    rospy.loginfo('[twiststamped_to_twist] %s (TwistStamped) -> %s (Twist)',
                  input_topic, output_topic)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
