#!/usr/bin/env python3
"""Read-only MoveIt IK probe for collision-free loaded transport poses.

This helper never executes a trajectory.  It evaluates a small, explicit grid
of top-down TCP poses against the *current* planning scene, including the
attached landmine, and prints collision-free candidates for calibration.
"""

import itertools
import math

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState


ARM_JOINTS = (
    "ur5_shoulder_pan_joint",
    "ur5_shoulder_lift_joint",
    "ur5_elbow_joint",
    "ur5_wrist_1_joint",
    "ur5_wrist_2_joint",
    "ur5_wrist_3_joint",
)


def main():
    rospy.init_node("transport_pose_probe", anonymous=True)
    service_name = rospy.get_param("~compute_ik_service", "/compute_ik")
    frame = rospy.get_param("~frame", "ur5_base_link")
    group = rospy.get_param("~group", "ur5_arm")
    link = rospy.get_param("~tcp_link", "grasp_tcp")
    xs = rospy.get_param("~x_values", [0.28, 0.32, 0.36, 0.40, 0.44])
    ys = rospy.get_param("~y_values", [0.0])
    zs = rospy.get_param("~z_values", [-0.10, -0.05, 0.0, 0.05, 0.10])
    yaws_deg = rospy.get_param("~yaw_values_deg", [90.0, 0.0])

    rospy.wait_for_service(service_name, timeout=10.0)
    compute_ik = rospy.ServiceProxy(service_name, GetPositionIK)
    joints = rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
    seed = RobotState(joint_state=joints, is_diff=True)

    valid = []
    for x, y, z, yaw_deg in itertools.product(xs, ys, zs, yaws_deg):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        q = tft.quaternion_from_euler(math.pi, 0.0, math.radians(yaw_deg))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        request = GetPositionIKRequest()
        request.ik_request.group_name = group
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state = seed
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = rospy.Duration(0.75)
        response = compute_ik(request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            continue
        values = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        solution = [values[name] for name in ARM_JOINTS]
        elbow = abs(math.atan2(math.sin(solution[2]), math.cos(solution[2])))
        valid.append((x, y, z, yaw_deg, elbow, solution))

    if not valid:
        print("NO_COLLISION_FREE_TRANSPORT_POSE")
        return
    for x, y, z, yaw_deg, elbow, solution in valid:
        print(
            "VALID xyz=({:.3f},{:.3f},{:.3f}) yaw={:.1f} elbow={:.3f} joints={}".format(
                x, y, z, yaw_deg, elbow,
                "[{}]".format(", ".join("{:.5f}".format(v) for v in solution)),
            )
        )


if __name__ == "__main__":
    main()
