#!/usr/bin/env python3

import sys
import math
import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import pandas as pd

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

# Custom YOLO Service
from mobile_manipulator.srv import DetectObjects, DetectObjectsRequest

class MasterControl:
    def __init__(self):

        rospy.loginfo("Connecting to move_base server...")
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server(rospy.Duration(10.0))

        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_group = moveit_commander.MoveGroupCommander("ur5_arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("hand_e_gripper")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.bridge = CvBridge()
        rospy.loginfo("MCP Booted. Navigation, MoveIt, and TF2 are ready.")

    def move_base_to(self, target_x, target_y, target_yaw_degrees):
        rospy.loginfo(f"Navigating base to X:{target_x}, Y:{target_y}, Yaw:{target_yaw_degrees}°...")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.position.z = 0.0

        yaw_rad = math.radians(target_yaw_degrees)
        q = quaternion_from_euler(0, 0, yaw_rad)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.nav_client.send_goal(goal)
        wait = self.nav_client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            return False
        return self.nav_client.get_state() == actionlib.GoalStatus.SUCCEEDED

    def call_yolo_service(self, target_class="cup"):
        rospy.loginfo(f"Waiting for YOLO service to find '{target_class}'...")
        rospy.wait_for_service('/yolo/detect')

        try:
            image_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
            yolo_service = rospy.ServiceProxy('/yolo/detect', DetectObjects)
            request = DetectObjectsRequest()
            request.image = image_msg

            response = yolo_service(request)

            if not response.detections:
                return None, None

            for det in response.detections:
                if det.class_name == target_class:
                    return int(det.center_u), int(det.center_v)

            return None, None

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None, None

    def get_3d_coordinates(self, u, v):
        cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=5.0)
        fx, cx, fy, cy = cam_info.K[0], cam_info.K[2], cam_info.K[4], cam_info.K[5]

        depth_msg = rospy.wait_for_message("/camera/depth/image_raw", Image, timeout=5.0)
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        z_depth = depth_image[v, u]

        if z_depth == 0 or pd.isna(z_depth):
            return None

        x_cam = (u - cx) * z_depth / fx
        y_cam = (v - cy) * z_depth / fy
        z_cam = z_depth

        target_pose = PoseStamped()
        target_pose.header.frame_id = "realsense_camera_optical_frame"
        target_pose.pose.position.x = x_cam
        target_pose.pose.position.y = y_cam
        target_pose.pose.position.z = z_cam
        target_pose.pose.orientation.w = 1.0

        try:
            self.tf_buffer.can_transform("ur5_base_link", "realsense_camera_optical_frame", rospy.Time(0), rospy.Duration(3.0))
            return self.tf_buffer.transform(target_pose, "ur5_base_link")
        except Exception:
            return None

    def execute_pick(self, base_pose):
        rospy.loginfo("Opening Gripper...")
        self.gripper_group.set_joint_value_target([0.0, 0.0])
        self.gripper_group.go(wait=True)

        rospy.loginfo("Moving Arm to Pre-Grasp Pose...")
        base_pose.pose.position.z += 0.15
        base_pose.pose.orientation.x = 0.0
        base_pose.pose.orientation.y = 0.707
        base_pose.pose.orientation.z = 0.0
        base_pose.pose.orientation.w = 0.707

        self.arm_group.set_pose_target(base_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if success:
            rospy.loginfo("Moving down to grasp...")
            base_pose.pose.position.z -= 0.15
            self.arm_group.set_pose_target(base_pose)
            self.arm_group.go(wait=True)

            rospy.loginfo("Closing Gripper...")
            self.gripper_group.set_joint_value_target([0.025, 0.025])
            self.gripper_group.go(wait=True)
            return True
        return False