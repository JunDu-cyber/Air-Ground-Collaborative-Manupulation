#!/usr/bin/env python3

import sys
import math
import threading
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

    def get_current_pose(self):
        """Returns (x, y, yaw_rad) in the map frame, or None on TF failure."""
        try:
            from tf.transformations import euler_from_quaternion
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(0), rospy.Duration(2.0)
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return t.x, t.y, yaw
        except Exception as e:
            rospy.logwarn(f"[POSE] Could not get current pose: {e}")
            return None

    # Arm joint configs for scanning: [shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3]
    #
    # Camera frame has Z-axis pointing forward (into the scene).
    # Goal: keep the forearm roughly horizontal and set wrist_1 so the camera Z
    # sweeps forward at table/shelf height, not at the floor.
    #
    # shoulder_lift=-0.8, elbow=1.2  → forearm angled forward and slightly up
    # wrist_1=-0.4                   → small downward tilt; camera Z ≈ horizontal
    # wrist_2=-1.57                  → roll to keep camera upright
    #
    # Tune shoulder_lift / elbow / wrist_1 in RViz if the camera still dips.
    SCAN_JOINT_CONFIGS = [
        [ 0.00, -0.80,  1.20, -0.40, -1.57, 0.0],  # forward centre
        [ 0.52, -0.80,  1.20, -0.40, -1.57, 0.0],  # left  ~30°
        [-0.52, -0.80,  1.20, -0.40, -1.57, 0.0],  # right ~30°
        [ 1.05, -0.80,  1.20, -0.40, -1.57, 0.0],  # left  ~60°
        [-1.05, -0.80,  1.20, -0.40, -1.57, 0.0],  # right ~60°
        [ 0.00, -1.10,  1.40, -0.30, -1.57, 0.0],  # forward, look slightly down (near-base objects)
    ]

    def scan_with_arm(self, target_class: str) -> tuple:
        """
        Sweeps the UR5 arm through SCAN_JOINT_CONFIGS while running YOLO
        continuously in a parallel thread at camera frame rate (~50 fps).
        The arm stops immediately on the first positive detection.
        Returns (u, v) or (None, None).
        """
        found   = {'u': None, 'v': None}
        stop_ev = threading.Event()

        def _detect_loop():
            rospy.wait_for_service('/yolo/detect')
            proxy = rospy.ServiceProxy('/yolo/detect', DetectObjects)
            while not stop_ev.is_set() and not rospy.is_shutdown():
                try:
                    img = rospy.wait_for_message(
                        "/camera/color/image_raw", Image, timeout=0.05
                    )
                    req = DetectObjectsRequest()
                    req.image = img
                    resp = proxy(req)
                    for det in resp.detections:
                        if det.class_name == target_class:
                            found['u'] = int(det.center_u)
                            found['v'] = int(det.center_v)
                            stop_ev.set()
                            return
                except Exception:
                    pass

        rospy.loginfo(f"[SCAN] Continuous YOLO scan for '{target_class}' during arm sweep...")
        t = threading.Thread(target=_detect_loop, daemon=True)
        t.start()

        for i, joints in enumerate(self.SCAN_JOINT_CONFIGS):
            if stop_ev.is_set():
                break
            rospy.loginfo(f"[SCAN] Arm pose {i+1}/{len(self.SCAN_JOINT_CONFIGS)}...")
            self.arm_group.set_joint_value_target(joints)
            self.arm_group.go(wait=True)   # YOLO thread runs freely during motion
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()

        stop_ev.set()
        self.arm_group.stop()
        t.join(timeout=2.0)

        if found['u'] is not None:
            rospy.loginfo(f"[SCAN] Found '{target_class}' at pixel ({found['u']}, {found['v']}).")
        else:
            rospy.logwarn(f"[SCAN] '{target_class}' not found after full arm sweep.")
        return found['u'], found['v']

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