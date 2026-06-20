#!/usr/bin/env python3

import copy
import sys
import math
import threading
import concurrent.futures
import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped/etc. with tf2_ros.Buffer.transform()
import moveit_commander
import numpy as np
from tf.transformations import (quaternion_from_euler, quaternion_matrix,
                                quaternion_multiply, quaternion_from_matrix)
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import std_srvs.srv
from cv_bridge import CvBridge

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Custom YOLO Service
from mobile_manipulator.srv import DetectObjects, DetectObjectsRequest

# GPD (Grasp Pose Detection) messages — used to request learned 6-DOF grasps.
from gpd_ros.msg import CloudSamples, CloudSources, GraspConfigList

class MasterControl:
    def __init__(self):

        rospy.loginfo("Connecting to move_base server...")
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server(rospy.Duration(10.0))

        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_group   = moveit_commander.MoveGroupCommander("ur5_arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("hand_e_gripper")
        self.scene       = moveit_commander.PlanningSceneInterface()

        self.tf_buffer  = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.bridge = CvBridge()
        self._filtered_cloud_pub = rospy.Publisher(
            "/scene_filtered_cloud", PointCloud2, queue_size=1, latch=True)

        # Cloud (cropped to the target object) fed to the GPD detect_grasps node.
        # GPD returns grasps on /clustered_grasps in the same frame (ur5_base_link).
        self._gpd_cloud_pub = rospy.Publisher(
            "/gpd_cloud", CloudSamples, queue_size=1)

        # MoveIt advertises clear_octomap on whichever node handle the
        # ClearOctomapService capability was initialised with. Default is
        # /clear_octomap (root); some configs put it under /move_group/.
        self._clear_octomap_srv_name = None
        for name in ("/clear_octomap", "/move_group/clear_octomap"):
            try:
                rospy.wait_for_service(name, timeout=2.0)
                self._clear_octomap_srv_name = name
                rospy.loginfo(f"[INIT] clear_octomap service: {name}")
                break
            except rospy.ROSException:
                continue
        if self._clear_octomap_srv_name is None:
            rospy.logwarn("[INIT] clear_octomap service not found; OctoMap will not be cleared.")
        rospy.sleep(1.0)  # let the planning scene interface connect

        # Floor plane keeps RRTConnect from routing the arm below the robot base.
        # In ur5_base_link the floor is ~35 cm below the origin.
        floor = PoseStamped()
        floor.header.frame_id = "ur5_base_link"
        floor.pose.position.z = -0.35
        floor.pose.orientation.w = 1.0
        self.scene.add_box("floor", floor, size=(5.0, 5.0, 0.02))

        rospy.loginfo("MCP Booted. Navigation, MoveIt, and TF2 are ready.")
        self.marker_pub = rospy.Publisher(
            "/debug_pregrasp_marker", Marker, queue_size=1
        )

    def _try_clear_octomap(self, label: str = "OCTOMAP") -> bool:
        """Best-effort OctoMap clear. Re-resolves the service each call so a
        stale persistent proxy can't silently drop the request."""
        if self._clear_octomap_srv_name is None:
            rospy.logwarn(f"[{label}] clear_octomap unavailable — skipping clear.")
            return False
        try:
            rospy.wait_for_service(self._clear_octomap_srv_name, timeout=1.0)
            srv = rospy.ServiceProxy(self._clear_octomap_srv_name, std_srvs.srv.Empty)
            srv()
            rospy.loginfo(f"[{label}] OctoMap cleared.")
            return True
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn(f"[{label}] clear_octomap failed: {e}")
            return False

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
        [ 0.00, -1.90,  0.70, 1.80, 1.57, 0.0],  # forward centre
        [ 0.52, -1.90,  0.70, 1.80, 1.57, 0.0],  # left  ~30°
        [-0.52, -1.90,  0.70, 1.80, 1.57, 0.0],  # right ~30°
        [ 1.05, -1.90,  0.70, 1.80, 1.57, 0.0],  # left  ~60°
        [-1.05, -1.90,  0.70, 1.80, 1.57, 0.0],  # right ~60°
        [ 0.00, -1.90,  0.70, 1.80, 1.57, 0.0],  # forward, look slightly down (near-base objects)
    ]

    def scan_with_arm(self, target_class: str) -> tuple:
        """
        Sweeps the UR5 arm through SCAN_JOINT_CONFIGS while running YOLO
        continuously in a parallel thread at camera frame rate (~50 fps).
        The arm stops immediately on the first positive detection.
        Returns (u, v) or (None, None).
        """
        found   = {'u': None, 'v': None, 'w': None}
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
                            found['w'] = int(det.width)
                            stop_ev.set()
                            rospy.loginfo(f"[SCAN] Detected '{target_class}' at pixel ({found['u']}, {found['v']}), bbox_w={found['w']}px. Stopping arm sweep.")
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
            # no stop() here — go(wait=True) already settled; stop() was triggering
            # a 0.5 s hold trajectory on every waypoint, causing the arm to "dance"
            self.arm_group.clear_pose_targets()

        stop_ev.set()
        self.arm_group.stop()
        t.join(timeout=2.0)

        if found['u'] is not None:
            rospy.loginfo(f"[SCAN] Found '{target_class}' at pixel ({found['u']}, {found['v']}), bbox_w={found['w']}px.")
        else:
            rospy.logwarn(f"[SCAN] '{target_class}' not found after full arm sweep.")
        return found['u'], found['v'], found['w']

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

    def get_3d_coordinates(self, u, v, bbox_width_px=None):
        """Returns (PoseStamped in ur5_base_link, diameter_m) or (None, 0.0) on failure."""
        cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=5.0)
        fx, cx, fy, cy = cam_info.K[0], cam_info.K[2], cam_info.K[4], cam_info.K[5]

        depth_msg = rospy.wait_for_message("/camera/depth/image_raw", Image, timeout=5.0)
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        # 7×7 patch, 20th-percentile depth → nearest visible surface (front of object).
        h, w = depth_image.shape[:2]
        r = 3
        patch = depth_image[max(0, v - r):min(h, v + r + 1),
                            max(0, u - r):min(w, u + r + 1)]
        valid = patch[(patch > 0.05) & (patch < 5.0) & ~np.isnan(patch)]
        if len(valid) == 0:
            rospy.logwarn(f"[DEPTH] No valid readings in patch around ({u},{v})")
            return None, 0.0
        z_near = float(np.percentile(valid, 20))

        diameter_m = 0.0
        if bbox_width_px and bbox_width_px > 0:
            radius = float(bbox_width_px) * z_near / (2.0 * fx)
            diameter_m = 2.0 * radius
            z_center = z_near + radius
            rospy.loginfo(f"[DEPTH] near={z_near:.3f} m  bbox_w={bbox_width_px}px "
                          f"→ r={radius:.3f} m  center={z_center:.3f} m")
        else:
            z_center = z_near
            rospy.loginfo(f"[DEPTH] z={z_near:.3f} m  ({len(valid)} valid pixels, 20th-pct)")

        x_cam = (u - cx) * z_center / fx
        y_cam = (v - cy) * z_center / fy

        target_pose = PoseStamped()
        target_pose.header.frame_id = "realsense_camera_optical_frame"
        target_pose.header.stamp = rospy.Time(0)
        target_pose.pose.position.x = x_cam
        target_pose.pose.position.y = y_cam
        target_pose.pose.position.z = z_center
        target_pose.pose.orientation.w = 1.0

        try:
            return self.tf_buffer.transform(target_pose, "ur5_base_link", rospy.Duration(3.0)), diameter_m
        except Exception as e:
            rospy.logwarn(f"[DEPTH] TF failed: {e}")
            return None, 0.0

    # Known-good arm pose for replanning: forward-facing scan position
    _READY_JOINTS = [0.00, -1.90,  0.70, 1.80, 1.57, 0.0]

    def _plan_and_go(self, pose: PoseStamped) -> bool:
        """Plan to a Cartesian pose from the current arm state (single attempt)."""
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(5)

        self.arm_group.set_goal_position_tolerance(0.01)
        self.arm_group.set_goal_orientation_tolerance(0.05)
        self.arm_group.set_goal_joint_tolerance(0.01)
        self.arm_group.set_pose_target(pose)
        ok = self.arm_group.go(wait=True)
        self.arm_group.clear_pose_targets()
        if ok:
            rospy.sleep(0.2)
        return ok

    def _go_ready(self):
        """Return to the known-good ready configuration (fast, joint-space)."""
        self.arm_group.set_joint_value_target(self._READY_JOINTS)
        self.arm_group.go(wait=True)
        self.arm_group.clear_pose_targets()
        rospy.sleep(0.2)

    def _update_scene_collisions(self, obj_pose: PoseStamped,
                                 target_class: str = "") -> tuple:
        """
        Retreat the camera, optionally re-detect the target for a fresh pose,
        capture depth, and publish the self-filtered point cloud to MoveIt's
        OctoMap sensor topic.

        Returns (refined_pose, refined_diameter): the fresher pose from re-detection
        if successful, otherwise the original obj_pose and 0.0.
        """
        # --- 1. Retreat camera along the approach axis ----------------------------
        px = obj_pose.pose.position.x
        py = obj_pose.pose.position.y
        horiz = math.hypot(px, py)
        dx, dy = (px / horiz, py / horiz) if horiz > 0.01 else (1.0, 0.0)

        ee = self.arm_group.get_current_pose()
        retreat = copy.deepcopy(ee)
        retreat.pose.position.x = ee.pose.position.x - dx * 0.25
        retreat.pose.position.y = ee.pose.position.y - dy * 0.25

        self.arm_group.set_planning_time(3.0)
        self.arm_group.set_goal_position_tolerance(0.01)
        self.arm_group.set_goal_orientation_tolerance(0.05)
        self.arm_group.set_goal_joint_tolerance(0.01)
        self.arm_group.set_pose_target(retreat)
        if not self.arm_group.go(wait=True):
            rospy.logwarn("[SCENE] Retreat failed — using current camera view.")
        self.arm_group.clear_pose_targets()
        rospy.sleep(0.35)

        # --- 1b. Re-detect while camera is still settled on the scene -------------
        refined_pose = obj_pose
        refined_diameter = 0.0
        if target_class:
            try:
                img = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=3.0)
                yolo_srv = rospy.ServiceProxy('/yolo/detect', DetectObjects)
                req = DetectObjectsRequest()
                req.image = img
                resp = yolo_srv(req)
                for det in resp.detections:
                    if det.class_name == target_class:
                        fresh, diam = self.get_3d_coordinates(
                            int(det.center_u), int(det.center_v), int(det.width))
                        if fresh is not None:
                            refined_pose = fresh
                            refined_diameter = diam
                            fp = fresh.pose.position
                            rospy.loginfo(
                                f"[SCENE] Re-detected '{target_class}' at "
                                f"({fp.x:.3f},{fp.y:.3f},{fp.z:.3f}), "
                                f"diameter={diam*1000:.1f} mm")
                        break
            except Exception as e:
                rospy.logwarn(f"[SCENE] Re-detection failed ({e}) — using original pose.")

        # --- 2. Capture depth + intrinsics ----------------------------------------
        try:
            cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=3.0)
            depth_msg = rospy.wait_for_message("/camera/depth/image_raw", Image, timeout=3.0)
        except rospy.ROSException:
            rospy.logwarn("[SCENE] Depth timeout — skipping scene update.")
            return

        depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        h, w = depth.shape
        fx, cx, fy, cy = cam_info.K[0], cam_info.K[2], cam_info.K[4], cam_info.K[5]

        # --- 3. Camera→base as a 4×4 matrix (enables vectorised batch transform) --
        try:
            tf_s = self.tf_buffer.lookup_transform(
                "ur5_base_link", "realsense_camera_optical_frame",
                rospy.Time(0), rospy.Duration(3.0))
        except Exception as e:
            rospy.logwarn(f"[SCENE] TF lookup failed: {e}")
            return

        T = quaternion_matrix([tf_s.transform.rotation.x,
                               tf_s.transform.rotation.y,
                               tf_s.transform.rotation.z,
                               tf_s.transform.rotation.w])
        T[0, 3] = tf_s.transform.translation.x
        T[1, 3] = tf_s.transform.translation.y
        T[2, 3] = tf_s.transform.translation.z

        # --- 4. Back-project subsampled depth pixels → ur5_base_link -------------
        step = 8
        vs, us = np.mgrid[0:h:step, 0:w:step]
        zs = depth[::step, ::step].ravel().astype(np.float64)
        us = us.ravel().astype(np.float64)
        vs = vs.ravel().astype(np.float64)

        # Minimum camera depth 0.25 m: filters out the gripper, which is right in
        # front of the lens and would otherwise appear as a false obstacle.
        valid = (zs > 0.25) & (zs < 3.0) & np.isfinite(zs)
        zs, us, vs = zs[valid], us[valid], vs[valid]
        if len(zs) < 10:
            rospy.logwarn("[SCENE] Too few valid depth pixels.")
            return

        xs_cam = (us - cx) * zs / fx
        ys_cam = (vs - cy) * zs / fy
        pts_cam = np.vstack([xs_cam, ys_cam, zs, np.ones(len(zs))])
        pts_base = (T @ pts_cam)[:3].T  # N×3 in ur5_base_link

        # --- 5. Self-observation filter -------------------------------------------
        # Use the refined pose (from re-detection) if available so the target
        # object exclusion sphere is centred on the freshest known position.
        ox = refined_pose.pose.position.x
        oy = refined_pose.pose.position.y
        oz = refined_pose.pose.position.z

        r_xy = np.hypot(pts_base[:, 0], pts_base[:, 1])
        dist_to_obj = np.sqrt((pts_base[:, 0] - ox) ** 2 +
                              (pts_base[:, 1] - oy) ** 2 +
                              (pts_base[:, 2] - oz) ** 2)
        env_mask = (
            (pts_base[:, 0] > 0.10) &          # exclude Husky chassis behind arm
            (r_xy > 0.20) &                     # exclude arm links near axis
            (pts_base[:, 2] > -0.25) &          # above floor
            (pts_base[:, 2] < oz + 0.60) &      # reasonable ceiling
            (dist_to_obj > 0.10)                # exclude the target object itself
        )
        pts_base = pts_base[env_mask]
        if len(pts_base) < 5:
            rospy.logwarn("[SCENE] No environment points after self-filter.")
            return

        # --- 6. Clear old OctoMap and publish filtered cloud ----------------------
        self._try_clear_octomap("SCENE")

        cloud_msg = pc2.create_cloud_xyz32(
            Header(frame_id="ur5_base_link", stamp=rospy.Time.now()),
            pts_base.tolist()
        )
        self._filtered_cloud_pub.publish(cloud_msg)
        rospy.sleep(0.5)  # give OctomapUpdater time to process the latched cloud
        rospy.loginfo(f"[SCENE] Published {len(pts_base)} pts to OctoMap sensor topic.")
        return refined_pose, refined_diameter

    # ── GPD (Grasp Pose Detection) integration ───────────────────────────────
    # Distance (m) the GPD hand-base center sits ahead of ur5_tool0 along the
    # approach axis. GPD's grasp `position` is the hand base; tool0 must sit this
    # far behind it so the fingers close around the sampled point. Tune in sim.
    GPD_TOOL_OFFSET = 0.12

    def _build_gpd_cloud(self, target_pose: PoseStamped):
        """Capture one depth frame, back-project to ur5_base_link, and crop to a
        box around the target object. Returns (points_Nx3_list, view_point) where
        view_point is the camera origin in ur5_base_link (geometry_msgs/Point),
        or (None, None) on failure. Mirrors the back-projection in
        _update_scene_collisions but keeps the target points (no self-filter)."""
        try:
            cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=3.0)
            depth_msg = rospy.wait_for_message("/camera/depth/image_raw", Image, timeout=3.0)
        except rospy.ROSException:
            rospy.logwarn("[GPD] Depth/camera_info timeout — cannot build cloud.")
            return None, None

        depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        h, w = depth.shape
        fx, cx, fy, cy = cam_info.K[0], cam_info.K[2], cam_info.K[4], cam_info.K[5]

        try:
            tf_s = self.tf_buffer.lookup_transform(
                "ur5_base_link", "realsense_camera_optical_frame",
                rospy.Time(0), rospy.Duration(3.0))
        except Exception as e:
            rospy.logwarn(f"[GPD] TF lookup failed: {e}")
            return None, None

        T = quaternion_matrix([tf_s.transform.rotation.x,
                               tf_s.transform.rotation.y,
                               tf_s.transform.rotation.z,
                               tf_s.transform.rotation.w])
        T[0, 3] = tf_s.transform.translation.x
        T[1, 3] = tf_s.transform.translation.y
        T[2, 3] = tf_s.transform.translation.z

        # Denser sampling than the octomap path (step=4) so GPD has enough surface.
        step = 4
        vs, us = np.mgrid[0:h:step, 0:w:step]
        zs = depth[::step, ::step].ravel().astype(np.float64)
        us = us.ravel().astype(np.float64)
        vs = vs.ravel().astype(np.float64)
        valid = (zs > 0.20) & (zs < 3.0) & np.isfinite(zs)
        zs, us, vs = zs[valid], us[valid], vs[valid]
        if len(zs) < 10:
            rospy.logwarn("[GPD] Too few valid depth pixels.")
            return None, None

        xs_cam = (us - cx) * zs / fx
        ys_cam = (vs - cy) * zs / fy
        pts_cam = np.vstack([xs_cam, ys_cam, zs, np.ones(len(zs))])
        pts_base = (T @ pts_cam)[:3].T  # N×3 in ur5_base_link

        # Crop to a box around the target so GPD samples grasps on the object
        # (plus immediate support surface), not the whole scene.
        ox = target_pose.pose.position.x
        oy = target_pose.pose.position.y
        oz = target_pose.pose.position.z
        HALF = 0.20  # box half-extent (m)
        m = (
            (np.abs(pts_base[:, 0] - ox) < HALF) &
            (np.abs(pts_base[:, 1] - oy) < HALF) &
            (np.abs(pts_base[:, 2] - oz) < HALF)
        )
        pts_base = pts_base[m]
        if len(pts_base) < 50:
            rospy.logwarn(f"[GPD] Only {len(pts_base)} pts in crop box — too sparse.")
            return None, None

        view_point = Point(x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3]))
        return pts_base.tolist(), view_point

    def get_gpd_grasp(self, target_pose: PoseStamped, target_class: str = ""):
        """Ask GPD for a 6-DOF grasp on the target object. Returns
        (grasp_pose_stamped, approach_unit_np3, width_m) where grasp_pose is the
        ur5_tool0 target in ur5_base_link, or None to fall back to the heuristic."""
        pts, view_point = self._build_gpd_cloud(target_pose)
        if pts is None:
            return None

        # Build the CloudSamples request: cropped cloud + camera view_point +
        # sample points on/around the target so GPD searches there.
        header = Header(frame_id="ur5_base_link", stamp=rospy.Time.now())
        cloud_msg = pc2.create_cloud_xyz32(header, pts)

        sources = CloudSources()
        sources.cloud = cloud_msg
        sources.camera_source = [0] * len(pts)          # single camera (index 0)
        sources.view_points = [view_point]

        oc = target_pose.pose.position
        samples = [Point(x=oc.x, y=oc.y, z=oc.z)]
        for dx, dy, dz in ((0.03, 0, 0), (-0.03, 0, 0), (0, 0.03, 0),
                           (0, -0.03, 0), (0, 0, 0.03), (0, 0, -0.03)):
            samples.append(Point(x=oc.x + dx, y=oc.y + dy, z=oc.z + dz))

        req = CloudSamples()
        req.cloud_sources = sources
        req.samples = samples

        self._gpd_cloud_pub.publish(req)
        rospy.loginfo(f"[GPD] Published cloud ({len(pts)} pts) + {len(samples)} "
                      f"samples; waiting for grasps...")
        try:
            grasp_list = rospy.wait_for_message(
                "/clustered_grasps", GraspConfigList, timeout=20.0)
        except rospy.ROSException:
            rospy.logwarn("[GPD] No grasps received within timeout — using heuristic.")
            return None
        if not grasp_list.grasps:
            rospy.logwarn("[GPD] Empty grasp list — using heuristic.")
            return None

        # Pick the highest-scoring grasp whose position is near the target.
        ox, oy, oz = oc.x, oc.y, oc.z
        best, best_score = None, -1e9
        for g in grasp_list.grasps:
            d = math.sqrt((g.position.x - ox) ** 2 + (g.position.y - oy) ** 2
                          + (g.position.z - oz) ** 2)
            if d > 0.15:
                continue
            s = g.score.data
            if s > best_score:
                best, best_score = g, s
        if best is None:
            rospy.logwarn("[GPD] No grasp near the target — using heuristic.")
            return None

        # GPD orientation: R = [approach binormal axis] (columns). The Robotiq
        # tool0 convention used here is tool0 +Z = approach, tool0 +X = finger
        # closing direction (= binormal). So build tool0 rotation with columns
        # [binormal, axis, approach] and convert to a quaternion.
        approach = np.array([best.approach.x, best.approach.y, best.approach.z])
        binormal = np.array([best.binormal.x, best.binormal.y, best.binormal.z])
        axis     = np.array([best.axis.x,     best.axis.y,     best.axis.z])
        n = np.linalg.norm(approach)
        if n < 1e-6:
            rospy.logwarn("[GPD] Degenerate approach vector — using heuristic.")
            return None
        approach = approach / n

        R = np.eye(4)
        R[0:3, 0] = binormal
        R[0:3, 1] = axis
        R[0:3, 2] = approach
        q = quaternion_from_matrix(R)

        # tool0 sits GPD_TOOL_OFFSET behind the GPD hand-base center, along -approach.
        gx = best.position.x - approach[0] * self.GPD_TOOL_OFFSET
        gy = best.position.y - approach[1] * self.GPD_TOOL_OFFSET
        gz = best.position.z - approach[2] * self.GPD_TOOL_OFFSET

        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "ur5_base_link"
        grasp_pose.header.stamp = rospy.Time(0)
        grasp_pose.pose.position.x = gx
        grasp_pose.pose.position.y = gy
        grasp_pose.pose.position.z = gz
        grasp_pose.pose.orientation.x = q[0]
        grasp_pose.pose.orientation.y = q[1]
        grasp_pose.pose.orientation.z = q[2]
        grasp_pose.pose.orientation.w = q[3]

        width = float(best.width.data)
        rospy.loginfo(f"[GPD] Selected grasp score={best_score:.3f} "
                      f"width={width*1000:.0f} mm at "
                      f"tool0=({gx:.3f},{gy:.3f},{gz:.3f})")
        return grasp_pose, approach, width

    def execute_pick(self, base_pose: PoseStamped, target_class: str = "", obj_diameter: float = 0.0) -> bool:
        p = base_pose.pose.position
        rospy.loginfo(f"[PICK] Target in {base_pose.header.frame_id}: "
                      f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}")

        PRE_OFFSET = 0.18   # standoff (m) before the insertion stroke (both paths)

        # ── Try GPD first (captured now, while the camera is still on the object).
        # On any failure get_gpd_grasp returns None and we fall back to the
        # fixed-horizontal heuristic below.
        gpd = None
        try:
            gpd = self.get_gpd_grasp(base_pose, target_class)
        except Exception as e:
            rospy.logwarn(f"[PICK] GPD grasp failed ({e}) — using heuristic.")
            gpd = None

        if gpd is not None:
            # ── GPD path: learned 6-DOF grasp. Approach along GPD's approach axis.
            grasp_pose, approach, gpd_width = gpd
            obj_diameter = gpd_width
            grasp_pose.header.frame_id = base_pose.header.frame_id
            gp = grasp_pose.pose.position
            pre_grasp = copy.deepcopy(grasp_pose)
            pre_grasp.pose.position.x = gp.x - approach[0] * PRE_OFFSET
            pre_grasp.pose.position.y = gp.y - approach[1] * PRE_OFFSET
            pre_grasp.pose.position.z = gp.z - approach[2] * PRE_OFFSET
            rospy.loginfo("[PICK] using GPD grasp")
        else:
            # ── Heuristic path: horizontal approach from behind along the XY vector.
            horiz_dist = math.hypot(p.x, p.y)
            if horiz_dist < 0.15:
                rospy.logerr(f"[PICK] Object too close to arm base ({horiz_dist:.3f} m) — aborting.")
                return False
            dx, dy = p.x / horiz_dist, p.y / horiz_dist
            approach_yaw = math.atan2(p.y, p.x)

            # Orientation: tool Z pointing horizontally toward the object.
            # With sxyz extrinsic Euler, R = R_z(yaw)*R_y(pitch):
            #   pitch=+π/2 → EE-Z maps to [cos(yaw), sin(yaw), 0]  (toward object) ✓
            #   pitch=-π/2 → EE-Z maps to [-cos(yaw),-sin(yaw), 0] (away — wrong)
            # Then rotate 90° around EE-Z (intrinsic) so EE-X becomes horizontal
            # (fingers open left/right) instead of vertical (fingers would hit the table).
            q_approach = quaternion_from_euler(0.0, math.pi / 2, approach_yaw)
            q_roll90   = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
            q = quaternion_multiply(q_approach, q_roll90)

            # base_pose is the cylinder CENTRE (get_3d_coordinates already shifted from
            # near surface by the radius derived from bbox width).
            # tool0 must sit FINGER_REACH behind the centre so the fingertips land there.
            # 2F-140 is longer than the Hand-E: fingertip pad sits ~0.18 m ahead of
            # ur5_tool0 when open. Tune empirically against the sim if grasps land short/long.
            FINGER_REACH = 0.18   # 2F-140 fingertip distance from ur5_tool0, metres

            grasp_pose = copy.deepcopy(base_pose)
            grasp_pose.pose.position.x = p.x - dx * FINGER_REACH
            grasp_pose.pose.position.y = p.y - dy * FINGER_REACH
            grasp_pose.pose.orientation.x = q[0]
            grasp_pose.pose.orientation.y = q[1]
            grasp_pose.pose.orientation.z = q[2]
            grasp_pose.pose.orientation.w = q[3]

            pre_grasp = copy.deepcopy(base_pose)
            pre_grasp.pose.position.x = p.x - dx * (FINGER_REACH + PRE_OFFSET)
            pre_grasp.pose.position.y = p.y - dy * (FINGER_REACH + PRE_OFFSET)
            # z unchanged — horizontal approach stays at object height
            pre_grasp.pose.orientation.x = q[0]
            pre_grasp.pose.orientation.y = q[1]
            pre_grasp.pose.orientation.z = q[2]
            pre_grasp.pose.orientation.w = q[3]

        rospy.loginfo(f"[PICK] pre_grasp : x={pre_grasp.pose.position.x:.3f} y={pre_grasp.pose.position.y:.3f} z={pre_grasp.pose.position.z:.3f}")
        rospy.loginfo(f"[PICK] grasp_pose: x={grasp_pose.pose.position.x:.3f} y={grasp_pose.pose.position.y:.3f} z={grasp_pose.pose.position.z:.3f}")

        # Debug arrow in RViz: tail = pre-grasp, tip points toward grasp (= PRE_OFFSET travel)
        m = Marker()
        m.header = pre_grasp.header
        m.ns, m.id = "debug", 0
        m.type, m.action = Marker.ARROW, Marker.ADD
        m.pose = pre_grasp.pose
        m.scale.x = PRE_OFFSET
        m.scale.y = m.scale.z = 0.02
        m.color.a, m.color.r = 1.0, 1.0
        self.marker_pub.publish(m)

        fresh_pose, fresh_diameter = self._update_scene_collisions(base_pose, target_class)

        # Refine grasp/pre-grasp poses with the re-detected position returned
        # from the scene update (re-detection happened right after the retreat
        # while the camera view was still settled). This heuristic recomputation
        # (XY approach + FINGER_REACH) does not apply to a GPD 6-DOF grasp, so it
        # is skipped on the GPD path — the octomap cloud above is still published.
        if gpd is None and fresh_pose is not base_pose:
            fp = fresh_pose.pose.position
            rospy.loginfo(
                f"[PICK] Pose refined: ({p.x:.3f},{p.y:.3f},{p.z:.3f}) → "
                f"({fp.x:.3f},{fp.y:.3f},{fp.z:.3f})"
            )
            if fresh_diameter > 0:
                obj_diameter = fresh_diameter
            horiz_dist_new = math.hypot(fp.x, fp.y)
            if horiz_dist_new >= 0.15:
                dx_new = fp.x / horiz_dist_new
                dy_new = fp.y / horiz_dist_new
                grasp_pose.pose.position.x = fp.x - dx_new * FINGER_REACH
                grasp_pose.pose.position.y = fp.y - dy_new * FINGER_REACH
                grasp_pose.pose.position.z = fp.z
                pre_grasp.pose.position.x = fp.x - dx_new * (FINGER_REACH + PRE_OFFSET)
                pre_grasp.pose.position.y = fp.y - dy_new * (FINGER_REACH + PRE_OFFSET)
                pre_grasp.pose.position.z = fp.z
                rospy.loginfo(f"[PICK] pre_grasp (refined): x={pre_grasp.pose.position.x:.3f} y={pre_grasp.pose.position.y:.3f} z={pre_grasp.pose.position.z:.3f}")
                rospy.loginfo(f"[PICK] grasp_pose (refined): x={grasp_pose.pose.position.x:.3f} y={grasp_pose.pose.position.y:.3f} z={grasp_pose.pose.position.z:.3f}")
            else:
                rospy.logwarn("[PICK] Refined pose too close to arm base — keeping original.")

        # # Clear the OctoMap before the arm moves so voxels near the pre_grasp
        # # position don't block planning.  The static floor box still guards below.
        # try:
        #     self._clear_octomap()
        # except rospy.ServiceException as e:
        #     rospy.logwarn(f"[PICK] clear_octomap failed: {e}")
        # rospy.sleep(0.15)

        rospy.loginfo("[PICK] Opening gripper...")
        self.gripper_group.set_joint_value_target([0.0])  # 2F-140: 0 rad = fully open
        self.gripper_group.go(wait=True)
        rospy.sleep(0.3)

        # Return to a neutral configuration before planning so RRTConnect starts
        # from a well-conditioned state rather than the constrained scan pose.
        rospy.loginfo("[PICK] Returning to ready pose before planning...")
        self._go_ready()

        # ── Stage 1: free-space plan to the standoff position ────────────────
        # RRTConnect can take any path here; the floor collision box prevents it
        # from routing under the robot/table.
        rospy.loginfo("[PICK] Free-space plan → pre-grasp standoff...")
        if not self._plan_and_go(pre_grasp):
            rospy.logerr("[PICK] Cannot reach pre-grasp standoff.")
            return False

        m = Marker()
        m.header = grasp_pose.header
        m.ns, m.id = "debug", 0
        m.type, m.action = Marker.ARROW, Marker.ADD
        m.pose = grasp_pose.pose
        m.scale.x = FINGER_REACH
        m.scale.y = m.scale.z = 0.02
        m.color.a, m.color.b = 1.0, 1.0
        self.marker_pub.publish(m)

        # ── Stage 2: straight-line Cartesian insertion ───────────────────────
        rospy.sleep(0.5)
        self.arm_group.set_start_state_to_current_state()

        planning_frame = self.arm_group.get_planning_frame()
        try:
            grasp_in_planning = self.tf_buffer.transform(
                grasp_pose, planning_frame, rospy.Duration(1.0)
            )
        except Exception as e:
            rospy.logerr(f"[PICK] Cannot transform grasp_pose to planning frame '{planning_frame}': {e}")
            return False
        rospy.loginfo(f"[PICK] Cartesian insertion — computing path... "
                      f"target in {planning_frame}: "
                      f"x={grasp_in_planning.pose.position.x:.3f} "
                      f"y={grasp_in_planning.pose.position.y:.3f} "
                      f"z={grasp_in_planning.pose.position.z:.3f}")
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as ex:
            fut = ex.submit(
                self.arm_group.compute_cartesian_path,
                [grasp_in_planning.pose], 0.005, False,  # eef_step, avoid_collisions
            )
            try:
                plan, fraction = fut.result(timeout=15.0)
            except concurrent.futures.TimeoutError:
                rospy.logerr("[PICK] compute_cartesian_path timed out — aborting.")
                self.arm_group.set_joint_value_target(self._READY_JOINTS)
                self.arm_group.go(wait=True)
                self.arm_group.clear_pose_targets()
                return False
        rospy.loginfo(f"[PICK] Cartesian path {fraction*100:.0f}% complete.")
        if fraction < 0.9:
            rospy.logerr("[PICK] Cartesian path incomplete — aborting.")
            self.arm_group.set_joint_value_target(self._READY_JOINTS)
            self.arm_group.go(wait=True)
            self.arm_group.clear_pose_targets()
            return False

        rospy.loginfo("[PICK] Retiming trajectory...")
        plan = self.arm_group.retime_trajectory(
            self.arm_group.get_current_state(), plan,
            velocity_scaling_factor=0.15,
            acceleration_scaling_factor=0.15,
        )
        if not plan.joint_trajectory.points:
            rospy.logerr("[PICK] Retime produced empty trajectory — aborting.")
            self.arm_group.set_joint_value_target(self._READY_JOINTS)
            self.arm_group.go(wait=True)
            self.arm_group.clear_pose_targets()
            return False
        duration = plan.joint_trajectory.points[-1].time_from_start.to_sec()
        rospy.loginfo(f"[PICK] Executing insertion ({duration:.1f} s expected)...")
        self.arm_group.execute(plan, wait=True)
        rospy.sleep(0.3)

        # Compute gripper target from detected object diameter.
        # 2F-140: single revolute finger_joint. 0 rad = fully open (≈140 mm gap),
        # GRIPPER_MAX_RAD ≈ fully closed (0 mm gap). The finger gap is roughly
        # linear in the joint angle: gap ≈ STROKE * (1 - angle/MAX_RAD), so to
        # squeeze an object of diameter d we aim for a gap a little below d.
        GRIPPER_STROKE  = 0.140   # max finger gap (m) at finger_joint = 0
        GRIPPER_MAX_RAD = 0.70    # finger_joint angle (rad) at full close
        GRIP_SQUEEZE    = 0.010   # close this much tighter than the object for a firm hold
        if obj_diameter > 0:
            target_gap = max(0.0, min(GRIPPER_STROKE, obj_diameter - GRIP_SQUEEZE))
            gripper_angle = GRIPPER_MAX_RAD * (1.0 - target_gap / GRIPPER_STROKE)
        else:
            gripper_angle = 0.45  # fallback partial close when no vision estimate
        gripper_angle = max(0.0, min(GRIPPER_MAX_RAD, gripper_angle))
        rospy.loginfo(f"[PICK] Closing gripper to {gripper_angle:.3f} rad "
                      f"(obj diameter {obj_diameter*1000:.1f} mm)...")
        self.gripper_group.set_joint_value_target([gripper_angle])
        self.gripper_group.go(wait=True)
        rospy.sleep(0.5)
        self._try_clear_octomap("PICK")
        rospy.sleep(0.15)
        self._go_ready()
        return True

    # Fixed drop joint configuration: arm extended in front of the robot.
    _PLACE_JOINTS = [0.0, -1.0, 1.0, 0.0, 1.57, 0.0]

    def execute_place(self, map_x: float = 0.0, map_y: float = 0.0,
                      surface_height: float = 0.0) -> bool:
        """Release the held object at a fixed pose in front of the robot.

        The map_x, map_y, and surface_height arguments are ignored — the drop
        configuration is a hard-coded joint-space target so the robot always
        places objects directly in front of itself.
        """
        rospy.loginfo(f"[PLACE] Moving arm to fixed place joints: {self._PLACE_JOINTS}")
        self._try_clear_octomap("PLACE")
        rospy.sleep(0.3)  # let the planning scene update propagate

        self.arm_group.set_planning_time(15.0)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_goal_joint_tolerance(0.01)
        self.arm_group.set_joint_value_target(self._PLACE_JOINTS)
        if not self.arm_group.go(wait=True):
            rospy.logerr("[PLACE] Could not reach place joint configuration.")
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            return False
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        rospy.loginfo("[PLACE] Releasing object...")
        self.gripper_group.set_joint_value_target([0.0])  # 2F-140: 0 rad = fully open
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()

        # Retreat to ready so the robot can drive away safely
        self.arm_group.set_joint_value_target(self._READY_JOINTS)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return True