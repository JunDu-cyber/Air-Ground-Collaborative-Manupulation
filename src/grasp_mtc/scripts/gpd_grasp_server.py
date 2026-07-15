#!/usr/bin/env python3
"""GPD behind the GetGrasps seam — grasp_source:=gpd.

Same service as analytic_grasp_server.py (/get_grasps -> ranked grasp_tcp poses), so grasp_task
does not change by one line. That is the whole point of the seam: the A/B then measures the
DETECTOR, not the plumbing.

WHAT THIS NODE ACTUALLY DOES, AND WHY IT IS SHAPED THIS WAY
-----------------------------------------------------------
TWO segmentations, doing two different jobs:

    the MINE   (red disc + yellow detonator, or landmine.onnx's mask) — published on
               ~segmented_cloud for eyeballing, and it is what tells us a mine is really there.

    the DETONATOR (the yellow block) — THIS is the cloud GPD actually receives, and it is the
               whole trick: GPD searches the cloud it is given, so a cloud that contains only the
               detonator cannot yield a grasp anywhere else. It will never propose one across the
               136 mm disc, which is wider than the 2F-140's usable aperture and is not the part
               we were asked to pick.

The obvious way to say that is CloudSamples ("the points at which the grasp detector should
search"), and it is what this node sent first. gpd_ros's CloudSamples path SEGFAULTS on a
message proven well-formed — see husky_ur5_gpd.launch. So we say the same thing with a plain
cloud, which works. The cost: GPD cannot see the disc, so it cannot reason about the fingers
hitting it. That check does not vanish, it MOVES — the planning scene carries the mine as
disc + detonator, and ComputeIK throws out any grasp whose gripper would hit the disc. GPD
proposes; MoveIt disposes.

The mine mask can come from colour or from models/landmine.onnx (~segmentation). It is COLOUR by
default, and that is measured, not preferred — see the note on self.seg.

THE ADAPTER (the thing the legacy code got wrong three times over)
-----------------------------------------------------------------
GPD returns, per grasp:
    position  — the hand's BOTTOM/BASE centre, NOT the point between the fingers
    approach, binormal, axis  — an orthonormal basis, R = [approach binormal axis]

Legacy master_control.py applied that basis directly to ur5_tool0 and then slid it back along
the approach by a guessed GPD_TOOL_OFFSET=0.12. Three compounding frame errors (tool0's axes are
NOT the gripper's — it is bolted on with rpy="0 -1.57 1.57"), and it is why every GPD grasp came
out wrong.

Here it is a pure change of basis, because grasp_tcp was defined to make it one:
    grasp_tcp: +Z = approach, +X = finger closing (= GPD binormal), +Y = GPD axis
    => R_tcp = [binormal | axis | approach]   (columns)
and the position is moved from the hand base to the JAW CENTRE-LINE by half the hand depth:
    tcp = position + approach * (hand_depth/2)      hand_depth = 0.05 in husky_ur5_gpd.cfg
No magic scalars survive. If the markers do not land between the pads, the fix is here, in one
place, and it is visible: ~candidates is published for exactly that check.
"""
import math
import struct
import sys

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32, Int64
from tf.transformations import quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray

import cv2

from gpd_ros.msg import CloudSamples, CloudSources, GraspConfigList
from grasp_mtc.srv import GetGrasps, GetGraspsResponse
from mobile_manipulator.srv import DetectLandmines

# Yellow detonator, red disc (red wraps around hue 0, hence two bands). Same thresholds as
# landmine_detector.py -- if you retune them, retune them there too.
YELLOW_LO, YELLOW_HI = (20, 120, 120), (35, 255, 255)
RED_LO_A, RED_HI_A = (0, 120, 80), (10, 255, 255)
RED_LO_B, RED_HI_B = (170, 120, 80), (180, 255, 255)

# From gpd_ros/config/husky_ur5_gpd.cfg. GPD's `position` is the hand BASE centre; the jaw
# centre-line (which is what grasp_tcp is) sits half a hand-depth further along the approach.
HAND_DEPTH = 0.05


class GpdGraspServer(object):
    def __init__(self):
        self.frame = rospy.get_param('~planning_frame', 'base_link')
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.info_topic = rospy.get_param('~info_topic', '/camera/color/camera_info')
        self.tcp_offset = float(rospy.get_param('~tcp_offset', HAND_DEPTH / 2.0))
        # 25 s was too tight and it cost us a run: GPD had already found 1014 candidates on the
        # detonator and was scoring them with its LeNet when we gave up on it. It runs the CNN on
        # the CPU here, so scoring a thousand candidates is simply slow. Wait properly.
        self.timeout = float(rospy.get_param('~gpd_timeout', 120.0))
        self.max_grasps = int(rospy.get_param('~max_grasps', 8))

        # WHERE THE MINE MASK COMES FROM.
        #
        #   color : red disc + yellow detonator. Exact on this prop, and the DEFAULT.
        #   onnx  : models/landmine.onnx (YOLO11s-seg).
        #
        # onnx is not the default, and that is a MEASURED decision, not a preference. The model
        # was trained on photographs of real mines; ours is a flat-shaded red disc with a yellow
        # cube on asphalt. Fed a Gazebo frame with the mine plainly in view (4046 yellow px,
        # 13187 red px), its best score anywhere in the output tensor was 0.0012, and its best
        # box was a 28x7 px sliver at the bottom of the frame -- nowhere near the mine. The
        # wrapper is not at fault: the decode was verified (the raw boxes are plausible pixel
        # coordinates), the letterbox is correct, the channel order is RGB. The model simply
        # cannot see this prop, and no threshold fixes that.
        #
        # Keep the onnx path: the moment landmine.onnx is fine-tuned on sim renders (or the prop
        # is made photoreal), flipping this back is a one-word change and everything downstream
        # is already built and tested.
        self.seg = str(rospy.get_param('~segmentation', 'color')).lower()
        self.onnx = None

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        if self.seg == 'onnx':
            rospy.wait_for_service('/landmine/detect')
            self.onnx = rospy.ServiceProxy('/landmine/detect', DetectLandmines)

        # A PLAIN PointCloud2, not CloudSamples. gpd_ros's CloudSamples path segfaults on a
        # perfectly well-formed message (its initCloudCamera writes out of bounds; see
        # gpd_ros/launch/husky_ur5_gpd.launch). The plain-cloud path is the one that works, and
        # we get "search only the detonator" by SENDING only the detonator.
        self.cloud_pub = rospy.Publisher('/gpd_cloud', PointCloud2, queue_size=1, latch=True)
        self.viz = rospy.Publisher('~candidates', MarkerArray, queue_size=1, latch=True)
        # TWO CLOUDS, AND ONLY ONE OF THEM IS REAL, so name them for what they are. The old
        # single '~segmented_cloud' showed the whole mine and looked, reasonably enough, like
        # the thing being handed to GPD. It is not.
        #
        #   ~cloud_to_gpd : THE DETONATOR POINTS. This is the cloud GPD actually receives, and
        #                   the reason it can never propose a grasp across the disc.
        #   ~cloud_mine   : the whole mine (red disc + yellow block). CONTEXT ONLY. Nothing
        #                   consumes it; it exists so you can see what the segmentation found.
        self.cloud_to_gpd = rospy.Publisher('~cloud_to_gpd', PointCloud2, queue_size=1,
                                            latch=True)
        self.cloud_mine = rospy.Publisher('~cloud_mine', PointCloud2, queue_size=1, latch=True)

        # /detect_grasps/clustered_grasps, NOT /clustered_grasps.
        #
        # gpd_ros advertises with a PRIVATE NodeHandle (ros::NodeHandle("~")), so its documented
        # "clustered_grasps" topic actually lands under the node's own namespace. Subscribing to
        # the un-namespaced name fails in the most confusing way available: rospy CREATES
        # /clustered_grasps for you, `rostopic list` shows it, `rostopic info` shows a subscriber
        # -- and nothing is ever published to it. Meanwhile GPD had already found the grasps and
        # printed them ("Grasp 0: 1308.78"), which reads as if the fault were downstream.
        self.grasp_topic = rospy.get_param('~grasp_topic', '/detect_grasps/clustered_grasps')
        self._grasps = None
        rospy.Subscriber(self.grasp_topic, GraspConfigList, self._grasps_cb, queue_size=1)

        rospy.Service('/get_grasps', GetGrasps, self._cb)
        rospy.loginfo('[gpd_grasp] serving /get_grasps in %s  (segmentation=%s; GPD is given '
                      'the DETONATOR cloud only)', self.frame, self.seg)

    def _grasps_cb(self, msg):
        self._grasps = msg

    # ---------- perception ----------
    def _capture(self):
        rgb = rospy.wait_for_message(self.rgb_topic, Image, timeout=5.0)
        depth = rospy.wait_for_message(self.depth_topic, Image, timeout=5.0)
        info = rospy.wait_for_message(self.info_topic, CameraInfo, timeout=3.0)
        return rgb, depth, info

    def _masks(self, rgb_msg, bgr):
        """(mine mask, detonator mask) as mono8, or (None, None).

        TWO masks, doing two DIFFERENT jobs -- this is the heart of the node:

            mine       -> becomes the CLOUD.   GPD needs the real local geometry, and that means
                          the 136 mm disc: the disc is what the fingers would collide with.
            detonator  -> becomes the SAMPLES. GPD's CloudSamples.samples is literally "the
                          points at which the grasp detector should search for grasp
                          candidates". Give it only these and it cannot propose a grasp across
                          the disc, which is wider than the 2F-140's usable aperture and is not
                          the part we were asked to pick.
        """
        kernel = np.ones((3, 3), np.uint8)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        det = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        det = cv2.morphologyEx(det, cv2.MORPH_OPEN, kernel)

        if self.onnx is not None:
            res = self.onnx(image=rgb_msg)
            if not res.detections:
                rospy.logwarn('[gpd_grasp] the model saw no landmine')
                return None, None
            best = max(range(len(res.detections)), key=lambda i: res.detections[i].confidence)
            mine = self.bridge.imgmsg_to_cv2(res.masks[best], 'mono8')
            rospy.loginfo('[gpd_grasp] onnx: landmine %.0f%% in %.0f ms',
                          res.detections[best].confidence * 100.0, res.inference_time_ms)
            det = cv2.bitwise_and(det, mine)   # yellow, but only inside a mine the net found
        else:
            # The mine IS the red disc plus the yellow detonator -- one rigid body, so one mask.
            red = cv2.bitwise_or(cv2.inRange(hsv, RED_LO_A, RED_HI_A),
                                 cv2.inRange(hsv, RED_LO_B, RED_HI_B))
            mine = cv2.bitwise_or(cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel), det)
            # Close the seam between disc and block so the cloud is one connected body rather
            # than two, which is what GPD's neighbourhood search expects.
            mine = cv2.morphologyEx(mine, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        if cv2.countNonZero(det) == 0:
            rospy.logwarn('[gpd_grasp] mine found, but no yellow detonator visible in it')
            return mine, None
        rospy.loginfo('[gpd_grasp] seg=%s: mine %d px, detonator %d px',
                      self.seg, cv2.countNonZero(mine), cv2.countNonZero(det))
        return mine, det

    def _unproject(self, mask, dep, info, stamp, frame_id):
        """Masked depth pixels -> Nx3 points in self.frame."""
        fx, cx, fy, cy = info.K[0], info.K[2], info.K[4], info.K[5]
        vs, us = np.nonzero(mask)
        if vs.size == 0:
            return np.zeros((0, 3))
        z = dep[vs, us].astype(np.float64)
        ok = np.isfinite(z) & (z > 0.05) & (z < 3.0)
        vs, us, z = vs[ok], us[ok], z[ok]
        if vs.size == 0:
            return np.zeros((0, 3))
        pts = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1)   # optical frame

        try:
            tf = self.tf_buffer.lookup_transform(self.frame, frame_id, rospy.Time(0),
                                                 rospy.Duration(2.0))
        except Exception as exc:  # noqa: BLE001
            rospy.logerr('[gpd_grasp] TF %s -> %s failed: %s', frame_id, self.frame, exc)
            return np.zeros((0, 3))
        t = tf.transform.translation
        q = tf.transform.rotation
        from tf.transformations import quaternion_matrix
        T = quaternion_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = [t.x, t.y, t.z]
        h = np.hstack([pts, np.ones((pts.shape[0], 1))])
        return h.dot(T.T)[:, :3], np.array([t.x, t.y, t.z])

    @staticmethod
    def _cloud_msg(pts, frame, stamp):
        """An XYZ + RGBA cloud. THE RGBA FIELD IS NOT DECORATION.

        GPD builds a pcl::PointCloud<PointXYZRGBA> and calls pcl::fromROSMsg, which matches
        fields BY NAME. Hand it an xyz-only cloud and PCL prints

            Failed to find match for field 'rgba'.

        ...and GPD then dies on the half-initialised cloud, silently, leaving its node
        registered with the master but no process behind it. The symptom is maddening: the topic
        is connected, the subscriber is listed, and nothing ever comes back on
        /clustered_grasps. So: always ship the colour field, even though the colour itself is
        meaningless to the grasp.
        """
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]
        white = 0xFFFFFFFF
        data = b''.join(struct.pack('<fffI', p[0], p[1], p[2], white) for p in pts)
        msg = PointCloud2()
        msg.header.frame_id = frame
        msg.header.stamp = stamp
        msg.height = 1
        msg.width = len(pts)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * len(pts)
        msg.is_dense = True
        msg.data = data
        return msg

    # ---------- the adapter ----------
    def _to_tcp(self, g):
        """GPD GraspConfig -> a grasp_tcp PoseStamped. See the module docstring."""
        a = np.array([g.approach.x, g.approach.y, g.approach.z])
        b = np.array([g.binormal.x, g.binormal.y, g.binormal.z])
        x = np.array([g.axis.x, g.axis.y, g.axis.z])
        p = np.array([g.position.x, g.position.y, g.position.z])

        R = np.eye(4)
        R[:3, 0] = b        # grasp_tcp +X = finger closing = GPD binormal
        R[:3, 1] = x        # grasp_tcp +Y = GPD axis
        R[:3, 2] = a        # grasp_tcp +Z = approach
        q = quaternion_from_matrix(R)

        tcp = p + a * self.tcp_offset      # hand BASE centre -> jaw centre-line

        ps = PoseStamped()
        ps.header.frame_id = self.frame
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in tcp)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = (float(v) for v in q)
        return ps

    # ---------- the seam ----------
    def _cb(self, req):
        res = GetGraspsResponse()
        try:
            rgb, depth, info = self._capture()
        except rospy.ROSException as exc:
            res.success = False
            res.message = 'capture failed: %s' % exc
            rospy.logwarn('[gpd_grasp] %s', res.message)
            return res

        bgr = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        dep = self.bridge.imgmsg_to_cv2(depth, '32FC1')
        mine, det = self._masks(rgb, bgr)
        if mine is None or det is None:
            res.success = False
            res.message = 'segmentation produced no target'
            return res

        cloud_pts, view_point = self._unproject(mine, dep, info, rgb.header.stamp,
                                                rgb.header.frame_id)
        sample_pts, _ = self._unproject(det, dep, info, rgb.header.stamp, rgb.header.frame_id)
        if len(cloud_pts) < 50 or len(sample_pts) < 5:
            res.success = False
            res.message = ('too few points (cloud %d, samples %d) -- is the depth image valid?'
                           % (len(cloud_pts), len(sample_pts)))
            rospy.logwarn('[gpd_grasp] %s', res.message)
            return res

        # WHAT GPD ACTUALLY GETS: the DETONATOR cloud, and nothing else.
        #
        # The intent has not changed -- GPD must only ever propose grasps on the detonator, never
        # across the 136 mm disc, which is wider than the 2F-140's usable aperture and is not the
        # part we were asked to pick. What changed is HOW we say it. CloudSamples exists to say
        # exactly this ("the points at which the detector should search") and gpd_ros's handling
        # of it segfaults. So instead of telling GPD where to look inside a bigger cloud, we hand
        # it a cloud that IS the place to look. Same guarantee, via a code path that works.
        #
        # What we give up: GPD can no longer see the disc, so it cannot reason about the fingers
        # colliding with it. That check does not disappear, it MOVES -- the planning scene carries
        # the mine as disc + detonator, so ComputeIK rejects any GPD grasp whose gripper would hit
        # the disc. GPD proposes; MoveIt disposes.
        cloud_msg = self._cloud_msg(sample_pts, self.frame, rgb.header.stamp)
        self.cloud_to_gpd.publish(cloud_msg)                                   # what GPD gets
        self.cloud_mine.publish(self._cloud_msg(cloud_pts, self.frame,         # context only
                                                rgb.header.stamp))
        self._grasps = None
        self.cloud_pub.publish(cloud_msg)
        rospy.loginfo('[gpd_grasp] sent GPD %d DETONATOR points on ~cloud_to_gpd (the %d-point '
                      'whole-mine cloud is on ~cloud_mine, for eyeballing only); waiting...',
                      len(sample_pts), len(cloud_pts))

        deadline = rospy.Time.now() + rospy.Duration(self.timeout)
        r = rospy.Rate(20)
        while self._grasps is None and rospy.Time.now() < deadline and not rospy.is_shutdown():
            r.sleep()
        if self._grasps is None:
            res.success = False
            res.message = ('GPD published nothing on %s in %.0f s. Check that detect_grasps is '
                           'alive (it segfaults on the CloudSamples path) and that this is the '
                           'topic it actually advertises.' % (self.grasp_topic, self.timeout))
            rospy.logerr('[gpd_grasp] %s', res.message)
            return res

        grasps = sorted(self._grasps.grasps, key=lambda g: -g.score.data)[:self.max_grasps]
        if not grasps:
            res.success = False
            res.message = 'GPD found 0 grasps on the detonator'
            return res

        markers = MarkerArray()
        for i, g in enumerate(grasps):
            ps = self._to_tcp(g)
            res.grasps.append(ps)
            res.width.append(float(g.width.data))
            res.score.append(float(g.score.data))

            m = Marker()
            m.header = ps.header
            m.ns, m.id = 'gpd', i
            m.type, m.action = Marker.ARROW, Marker.ADD
            m.pose = ps.pose
            m.scale.x, m.scale.y, m.scale.z = 0.06, 0.008, 0.008
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 0.9
            markers.markers.append(m)
        self.viz.publish(markers)

        res.success = True
        res.message = ('%d GPD grasps on the detonator (best score %.3f); TCP = hand base + '
                       'approach * %.3f m' % (len(grasps), grasps[0].score.data,
                                              self.tcp_offset))
        rospy.loginfo('[gpd_grasp] %s', res.message)
        return res


if __name__ == '__main__':
    rospy.init_node('gpd_grasp_server')
    GpdGraspServer()
    rospy.spin()
