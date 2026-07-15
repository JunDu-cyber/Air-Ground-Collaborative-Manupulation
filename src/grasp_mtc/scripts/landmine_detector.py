#!/usr/bin/env python3
"""The UGV's OWN landmine detector — the wrist camera, close up, accurate enough to grasp from.

THIS IS NOT THE UAV'S DETECTOR, and the distinction is the whole reason this node exists
separately. Two detections, two topics, two jobs:

    /detected_targets        THE UAV. Coarse: found from ~12 m while scanning, in a world-ish
                             frame. ugv_target_tour COLLECTS these -- they are the list of mines
                             to go and visit. Its producer is not our work; the seam is
                             WorldTarget on that topic and it is already specified.

    /ugv/landmine_detection  THIS NODE. The wrist RealSense at ~0.5 m, in base_link, measured to
                             1.6 mm against Gazebo truth. It is what the final approach aligns on
                             and what the grasp is planned from.

Publishing this on /detected_targets (which it used to do) makes the tour collect the UGV's own
close-range detections as BRAND NEW TARGETS, so it grows phantom mines at its own feet forever.
Harmless on the bench, where nothing else publishes there. Fatal in the mission.

It is a mock, not a toy: the landmine prop is deliberately colour-coded, so an HSV
threshold is an *unambiguous* segmentation of the exact part we must grasp.

    disc body  : red    cylinder, d=136 mm     -> context / disambiguation
    detonator  : yellow box, 40x40x60 mm       -> THE GRASP TARGET

That distinction matters: this is a PART-level grasp. We do not grasp "the landmine", we
grasp its detonator, and the colour tells us which is which for free.

Measured accuracy against Gazebo ground truth, wrist camera at 0.55 m looking straight
down: |error| = 1.7 mm. Good enough to grasp a 40 mm block by a wide margin.

Two gotchas this node handles, both of which cost real debugging time:

  * The naive global centroid of the yellow mask is WRONG. The scene contains ~30 small
    yellow specks; they drag the centroid tens of pixels off. We take the largest
    CONNECTED COMPONENT instead, and its area (~2300 px at 0.47 m) matches the 40 mm face
    to within a few percent.
  * rospy.wait_for_message can hand back a STALE frame captured before the arm moved,
    which silently reports the previous viewpoint's detection. Every frame here is
    stamp-checked against a capture-time barrier.

Publishes:
    /ugv/landmine_detection  mobile_manipulator/WorldTarget  (base_link; the GRASP's input)
    /detected_target_yaw     std_msgs/Float32                (the block's yaw; a 45 deg error
                                                              is a corner grasp)
    /landmine_marker         visualization_msgs/Marker       (RViz)
Service:
    ~detect_once        std_srvs/Trigger                 (detect on demand, for the grasp loop)
"""
import math
import threading

import cv2
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PointStamped with tf2_ros.Buffer.transform
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker

from mobile_manipulator.msg import WorldTarget
from std_msgs.msg import Float32

# The detonator's own SDF colour is diffuse (0.95, 0.85, 0.0). These HSV bounds are
# deliberately loose on hue and tight on saturation/value: Gazebo's lighting shifts the
# hue a little across the block's faces, but nothing else in outdoor_city is a saturated
# yellow, so precision comes from S/V, not H.
YELLOW_LO = (20, 120, 120)
YELLOW_HI = (35, 255, 255)
# Disc is diffuse (0.85, 0.1, 0.1). Red straddles the hue wrap, hence two bands.
RED_LO_A, RED_HI_A = (0, 120, 80), (10, 255, 255)
RED_LO_B, RED_HI_B = (170, 120, 80), (180, 255, 255)

DETONATOR_MM = 40.0   # the block is 40x40 mm in cross-section


class LandmineDetector(object):
    def __init__(self):
        self.frame = rospy.get_param('~target_frame', 'base_link')
        self.rgb_topic = rospy.get_param('~rgb_topic', '/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.info_topic = rospy.get_param('~info_topic', '/camera/color/camera_info')
        # Reject blobs too small to be a real detonator (noise) or implausibly large.
        self.min_area = int(rospy.get_param('~min_blob_area', 200))
        self.continuous = bool(rospy.get_param('~continuous', False))
        self.rate_hz = float(rospy.get_param('~rate', 2.0))

        # BACKEND: how we find the mine. This does NOT change how we find the DETONATOR.
        #
        #   color : HSV threshold on the yellow block, over the whole image. No learning.
        #           Fine in the sim, but it will happily lock onto any yellow thing in frame.
        #   onnx  : models/landmine.onnx (YOLO11s-seg, one class "landmine") first says WHERE
        #           THE MINE IS, as a pixel mask; the yellow search then runs ONLY inside that
        #           mask. The network is robust about "is this a mine"; the colour+geometry is
        #           precise about "where exactly the jaws go". Neither can do the other's job:
        #           the model's single class is the WHOLE mine (disc included), and the grasp
        #           needs the detonator to a couple of millimetres, with its yaw.
        self.backend = str(rospy.get_param('~backend', 'color')).lower()
        self.onnx_srv = str(rospy.get_param('~onnx_service', '/landmine/detect'))
        self.min_conf = float(rospy.get_param('~min_confidence', 0.35))
        self._onnx = None
        if self.backend == 'onnx':
            from mobile_manipulator.srv import DetectLandmines
            rospy.loginfo('[landmine_detector] backend=onnx; waiting for %s '
                          '(the FIRST call builds the TensorRT engine and takes minutes)',
                          self.onnx_srv)
            rospy.wait_for_service(self.onnx_srv)
            self._onnx = rospy.ServiceProxy(self.onnx_srv, DetectLandmines)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        self.lock = threading.Lock()

        self.yaw_pub = rospy.Publisher('/detected_target_yaw', Float32,
                                       queue_size=1, latch=True)
        # /ugv/landmine_detection, NOT /detected_targets, AND THE DIFFERENCE IS LOAD-BEARING.
        #
        # Those two topics carry the same message type and mean completely different things:
        #
        #   /detected_targets       the UAV's seam. Coarse (detected from ~12 m), in a world-ish
        #                           frame. ugv_target_tour COLLECTS these: they are the mines to
        #                           go and visit.
        #   /ugv/landmine_detection THIS. The wrist camera, at ~0.5 m, in base_link. Precise
        #                           enough to grasp from (measured 1.6 mm). It is the input to
        #                           the final approach and the grasp, and to NOTHING ELSE.
        #
        # Publishing this on /detected_targets -- which is what it used to do -- means the tour
        # collects the UGV's own close-range detections as BRAND NEW TARGETS, and grows phantom
        # mines at its own feet for as long as it is running. Harmless on the bench, where
        # nothing else publishes there. Fatal in the mission.
        self.target_pub = rospy.Publisher(
            rospy.get_param('~output_topic', '/ugv/landmine_detection'),
            WorldTarget, queue_size=10)
        self.marker_pub = rospy.Publisher('/landmine_marker', Marker,
                                          queue_size=1, latch=True)
        rospy.Service('~detect_once', Trigger, self._detect_once_cb)

        # STREAMING, switchable at runtime.
        #
        # On-demand detection is right for the GRASP: it wants one detection at the instant the
        # arm reaches the look pose, not a feed of stale viewpoints. But it is WRONG for the
        # final approach, which is a closed loop on a MOVING robot: between two detect_once calls
        # the aligner has nothing but a stale reading, and driving on one is driving blind.
        # Measured: at 0.12 m/s a 1.5 s-old detection is 18 cm out of date, and the UGV drove
        # straight over the mine and BULLDOZED IT 5.7 m across the map, still reporting it dead
        # ahead the whole way -- because it was, being pushed along by the bumper.
        #
        # So the tracker can be turned on for as long as the loop is closed, and off again after.
        self.stream_hz = float(rospy.get_param('~stream_rate', 8.0))
        self._timer = None
        rospy.Service('~stream', SetBool, self._stream_cb)
        if self.continuous:
            self._start_stream(self.rate_hz)

        rospy.loginfo('[landmine_detector] up; frame=%s  continuous=%s  (~stream turns '
                      'closed-loop tracking on at %.0f Hz)',
                      self.frame, self.continuous, self.stream_hz)

    def _start_stream(self, hz):
        if self._timer is not None:
            self._timer.shutdown()
        self._timer = rospy.Timer(rospy.Duration(1.0 / max(hz, 0.1)), self._tick)

    def _stream_cb(self, req):
        if req.data:
            self._start_stream(self.stream_hz)
            return SetBoolResponse(success=True,
                                   message='tracking at %.0f Hz' % self.stream_hz)
        if self._timer is not None:
            self._timer.shutdown()
            self._timer = None
        return SetBoolResponse(success=True, message='tracking off')

    # ---------- capture ----------
    def _fresh(self, topic, typ, timeout=5.0):
        """A frame captured AFTER this call. wait_for_message alone can return a stale
        image from before the arm moved — which reports the previous viewpoint and looks
        exactly like a detection bug."""
        barrier = rospy.Time.now()
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            msg = rospy.wait_for_message(topic, typ, timeout=timeout)
            if msg.header.stamp > barrier:
                return msg
        raise rospy.ROSException('no fresh frame on %s within %.1fs' % (topic, timeout))

    def _mine_mask(self, rgb_msg):
        """Ask models/landmine.onnx where the mine is. Returns a mono8 mask, or None.

        Takes the HIGHEST-CONFIDENCE detection: the grasp loop looks at one mine at a time
        (the tour drives to them one by one), so the best instance is the right one. Masks come
        back already scaled to the request image, so no resampling is needed here.
        """
        try:
            res = self._onnx(image=rgb_msg)
        except rospy.ServiceException as exc:
            rospy.logwarn('[landmine_detector] %s failed: %s', self.onnx_srv, exc)
            return None
        if not res.detections:
            return None

        best = max(range(len(res.detections)), key=lambda i: res.detections[i].confidence)
        det = res.detections[best]
        if det.confidence < self.min_conf:
            rospy.logwarn_throttle(5.0, '[landmine_detector] best mine only %.2f confident '
                                        '(< %.2f); ignoring', det.confidence, self.min_conf)
            return None
        rospy.loginfo('[landmine_detector] onnx: landmine %.0f%% in %.0f ms',
                      det.confidence * 100.0, res.inference_time_ms)
        return self.bridge.imgmsg_to_cv2(res.masks[best], 'mono8')

    # ---------- detection ----------
    def detect(self):
        """Returns (PointStamped in self.frame, blob_area_px, gap_estimate_m) or None.

        The point is the centre of the detonator's visible TOP FACE."""
        try:
            rgb = self._fresh(self.rgb_topic, Image)
            depth = self._fresh(self.depth_topic, Image)
            info = rospy.wait_for_message(self.info_topic, CameraInfo, timeout=3.0)
        except rospy.ROSException as exc:
            rospy.logwarn_throttle(5.0, '[landmine_detector] capture failed: %s', exc)
            return None

        bgr = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
        dep = self.bridge.imgmsg_to_cv2(depth, '32FC1')
        fx, cx, fy, cy = info.K[0], info.K[2], info.K[4], info.K[5]

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
        # Opening kills the single-pixel specks before they can become "blobs".
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        if self._onnx is not None:
            mine = self._mine_mask(rgb)
            if mine is None:
                rospy.logwarn_throttle(
                    5.0, '[landmine_detector] the model saw no landmine; refusing to grasp a '
                         'yellow blob on faith')
                return None
            # THE GATE. Keep only yellow that lies INSIDE a mine the network actually found.
            # Everything downstream (largest blob, centroid, minAreaRect yaw, depth) is
            # unchanged -- the model narrows WHERE to look, it does not do the looking.
            mask = cv2.bitwise_and(mask, mine)
            if cv2.countNonZero(mask) == 0:
                rospy.logwarn_throttle(
                    5.0, '[landmine_detector] mine detected but no yellow detonator inside its '
                         'mask -- occluded, or looking at the disc edge-on')
                return None

        # THE RED DISC IS THE GATE, and without it this detector is not safe to steer a robot
        # with. The world is full of yellow: outdoor_city's road markings are yellow, and when
        # the mine drifts out of the camera's view the largest yellow blob in frame becomes a
        # ROAD LINE. The final approach then turns toward it and drives away chasing paint --
        # measured, the UGV spun 103 degrees off the mine doing exactly this.
        #
        # The prop was built with the answer: the detonator is a yellow block sitting ON a red
        # disc. Road paint is yellow but it is not on a red disc. So keep only yellow that has
        # red around it -- dilate each candidate and require it to touch the red mask. This is
        # the "context / disambiguation" the disc was always there to provide.
        red = cv2.bitwise_or(cv2.inRange(hsv, RED_LO_A, RED_HI_A),
                             cv2.inRange(hsv, RED_LO_B, RED_HI_B))
        red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        # The block stands 60 mm proud of the disc, so at a glancing view its yellow may not
        # literally abut red; grow the neighbourhood enough to bridge that, and no further.
        near_red = cv2.dilate(red, np.ones((25, 25), np.uint8))
        gated = cv2.bitwise_and(mask, near_red)
        if cv2.countNonZero(gated) == 0:
            rospy.logwarn_throttle(
                5.0, '[landmine_detector] yellow in frame, but none of it is on a red disc. '
                     'That is road paint, not a landmine -- refusing it.')
            return None
        mask = gated

        n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        if n <= 1:
            return None
        # Largest component, NOT the global centroid: the scene carries ~30 small yellow
        # specks that would drag a global centroid tens of pixels off the block.
        best = max(range(1, n), key=lambda i: stats[i, cv2.CC_STAT_AREA])
        area = int(stats[best, cv2.CC_STAT_AREA])
        if area < self.min_area:
            return None
        u, v = centroids[best]

        # Depth from the INTERIOR of the block only. This is the reading the whole final
        # approach steers on, so a wrong one is not cosmetic: because x, y and z ALL scale
        # with this depth (x = (u-cx)*z/fx), a depth that is too SMALL makes the mine read
        # closer than the standoff, and the aligner then reverses AWAY from it. Measured: the
        # UGV drove backward 0.44 m off a single under-read depth and lost the track.
        #
        # The rim pixels are where it goes wrong: the block edge, and any few-pixel mismatch
        # between the colour mask and the depth frame while the robot is moving, let the depth
        # "see past" the 40 mm block to the disc 25 mm lower or the ground 85 mm lower. So
        # sample only pixels well INSIDE the blob -- erode it first.
        blob = (labels == best).astype(np.uint8)
        interior = cv2.erode(blob, np.ones((5, 5), np.uint8))
        if cv2.countNonZero(interior) < 10:
            interior = blob                       # tiny blob: keep what we have
        ys, xs = np.nonzero(interior)
        d = dep[ys, xs]
        d = d[np.isfinite(d) & (d > 0.05) & (d < 5.0)]
        if d.size < 10:
            return None
        z = float(np.median(d))

        ps = PointStamped()
        ps.header.frame_id = rgb.header.frame_id  # the camera OPTICAL frame
        ps.header.stamp = rospy.Time(0)           # latest available TF, not the capture stamp
        ps.point.x = (u - cx) * z / fx
        ps.point.y = (v - cy) * z / fy
        ps.point.z = z
        try:
            out = self.tf_buffer.transform(ps, self.frame, rospy.Duration(2.0))
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn('[landmine_detector] TF %s->%s failed: %s',
                          ps.header.frame_id, self.frame, exc)
            return None

        # ---- the block's YAW, and why a centroid alone is not enough ----------------
        # The detonator is a SQUARE 40 mm box, and a parallel-jaw gripper can only grip a
        # square across a pair of opposite FACES. Close on it at 45 deg and the pads land on
        # two opposite CORNERS, which cams the block straight out from between them: measured,
        # the mine squirts diagonally out of the jaws while finger_joint sails on to its
        # commanded angle as if nothing were there. So the grasp needs the block's
        # orientation, not just its centre.
        #
        # minAreaRect, not PCA: PCA is degenerate on a square (both eigenvalues equal, so the
        # axis it returns is noise). The min-area rectangle recovers the actual edges.
        # The edge direction is measured in 3-D, in the TARGET frame -- deproject two points
        # along one rect edge onto the top-face plane and difference them -- so it needs no
        # assumption that the camera is level or looking straight down.
        cnt, _ = cv2.findContours((labels == best).astype(np.uint8),
                                  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yaw = None
        if cnt:
            box = cv2.boxPoints(cv2.minAreaRect(cnt[0]))
            p0, p1 = box[0], box[1]                    # one edge of the top face
            ends = []
            for (uu, vv) in (p0, p1):
                q = PointStamped()
                q.header.frame_id = rgb.header.frame_id
                q.header.stamp = rospy.Time(0)
                q.point.x = (uu - cx) * z / fx
                q.point.y = (vv - cy) * z / fy
                q.point.z = z
                try:
                    ends.append(self.tf_buffer.transform(q, self.frame, rospy.Duration(1.0)))
                except Exception:  # noqa: BLE001
                    ends = []
                    break
            if len(ends) == 2:
                dx = ends[1].point.x - ends[0].point.x
                dy = ends[1].point.y - ends[0].point.y
                if math.hypot(dx, dy) > 1e-4:
                    # Fold into [0, 90): the square's 4-fold symmetry means an edge at t is
                    # the same grip as t+90, and the server offers both anyway.
                    yaw = math.atan2(dy, dx) % (math.pi / 2.0)

        # Apparent width of the block, from its pixel extent — and it is a DEPTH VALIDATOR, not
        # just a hint. apparent = w_px * z / fx, so if the depth z is wrong the apparent width
        # is wrong by the same factor -- and the detonator's true 40 mm is a fact we can check
        # against. A face-on block is 40 mm; a 45 deg-yawed one projects 40*sqrt(2) = 56.6 mm.
        # Anything outside [28, 64] mm means the depth is not measuring the top face, so the 3-D
        # point is garbage. REJECT IT: a dropped detection just makes the aligner wait for the
        # next frame, whereas a wrong one drives the robot the wrong way.
        w_px = float(stats[best, cv2.CC_STAT_WIDTH])
        gap = w_px * z / fx
        if not (0.028 <= gap <= 0.064):
            rospy.logwarn_throttle(
                2.0, '[landmine_detector] apparent width %.0f mm is not a 40 mm block: the '
                     'depth (%.3f m) is wrong, dropping this detection', gap * 1000.0, z)
            return None
        return out, area, gap, yaw

    # ---------- outputs ----------
    def _publish(self, pt, area, gap, yaw=None):
        # The block's yaw goes out on its own LATCHED topic rather than inside WorldTarget,
        # because WorldTarget carries a bare PointStamped and it is the tour's existing,
        # shared seam -- widening it would ripple into the nav side for the sake of one
        # float that only the grasp server consumes. The grasp source subscribes to this.
        if yaw is not None:
            self.yaw_pub.publish(Float32(data=float(yaw)))
        msg = WorldTarget()
        msg.class_name = 'landmine'
        msg.class_id = 0
        # Confidence from how close the apparent size is to the true 40 mm block.
        rel = abs(gap * 1000.0 - DETONATOR_MM) / DETONATOR_MM
        msg.confidence = float(max(0.0, min(1.0, 1.0 - rel)))
        msg.point = pt
        self.target_pub.publish(msg)

        m = Marker()
        m.header.frame_id = pt.header.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns, m.id = 'landmine', 0
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose.position = pt.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.04
        m.scale.z = 0.06
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.9, 0.0, 0.8
        self.marker_pub.publish(m)

        rospy.loginfo('[landmine_detector] detonator @ [%.3f %.3f %.3f] %s '
                      '(blob %d px, apparent %.0f mm, conf %.2f, yaw %s)',
                      pt.point.x, pt.point.y, pt.point.z, pt.header.frame_id,
                      area, gap * 1000.0, msg.confidence,
                      '%.1f deg' % math.degrees(yaw) if yaw is not None else 'unknown')

    def _tick(self, _evt):
        with self.lock:
            r = self.detect()
        if r:
            self._publish(*r)

    def _detect_once_cb(self, _req):
        with self.lock:
            r = self.detect()
        if not r:
            return TriggerResponse(success=False, message='no detonator visible')
        self._publish(*r)
        pt = r[0].point
        return TriggerResponse(
            success=True,
            message='%.4f %.4f %.4f %s' % (pt.x, pt.y, pt.z, r[0].header.frame_id))


if __name__ == '__main__':
    rospy.init_node('landmine_detector')
    LandmineDetector()
    rospy.spin()
