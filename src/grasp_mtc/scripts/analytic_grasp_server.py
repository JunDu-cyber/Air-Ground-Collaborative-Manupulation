#!/usr/bin/env python3
"""Analytic top-down grasp server — grasp_source:=analytic.

The FIRST detector behind the GetGrasps seam, and deliberately so. The landmine's own SDF
prescribes the grasp:

    "Grasp the 40mm detonator block top-down: closes to 40mm, 100mm of 2F-140 stroke margin."

So the correct grasp is *known*. Proving the whole mission with it — unstow, perceive,
plan, approach, close, attach, lift, restow — separates "does the pipeline work" from
"does the detector work". GPD and GraspGen then drop in behind this same service with the
MTC task unchanged, and this stands as the CONTROL ARM of the A/B: a strong ground truth
the old pipeline never had.

GEOMETRY (all measured by FK from the URDF, not guessed — see grasp_tcp in
husky_ur5.urdf.xacro):

  grasp_tcp is the jaw center-line: +Z approach, +X finger closing.

  The pads ADVANCE ~22.3 mm along +Z as they close on a 40 mm object. That is the 2F-140's
  four-bar linkage arc, not an error:
        q=0.000 -> gap 136.0 mm, jaw z 0.1770
        q=0.536 -> gap  40.0 mm, jaw z 0.1993   (+22.3 mm)
        q=0.695 -> gap   8.3 mm, jaw z 0.2007   (+23.7 mm)
  So a grasp aimed exactly at an object's centre will have the pads settle 22 mm PAST it.
  We compensate: aim high by that advance.

  The detector reports the detonator's TOP FACE. The block is 60 mm tall, so:
        pads should end up at   top - 30 mm     (mid-block)
        => TCP must be aimed at top - 30 + 22.3 = top - 7.7 mm
  i.e. the TCP sits just below the top face, and the pads slide down to the middle as they
  close. Aiming at the block's centre instead would drive the pads to within 8 mm of the
  red disc and risk fouling on it.
"""
import math

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray

from grasp_mtc.srv import GetGrasps, GetGraspsResponse

# --- 2F-140 four-bar geometry ---
# The pads travel DOWN the approach axis as they close (the four-bar swings them forward as
# well as inward), so the TCP must be aimed HIGH by exactly that much for the pads to end up
# where we want them. How much depends on where the close starts and where it ends -- which
# means this number is a property of THE CLOSE, and must be re-derived whenever the close
# changes.
#
# By FK from the URDF (distance from robotiq_arg2f_base_link along the approach axis):
#     grasp_tcp        0.1770        <- the TCP frame
#     pad @ q=0.000    0.1770        <- fully open: the pads sit exactly IN the TCP plane
#     pad @ q=0.495    0.1985        <- first contact on a 40 mm block
# so closing from open to contact advances the pads 0.1985 - 0.1770 = 21.5 mm.
#
# The close now runs from FULLY OPEN to grasp_task.CLOSE_TARGET = 0.400, where FK puts the pads
# at 0.1961, so the advance is 0.1961 - 0.1770 = 19.1 mm.
#
# KEEP THIS IN SYNC WITH grasp_task.CLOSE_TARGET: they are two halves of one number. Get it wrong
# and the pads come to rest at the wrong height on the block -- too low and their lower edge fouls
# the 136 mm disc, which levers the whole mine instead of cradling it.
PAD_ADVANCE_M = 0.0191
DETONATOR_H = 0.060       # block height
DETONATOR_W = 0.040       # block cross-section -> required gripper opening

# How far BELOW the detonator's top face the pads should finally come to rest.
#
# NOT mid-block. The mine is one rigid body: a 60 mm detonator standing on a 136 mm DISC whose
# top face is only 25 mm off the ground. Under-compensating the pad advance (the old 22.3 mm
# against a real 30 mm) left the pads finishing in the block's lowest third, where the pad's
# lower edge fouls the disc -- so instead of pinching the block the jaws lever the whole mine,
# which is exactly the "pads bat the mine away" behaviour we measured.
#
# 20 mm below the top face puts the pads in the block's UPPER third, ~40 mm clear of the disc,
# with the full pad face still on the 60 mm block.
PAD_DEPTH_M = 0.020


class AnalyticGraspServer(object):
    def __init__(self):
        self.frame = rospy.get_param('~planning_frame', 'base_link')
        # Candidate gripper rolls about the vertical.
        #
        # THE OLD VERSION SPRAYED 8 YAWS EVERY 22.5 deg, on the reasoning that "the block is
        # square, so any yaw grips it". That is exactly backwards. A parallel-jaw gripper can
        # only grip a square across a pair of opposite FACES. At 45 deg the pads meet two
        # opposite CORNERS, and squeezing a square by its corners CAMS IT OUT from between the
        # jaws: measured, the mine slid diagonally out while finger_joint closed straight
        # through to its commanded angle, as if nothing were there. Most candidates were
        # corner grasps, and IK happily chose one.
        #
        # So the yaws must be locked to the block's own faces: theta + k*90 deg, where theta
        # is the block's yaw from the detector. All four are face-parallel; offering all four
        # still lets ComputeIK pick a comfortable arm configuration.
        self.block_yaw = None
        rospy.Subscriber('/detected_target_yaw', Float32,
                         lambda m: setattr(self, 'block_yaw', float(m.data)))
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        self.viz = rospy.Publisher('~candidates', MarkerArray, queue_size=1, latch=True)
        rospy.Service('/get_grasps', GetGrasps, self._cb)
        rospy.loginfo('[analytic_grasp] serving /get_grasps in %s '
                      '(face-aligned yaws from /detected_target_yaw)', self.frame)

    def _cb(self, req):
        res = GetGraspsResponse()
        try:
            pt = self.tf_buffer.transform(req.target, self.frame, rospy.Duration(2.0))
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = 'TF %s -> %s failed: %s' % (
                req.target.header.frame_id, self.frame, exc)
            rospy.logwarn('[analytic_grasp] %s', res.message)
            return res

        top = np.array([pt.point.x, pt.point.y, pt.point.z])   # detonator TOP face centre
        # Aim so that AFTER the pads' 30 mm closing advance they come to rest PAD_DEPTH_M below
        # the top face. The TCP therefore starts slightly ABOVE the block and the jaws descend
        # onto it as they close -- which is what the four-bar does anyway.
        aim_z = top[2] - PAD_DEPTH_M + PAD_ADVANCE_M

        # Face-aligned yaws only. If the detector could not recover the block's orientation
        # we fall back to 0 deg, which is right whenever the mine is axis-aligned with the
        # base -- but a WARNING, because a wrong yaw is a corner grasp and a corner grasp
        # silently fails to hold.
        theta = self.block_yaw
        if theta is None:
            rospy.logwarn('[analytic_grasp] no block yaw published; assuming 0. A yaw that is '
                          'off by 45 deg is a CORNER grasp and will not hold.')
            theta = 0.0
        yaws = [theta + k * math.pi / 2.0 for k in range(4)]

        markers = MarkerArray()
        for i, yaw in enumerate(yaws):
            R = np.eye(4)
            R[:3, 0] = [math.cos(yaw), math.sin(yaw), 0.0]   # +X = finger closing
            R[:3, 2] = [0.0, 0.0, -1.0]                      # +Z = approach, straight DOWN
            R[:3, 1] = np.cross(R[:3, 2], R[:3, 0])          # +Y = right-handed
            q = quaternion_from_matrix(R)

            g = PoseStamped()
            g.header.frame_id = self.frame
            g.header.stamp = rospy.Time.now()
            g.pose.position.x, g.pose.position.y = float(top[0]), float(top[1])
            g.pose.position.z = float(aim_z)
            (g.pose.orientation.x, g.pose.orientation.y,
             g.pose.orientation.z, g.pose.orientation.w) = (float(v) for v in q)

            res.grasps.append(g)
            res.width.append(DETONATOR_W)
            res.score.append(1.0)

            m = Marker()
            m.header = g.header
            m.ns, m.id = 'grasp', i
            m.type, m.action = Marker.ARROW, Marker.ADD
            m.pose = g.pose
            m.scale.x, m.scale.y, m.scale.z = 0.06, 0.008, 0.008
            m.color.g, m.color.a = 1.0, 0.8
            markers.markers.append(m)

        self.viz.publish(markers)
        res.success = True
        res.message = ('%d top-down candidates; TCP aimed %.1f mm below the top face '
                       '(pads advance %.1f mm closing to %.0f mm)'
                       % (len(res.grasps), (top[2] - aim_z) * 1000.0,
                          PAD_ADVANCE_M * 1000.0, DETONATOR_W * 1000.0))
        rospy.loginfo('[analytic_grasp] %s', res.message)
        return res


if __name__ == '__main__':
    rospy.init_node('analytic_grasp_server')
    AnalyticGraspServer()
    rospy.spin()
