#!/usr/bin/env python3
"""Draw the ACTUAL 2F-140 JAWS at every grasp candidate.

WHY AN ARROW IS NOT ENOUGH
--------------------------
Both grasp servers already publish an arrow per candidate. An arrow tells you where the TCP is
and which way it points, and it CANNOT tell you the one thing that matters:

    do the pads actually straddle the detonator, or is the TCP sitting somewhere plausible-
    looking while the jaws close on thin air (or on the disc)?

That is exactly the failure the legacy pipeline died of, and it is invisible in an arrow. The
old code applied GPD's basis straight to ur5_tool0 (which is bolted on rpy="0 -1.57 1.57", so
its axes are NOT the gripper's) and then slid it back by a guessed 0.12 m. Every grasp came out
wrong, and every arrow looked perfectly reasonable.

So this node renders the gripper: two pads at the gap they will REALLY close to, at the height
the four-bar will REALLY carry them to, in the TCP's own frame. If the pads do not bracket the
yellow block on screen, the grasp is wrong -- before anything moves.

It is also the plan's own acceptance check for the GPD adapter ("overlay adapter TCP markers on
GPD's plot_grasps; they must coincide"), which you can now do by eye: GPD's own markers are on
/detect_grasps/plot_grasps, ours are here, in the same frame.

EVERY NUMBER BELOW IS FK FROM THE URDF, not a guess -- see grasp_mtc/scripts/grasp_task.py.
"""
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray

# --- 2F-140 geometry, all FK-derived (see the gap table in grasp_task.py) -------------------
# grasp_tcp: +Z = approach, +X = finger closing, +Y = the remaining axis.
#
#   finger_joint = 0.400  ->  pad gap 58.9 mm, pads 19.1 mm along +Z from the TCP plane
#
# 0.400 is what the pick actually commands (grasp_task.CLOSE_TARGET): the jaws CRADLE the 40 mm
# block with ~8.6 mm of air per side and never touch it, because the fingers are kinematic and
# cannot grip -- the mine is carried by a weld. Drawing the jaws anywhere else would be drawing
# a grasp that does not happen.
PAD_GAP = 0.0589          # face-to-face, at finger_joint = 0.400
PAD_ADVANCE = 0.0191      # how far the pads sit along +Z from the TCP plane once closed
PAD_X, PAD_Y, PAD_Z = 0.0075, 0.025, 0.060      # one pad's collision box (thickness, w, h)
PALM_BACK = 0.06          # palm sits this far back along -Z from the TCP

TOPICS = ['/analytic_grasp_server/candidates', '/gpd_grasp_server/candidates']


def _q(pose):
    o = pose.orientation
    return (o.x, o.y, o.z, o.w)


class GraspViz(object):
    def __init__(self):
        self.max_show = int(rospy.get_param('~max_candidates', 8))
        self.pub = rospy.Publisher('~jaws', MarkerArray, queue_size=1, latch=True)
        for t in rospy.get_param('~sources', TOPICS):
            rospy.Subscriber(t, MarkerArray, self._cb, callback_args=t, queue_size=1)
        rospy.loginfo('[grasp_viz] drawing the real jaws (gap %.1f mm) for candidates on: %s',
                      PAD_GAP * 1000, ', '.join(TOPICS))

    def _cb(self, msg, source):
        # The servers publish one ARROW per candidate, best first. Take their poses; we do not
        # care about the arrows themselves.
        poses = [m.pose for m in msg.markers if m.type == Marker.ARROW]
        if not poses:
            return
        poses = poses[:self.max_show]
        frame = msg.markers[0].header.frame_id
        ns = 'gpd' if 'gpd' in source else 'analytic'

        out = MarkerArray()
        mid = 0
        for rank, pose in enumerate(poses):
            best = (rank == 0)
            # The best candidate is the one MTC will most likely execute (they are cost-ordered),
            # so make it unmistakable and fade the rest -- a screen of eight identical grippers
            # tells you nothing about which one is about to happen.
            if best:
                rgba = (0.1, 1.0, 0.2, 0.95)
            else:
                rgba = (0.4, 0.6, 1.0, 0.25)

            for sx in (-1.0, 1.0):        # the two pads, either side of the closing axis (+X)
                m = self._box(frame, ns, mid, pose, rgba,
                              offset=(sx * (PAD_GAP + PAD_X) / 2.0, 0.0, PAD_ADVANCE),
                              scale=(PAD_X, PAD_Y, PAD_Z))
                out.markers.append(m)
                mid += 1

            # Palm, behind the TCP along -Z. Gives the eye something to read the approach from.
            out.markers.append(self._box(frame, ns, mid, pose, rgba,
                                         offset=(0.0, 0.0, -PALM_BACK / 2.0),
                                         scale=(0.09, 0.05, PALM_BACK)))
            mid += 1

            # The approach axis, so a top-down grasp is obviously top-down.
            a = Marker()
            a.header.frame_id = frame
            a.header.stamp = rospy.Time.now()
            a.ns, a.id = ns + '_approach', mid
            a.type, a.action = Marker.ARROW, Marker.ADD
            a.pose = pose
            a.scale.x, a.scale.y, a.scale.z = 0.10, 0.006, 0.006
            a.color.r, a.color.g, a.color.b, a.color.a = rgba
            out.markers.append(a)
            mid += 1

        self.pub.publish(out)
        rospy.loginfo_throttle(2.0, '[grasp_viz] %s: drew %d candidate gripper(s)',
                               ns, len(poses))

    @staticmethod
    def _box(frame, ns, mid, pose, rgba, offset, scale):
        """A box placed in the TCP's OWN frame, then carried into `frame` by the candidate pose.

        Doing the offset in the TCP frame (rather than adding it in base_link) is the whole
        point: it means the pads are drawn where the GRIPPER puts them, whatever orientation the
        detector proposed. Get this wrong and every grasp looks correct.
        """
        import numpy as np
        from tf.transformations import quaternion_matrix

        T = quaternion_matrix(_q(pose))
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        world = p + T[:3, :3].dot(np.array(offset))

        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = rospy.Time.now()
        m.ns, m.id = ns, mid
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose.orientation = pose.orientation          # the box rides with the gripper
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in world)
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m


if __name__ == '__main__':
    rospy.init_node('grasp_viz')
    GraspViz()
    rospy.spin()
