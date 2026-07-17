#!/usr/bin/env python3
"""Bridge the UAV detector's confirmed-mine map onto OUR tour's target seam.

THE ONE PIECE OF GLUE between the grafted airborne detector and this framework.

The UAV detection subsystem (ported verbatim from the uav-mine-to-ugv branch:
mine_seg_localizer -> mine_map_fusion) publishes a persistent, multi-frame-CONFIRMED
map of landmines as `uav_truth_tracker/MineMap` on /mine_detection/map, in the UAV's
latched world frame (uav0/map_local here; the branch called it `map`).

Our UGV tour (ugv_target_tour.py) does not know that message. It collects
`mobile_manipulator/WorldTarget` on /detected_targets, transforms each point into `odom`
through the live TF tree (its _to_odom()), and dedups. So this node simply re-expresses
every CONFIRMED map entry as one WorldTarget and lets the tour do the rest.

Why only `confirmed` entries: the fusion node holds unconfirmed candidates until they are
seen enough times with a tight enough spread (and pass its local-ground gate). Forwarding
those would put single-frame false positives on the tour. The whole point of keeping the
fusion node was to gate on `confirmed`.

Why re-publish on every map update rather than exactly once: /detected_targets is a plain
topic with no latching, and the tour may subscribe after the first confirmation. Re-sending
the confirmed set on each MineMap revision is cheap and idempotent — the tour dedups at its
own radius (0.5 m), so a mine already collected is merged, not duplicated. We still log each
id only the first time, so the console shows real discoveries, not the heartbeat.
"""
import rospy
from geometry_msgs.msg import PointStamped

from uav_truth_tracker.msg import MineMap
from mobile_manipulator.msg import WorldTarget


class MineMapToWorldTarget:
    def __init__(self):
        self.out_topic = rospy.get_param('~output_topic', '/detected_targets')
        self.in_topic = rospy.get_param('~map_topic', '/mine_detection/map')
        # Frame the confirmed positions live in. Prefer the MineMap header (the fusion
        # node stamps it), fall back to this param if the header is empty.
        self.default_frame = rospy.get_param('~map_frame', 'uav0/map_local')
        self.class_name = rospy.get_param('~class_name', 'landmine')
        self.class_id = int(rospy.get_param('~class_id', 0))

        self.seen_ids = set()
        self.pub = rospy.Publisher(self.out_topic, WorldTarget, queue_size=20)
        rospy.Subscriber(self.in_topic, MineMap, self._map_cb, queue_size=5)
        rospy.loginfo('[mine_bridge] %s (confirmed) -> %s  frame=%s',
                      self.in_topic, self.out_topic, self.default_frame)

    def _map_cb(self, msg):
        frame = msg.header.frame_id or self.default_frame
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        for mine in msg.mines:
            if not mine.confirmed:
                continue
            wt = WorldTarget()
            wt.class_name = self.class_name
            wt.class_id = self.class_id
            wt.confidence = float(mine.confidence)
            ps = PointStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = frame
            ps.point = mine.position
            wt.point = ps
            self.pub.publish(wt)
            if mine.id not in self.seen_ids:
                self.seen_ids.add(mine.id)
                rospy.loginfo('[mine_bridge] NEW confirmed mine M%03d at %s=(%.2f, %.2f, %.2f)',
                              mine.id, frame, mine.position.x, mine.position.y, mine.position.z)


if __name__ == '__main__':
    rospy.init_node('mine_map_to_worldtarget')
    MineMapToWorldTarget()
    rospy.spin()
