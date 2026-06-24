#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Collect UAV-detected targets and drive the UGV through them in sequence.

Mission flow (air-ground): the UAV scans the terrain and detects target points
with its camera, publishing one mobile_manipulator/WorldTarget per detection on
/detected_targets. This node:

  1. COLLECT (always): TF-transform each target's point from its own
     header.frame_id (a UAV camera frame) into `odom` -- using the live TF tree,
     whose odom->uav0/map_local edge is the latched air-ground anchor -- and dedup
     near-duplicate detections (keep the highest-confidence one).
  2. TRIGGER: the service /ugv/start_tour (std_srvs/Trigger) freezes the collected
     set and starts the tour. (The future "UAV finished scanning" signal calls it;
     call it by hand for now.)
  3. ORDER: greedy nearest-neighbor from the UGV's current /state_estimation pose.
     Targets sitting on untraversable terrain (sampled from /terrain_map cost) are
     deprioritized to the end so a reachable order comes first.
  4. DRIVE: publish the current target on /ugv/goal (PoseStamped, odom) -- which
     goal_to_waypoint feeds to the CMU localPlanner -- and watch /state_estimation;
     when within reach_tolerance (or a per-goal timeout) advance to the next.

Everything reaches the planner through /ugv/goal, so a hand-published /ugv/goal
still works for manual testing. RViz markers on /target_tour_markers show the
targets + the numbered visiting order.
"""
import math
import threading

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform)
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from mobile_manipulator.msg import WorldTarget


class Tgt(object):
    __slots__ = ('x', 'y', 'z', 'cls', 'conf')

    def __init__(self, x, y, z, cls, conf):
        self.x, self.y, self.z, self.cls, self.conf = x, y, z, cls, conf


class TargetTour(object):
    COLLECT, TOUR, DONE = 'COLLECT', 'TOUR', 'DONE'

    def __init__(self):
        self.detected_topic = rospy.get_param('~detected_topic', '/detected_targets')
        self.goal_topic = rospy.get_param('~goal_topic', '/ugv/goal')
        self.odom_topic = rospy.get_param('~odom_topic', '/state_estimation')
        self.terrain_topic = rospy.get_param('~terrain_topic', '/terrain_map')
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.dedup_radius = float(rospy.get_param('~dedup_radius', 0.5))
        self.reach_tolerance = float(rospy.get_param('~reach_tolerance', 0.6))
        self.goal_timeout = float(rospy.get_param('~goal_timeout', 60.0))
        self.obstacle_cost_thre = float(rospy.get_param('~obstacle_cost_thre', 0.3))
        self.terrain_sample_radius = float(rospy.get_param('~terrain_sample_radius', 0.5))
        self.republish_period = float(rospy.get_param('~republish_period', 1.0))

        self.lock = threading.Lock()
        self.targets = []          # collected, in odom
        self.state = self.COLLECT
        self.order = []            # indices into self.targets, visiting order
        self.cur = 0
        self.odom = None           # (x, y)
        self.terrain = None        # latest /terrain_map PointCloud2
        self.goal_start = 0.0
        self.last_pub = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/target_tour_markers', MarkerArray,
                                          queue_size=1, latch=True)
        rospy.Subscriber(self.detected_topic, WorldTarget, self._target_cb, queue_size=20)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(self.terrain_topic, PointCloud2, self._terrain_cb, queue_size=1)
        rospy.Service('/ugv/start_tour', Trigger, self._start_cb)
        rospy.Timer(rospy.Duration(0.2), self._loop)

        rospy.loginfo('[target_tour] collecting on %s -> drive %s ; call /ugv/start_tour to begin',
                      self.detected_topic, self.goal_topic)

    # ---------- inputs ----------
    def _odom_cb(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _terrain_cb(self, msg):
        self.terrain = msg

    def _target_cb(self, msg):
        pt = self._to_odom(msg.point)
        if pt is None:
            return
        with self.lock:
            if self.state != self.COLLECT:
                return  # tour already frozen; ignore late detections
            self._add(pt[0], pt[1], pt[2], msg.class_name, msg.confidence)

    def _to_odom(self, point_stamped):
        """TF a PointStamped into self.target_frame; (x,y,z) or None on failure."""
        frame = point_stamped.header.frame_id
        if not frame or frame == self.target_frame:
            p = point_stamped.point
            return (p.x, p.y, p.z)
        try:
            ps = self.tf_buffer.transform(point_stamped, self.target_frame,
                                          timeout=rospy.Duration(0.5))
            return (ps.point.x, ps.point.y, ps.point.z)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(
                5.0, '[target_tour] TF %s->%s failed (%s); target dropped (retry on next detection)',
                frame, self.target_frame, exc)
            return None

    def _add(self, x, y, z, cls, conf):
        for t in self.targets:
            if math.hypot(t.x - x, t.y - y) < self.dedup_radius:
                if conf > t.conf:                       # keep the more confident detection
                    t.x, t.y, t.z, t.cls, t.conf = x, y, z, cls, conf
                return
        self.targets.append(Tgt(x, y, z, cls, conf))
        rospy.loginfo('[target_tour] collected target #%d %s (%.2f, %.2f) conf=%.2f',
                      len(self.targets), cls, x, y, conf)

    # ---------- trigger + ordering ----------
    def _start_cb(self, _req):
        with self.lock:
            if not self.targets:
                return TriggerResponse(success=False, message='no targets collected')
            if self.state == self.TOUR:
                return TriggerResponse(success=False, message='tour already running')
            self.order = self._compute_order()
            self.state = self.TOUR
            self.cur = 0
            self.goal_start = rospy.Time.now().to_sec()
            self.last_pub = 0.0
            if self.order:
                self._publish_goal(self.order[0])
            msg = 'tour started: %d targets, order=%s' % (len(self.order), self.order)
            rospy.loginfo('[target_tour] %s', msg)
            return TriggerResponse(success=True, message=msg)

    def _terrain_cost(self, x, y):
        """Max /terrain_map cost within terrain_sample_radius of (x,y); None if no coverage."""
        if self.terrain is None:
            return None
        r2 = self.terrain_sample_radius * self.terrain_sample_radius
        best = None
        for px, py, inten in pc2.read_points(self.terrain, field_names=('x', 'y', 'intensity'),
                                             skip_nans=True):
            if (px - x) ** 2 + (py - y) ** 2 <= r2:
                if best is None or inten > best:
                    best = inten
        return best

    def _compute_order(self):
        """Greedy nearest-neighbor from the UGV pose; untraversable targets last."""
        sx, sy = self.odom if self.odom else (0.0, 0.0)
        keep, deferred = [], []
        for i in range(len(self.targets)):
            c = self._terrain_cost(self.targets[i].x, self.targets[i].y)
            if c is not None and c > self.obstacle_cost_thre:
                deferred.append(i)
            else:
                keep.append(i)
        if deferred:
            rospy.logwarn('[target_tour] %d target(s) on untraversable terrain -> deprioritized',
                          len(deferred))

        def nn_chain(pool, cx, cy):
            seq = []
            while pool:
                j = min(pool, key=lambda i: (self.targets[i].x - cx) ** 2 + (self.targets[i].y - cy) ** 2)
                seq.append(j)
                pool.remove(j)
                cx, cy = self.targets[j].x, self.targets[j].y
            return seq, cx, cy

        order, cx, cy = nn_chain(keep, sx, sy)
        tail, _, _ = nn_chain(deferred, cx, cy)
        return order + tail

    # ---------- drive loop ----------
    def _publish_goal(self, idx):
        t = self.targets[idx]
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.target_frame
        ps.pose.position.x = t.x
        ps.pose.position.y = t.y
        ps.pose.position.z = t.z
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)
        self.last_pub = rospy.Time.now().to_sec()

    def _loop(self, _evt):
        with self.lock:
            self._publish_markers()
            if self.state != self.TOUR or self.odom is None:
                return
            if self.cur >= len(self.order):
                self.state = self.DONE
                rospy.loginfo('[target_tour] tour complete (%d targets visited)', len(self.order))
                return
            idx = self.order[self.cur]
            t = self.targets[idx]
            d = math.hypot(t.x - self.odom[0], t.y - self.odom[1])
            now = rospy.Time.now().to_sec()
            if d < self.reach_tolerance:
                rospy.loginfo('[target_tour] reached %d/%d %s (d=%.2f)',
                              self.cur + 1, len(self.order), t.cls, d)
                self._advance(now)
            elif now - self.goal_start > self.goal_timeout:
                rospy.logwarn('[target_tour] %d/%d timeout (d=%.2f) -> skip',
                              self.cur + 1, len(self.order), d)
                self._advance(now)
            elif now - self.last_pub > self.republish_period:
                self._publish_goal(idx)

    def _advance(self, now):
        self.cur += 1
        self.goal_start = now
        if self.cur < len(self.order):
            self._publish_goal(self.order[self.cur])

    # ---------- viz ----------
    def _publish_markers(self):
        ma = MarkerArray()
        visited = set(self.order[:self.cur]) if self.state in (self.TOUR, self.DONE) else set()
        cur_idx = self.order[self.cur] if (self.state == self.TOUR and self.cur < len(self.order)) else -1
        for i, t in enumerate(self.targets):
            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = rospy.Time.now()
            m.ns = 'targets'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = t.x, t.y, t.z + 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            if i == cur_idx:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0   # current = yellow
            elif i in visited:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0   # visited = green
            else:
                m.color.r, m.color.g, m.color.b = 0.1, 0.4, 1.0   # pending = blue
            m.color.a = 0.9
            ma.markers.append(m)

            txt = Marker()
            txt.header.frame_id = self.target_frame
            txt.header.stamp = m.header.stamp
            txt.ns = 'labels'
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = t.x, t.y, t.z + 0.9
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.4
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            rank = self.order.index(i) + 1 if i in self.order else 0
            txt.text = '%d:%s' % (rank, t.cls) if rank else t.cls
            ma.markers.append(txt)

        # order path line strip
        if self.order:
            line = Marker()
            line.header.frame_id = self.target_frame
            line.header.stamp = rospy.Time.now()
            line.ns = 'order'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.5, 0.0, 0.8
            line.pose.orientation.w = 1.0
            from geometry_msgs.msg import Point as GPoint
            for idx in self.order:
                t = self.targets[idx]
                line.points.append(GPoint(t.x, t.y, t.z + 0.3))
            ma.markers.append(line)
        self.marker_pub.publish(ma)


def main():
    rospy.init_node('ugv_target_tour')
    TargetTour()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
