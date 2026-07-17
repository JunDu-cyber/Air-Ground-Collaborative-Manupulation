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
import json
import math
import threading

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform)
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from mobile_manipulator.msg import WorldTarget


class Tgt(object):
    __slots__ = ('x', 'y', 'z', 'cls', 'conf')

    def __init__(self, x, y, z, cls, conf):
        self.x, self.y, self.z, self.cls, self.conf = x, y, z, cls, conf


class TargetTour(object):
    COLLECT = 'COLLECT'
    NAV_MINE = 'NAV_TO_MINE'
    ALIGN = 'ALIGN'
    GRASP = 'GRASP'
    NAV_HOME = 'NAV_HOME'
    PLACE = 'PLACE'
    DONE = 'DONE'
    FAILED = 'FAILED'
    ACTIVE = (NAV_MINE, ALIGN, GRASP, NAV_HOME, PLACE)

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

        # ARRIVED -> GRASP. Default OFF, so a nav-only tour is byte-for-byte unchanged.
        # When on, the tour calls the grasp service on arrival and only advances afterwards.
        # NOTE: this is the NO-ALIGNMENT path -- it drives to the coarse target and grasps from
        # wherever nav parked, so it only succeeds when the mine happens to land in the arm's
        # reachable band (x in [0.60, 1.05] from base_link). The visual fine-alignment that would
        # guarantee that (mine_align) is a separate, not-yet-working stage; this wires the full
        # air-ground pipeline end to end without waiting on it.
        self.grasp_on_arrival = bool(rospy.get_param('~grasp_on_arrival', False))
        self.grasp_srv = rospy.get_param('~grasp_service', '/grasp/execute')
        self.grasp_wait = float(rospy.get_param('~grasp_wait', 180.0))
        self.align_before_grasp = bool(rospy.get_param('~align_before_grasp', True))
        self.align_srv = rospy.get_param('~align_service', '/ugv/align_to_mine')
        self.place_srv = rospy.get_param('~place_service', '/grasp/place')
        self.home_tolerance = float(rospy.get_param('~home_tolerance', 0.6))
        self.auto_status_topic = rospy.get_param('~auto_start_status_topic',
                                                  '/mine_survey/status')
        self.auto_status_token = rospy.get_param('~auto_start_status_token', 'COMPLETE')
        self.auto_start_requested = False
        self.working = False

        self.lock = threading.Lock()
        self.targets = []          # collected, in odom
        self.state = self.COLLECT
        self.order = []            # indices into self.targets, visiting order
        self.cur = 0
        self.odom = None           # (x, y)
        self.home = None           # first stable odometry pose; mission origin
        self.terrain = None        # latest /terrain_map PointCloud2
        self.goal_start = 0.0
        self.last_pub = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/target_tour_markers', MarkerArray,
                                          queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/ugv/tour_status', String,
                                          queue_size=1, latch=True)
        rospy.Subscriber(self.detected_topic, WorldTarget, self._target_cb, queue_size=20)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(self.terrain_topic, PointCloud2, self._terrain_cb, queue_size=1)
        if self.auto_status_topic:
            rospy.Subscriber(self.auto_status_topic, String, self._survey_status_cb,
                             queue_size=2)
        rospy.Service('/ugv/start_tour', Trigger, self._start_cb)
        rospy.Timer(rospy.Duration(0.2), self._loop)

        rospy.loginfo('[target_tour] collecting on %s -> drive %s ; call /ugv/start_tour to begin',
                      self.detected_topic, self.goal_topic)
        self._publish_status('waiting for confirmed targets and survey completion')

    # ---------- inputs ----------
    def _odom_cb(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.home is None:
            self.home = self.odom
            rospy.logwarn('[target_tour] latched mission origin at odom=(%.3f, %.3f)',
                          self.home[0], self.home[1])

    def _terrain_cb(self, msg):
        self.terrain = msg

    def _survey_status_cb(self, msg):
        if msg.data.strip() != self.auto_status_token:
            return
        with self.lock:
            self.auto_start_requested = True
        rospy.logwarn('[target_tour] survey COMPLETE received; auto-start armed')

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
            ok, msg = self._begin_tour_locked()
            return TriggerResponse(success=ok, message=msg)

    def _begin_tour_locked(self):
        if not self.targets:
            return False, 'no targets collected'
        if self.home is None:
            return False, 'mission origin not latched yet'
        if self.state in self.ACTIVE:
            return False, 'tour already running'
        if self.state in (self.DONE, self.FAILED):
            return False, 'tour already finished; restart the node for a new mission'
        self.order = self._compute_order()
        self.cur = 0
        self._set_state(self.NAV_MINE, 'tour started')
        self._publish_current_mine()
        msg = 'tour started: %d targets, order=%s' % (len(self.order), self.order)
        rospy.logwarn('[target_tour] %s', msg)
        return True, msg

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
        """Each successful pick returns home, so sort from home, unsafe last."""
        hx, hy = self.home
        scored = []
        for i, target in enumerate(self.targets):
            cost = self._terrain_cost(target.x, target.y)
            deferred = cost is not None and cost > self.obstacle_cost_thre
            distance2 = (target.x - hx) ** 2 + (target.y - hy) ** 2
            scored.append((deferred, distance2, i))
        deferred_count = sum(1 for item in scored if item[0])
        if deferred_count:
            rospy.logwarn('[target_tour] %d target(s) on untraversable terrain -> deprioritized',
                          deferred_count)
        return [item[2] for item in sorted(scored)]

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

    def _publish_home(self):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.target_frame
        ps.pose.position.x, ps.pose.position.y = self.home
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)
        self.last_pub = rospy.Time.now().to_sec()

    def _set_state(self, state, detail=''):
        self.state = state
        self.goal_start = rospy.Time.now().to_sec()
        self.last_pub = 0.0
        self._publish_status(detail)

    def _publish_status(self, detail=''):
        payload = {
            'state': self.state,
            'current': min(self.cur + 1, len(self.order)) if self.order else 0,
            'total': len(self.order),
            'detail': detail,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_current_mine(self):
        if self.cur >= len(self.order):
            self._set_state(self.DONE, 'all mines placed at origin')
            rospy.logwarn('[target_tour] mission complete: %d mines processed', len(self.order))
            return
        self._publish_goal(self.order[self.cur])

    def _loop(self, _evt):
        with self.lock:
            self._publish_markers()
            if (self.state == self.COLLECT and self.auto_start_requested and
                    self.targets and self.home is not None):
                self._begin_tour_locked()
            if self.state not in (self.NAV_MINE, self.NAV_HOME) or self.odom is None:
                return
            if self.working:
                return
            now = rospy.Time.now().to_sec()
            if self.state == self.NAV_HOME:
                d = math.hypot(self.home[0] - self.odom[0], self.home[1] - self.odom[1])
                if d < self.home_tolerance:
                    self.working = True
                    self._set_state(self.PLACE, 'arrived at origin')
                    threading.Thread(target=self._place_then_continue, daemon=True).start()
                elif now - self.goal_start > self.goal_timeout:
                    self._set_state(self.FAILED, 'home navigation timed out while carrying')
                    rospy.logerr('[target_tour] FAILED: could not return home while carrying')
                elif now - self.last_pub > self.republish_period:
                    self._publish_home()
                return

            idx = self.order[self.cur]
            t = self.targets[idx]
            d = math.hypot(t.x - self.odom[0], t.y - self.odom[1])
            if d < self.reach_tolerance:
                rospy.loginfo('[target_tour] reached %d/%d %s (d=%.2f)',
                              self.cur + 1, len(self.order), t.cls, d)
                if self.grasp_on_arrival:
                    self.working = True
                    threading.Thread(target=self._align_grasp_then_return,
                                     daemon=True).start()
                else:
                    self._advance_to_next('visited without grasp')
            elif now - self.goal_start > self.goal_timeout:
                rospy.logwarn('[target_tour] %d/%d timeout (d=%.2f) -> skip',
                              self.cur + 1, len(self.order), d)
                self._advance_to_next('mine navigation timed out')
            elif now - self.last_pub > self.republish_period:
                self._publish_goal(idx)

    @staticmethod
    def _call_trigger(service, timeout):
        try:
            rospy.wait_for_service(service, timeout=timeout)
            response = rospy.ServiceProxy(service, Trigger)()
            return bool(response.success), response.message
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)

    def _align_grasp_then_return(self):
        ok, msg = True, 'alignment disabled'
        if self.align_before_grasp:
            with self.lock:
                self._set_state(self.ALIGN, 're-acquiring mine with wrist camera')
            ok, msg = self._call_trigger(self.align_srv, self.grasp_wait)
            rospy.loginfo('[target_tour] align %s: %s', 'OK' if ok else 'FAILED', msg)
        if ok:
            with self.lock:
                self._set_state(self.GRASP, 'picking mine in carry mode')
            ok, msg = self._call_trigger(self.grasp_srv, self.grasp_wait)
            rospy.loginfo('[target_tour] grasp %s: %s', 'OK' if ok else 'FAILED', msg)
        with self.lock:
            self.working = False
            if ok:
                self._set_state(self.NAV_HOME, 'mine held; returning to origin')
                self._publish_home()
            else:
                self._advance_to_next('alignment/grasp failed: %s' % msg)

    def _place_then_continue(self):
        ok, msg = self._call_trigger(self.place_srv, self.grasp_wait)
        rospy.loginfo('[target_tour] place %s: %s', 'OK' if ok else 'FAILED', msg)
        with self.lock:
            self.working = False
            if not ok:
                self._set_state(self.FAILED, 'place failed at origin: %s' % msg)
                return
            self._advance_to_next('mine placed at origin')

    def _advance_to_next(self, detail):
        self.cur += 1
        if self.cur >= len(self.order):
            self._set_state(self.DONE, detail)
            rospy.logwarn('[target_tour] mission DONE (%d targets)', len(self.order))
        else:
            self._set_state(self.NAV_MINE, detail)
            self._publish_current_mine()

    # ---------- viz ----------
    def _publish_markers(self):
        ma = MarkerArray()
        visited = set(self.order[:self.cur]) if self.state != self.COLLECT else set()
        cur_idx = self.order[self.cur] if (self.state in self.ACTIVE and
                                           self.cur < len(self.order)) else -1
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
