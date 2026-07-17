#!/usr/bin/env python3
"""Execute a configurable coverage route while UAV mapping stays online.

The route and completion loiter contain no mine truth.  They are generic
survey-area configuration; detections remain entirely camera/model based.
"""

import copy
import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from uav_truth_tracker.msg import MineMap


def survey_completion_ready(
    map_received, confirmed_ids, candidate_ids, minimum_confirmed_mines
):
    """Return whether camera-derived mapping is ready for mission dispatch."""
    return (
        bool(map_received)
        and len(confirmed_ids) >= int(minimum_confirmed_mines)
        and not candidate_ids
    )


def ordered_route(waypoints, pass_index):
    """Alternate a generic coverage route without embedding mine locations."""
    route = list(waypoints)
    if int(pass_index) % 2:
        route.reverse()
    return route


class MineSurveyWaypoints:
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/uav/survey_goal")
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose"
        )
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.flight_height = float(rospy.get_param("~flight_height", 4.0))
        self.reach_radius = float(rospy.get_param("~reach_radius", 1.2))
        self.start_altitude = float(rospy.get_param("~start_altitude", 2.0))
        self.start_delay = float(rospy.get_param("~start_delay", 5.0))
        self.dwell = float(rospy.get_param("~dwell", 4.0))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.candidate_hold_duration = max(
            0.0, float(rospy.get_param("~candidate_hold_duration", 18.0))
        )
        self.candidate_map_topic = rospy.get_param(
            "~candidate_map_topic", "/mine_detection/map"
        )
        self.minimum_confirmed_mines = max(
            0, int(rospy.get_param("~minimum_confirmed_mines", 5))
        )
        self.max_rescan_rounds = max(
            0, int(rospy.get_param("~max_rescan_rounds", 3))
        )
        self.rescan_wait_duration = max(
            1.0, float(rospy.get_param("~rescan_wait_duration", 10.0))
        )
        self.status_period = max(
            0.5, float(rospy.get_param("~status_period", 2.0))
        )
        self.completion_loiter = (
            float(rospy.get_param("~completion_loiter_x", 0.0)),
            float(rospy.get_param("~completion_loiter_y", -6.0)),
        )
        self.completion_loiter_dwell = max(
            0.0, float(rospy.get_param("~completion_loiter_dwell", 3.0))
        )
        self.completion_loiter_timeout = max(
            1.0, float(rospy.get_param("~completion_loiter_timeout", 90.0))
        )
        raw_waypoints = rospy.get_param(
            "~waypoints",
            [[2.5, -0.5], [5.5, 0.4], [8.5, -0.4], [11.5, 0.5], [2.5, -0.5]],
        )
        self.waypoints = [(float(item[0]), float(item[1])) for item in raw_waypoints]
        if not self.waypoints:
            raise ValueError("survey waypoints cannot be empty")

        self.pose = None
        self.hold_pose = None
        self.hold_until_wall = 0.0
        self.hold_candidate_ids = set()
        self.seen_candidate_ids = set()
        self.map_received = False
        self.map_revision = 0
        self.map_last_wall = None
        self.confirmed_ids = set()
        self.unconfirmed_ids = set()
        self.manual_override = False
        self.manual_override_started_wall = None
        self.lock = threading.Lock()
        self.goal_pub = rospy.Publisher(
            self.goal_topic, PoseStamped, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/mine_survey/status", String, queue_size=1, latch=True
        )
        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=5)
        rospy.Subscriber(
            self.candidate_map_topic, MineMap, self._mine_map_cb, queue_size=5
        )
        rospy.Subscriber(
            rospy.get_param(
                "~goal_arbiter_status_topic", "/uav/goal_arbiter/status"
            ),
            String,
            self._arbiter_status_cb,
            queue_size=5,
        )
        threading.Thread(target=self._run, daemon=True).start()
        rospy.loginfo(
            "[MineSurvey] route points=%d topic=%s dwell=%.1fs "
            "minimum_confirmed=%d max_rescans=%d loiter=(%.1f,%.1f) "
            "(coverage only, no truth)",
            len(self.waypoints),
            self.goal_topic,
            self.dwell,
            self.minimum_confirmed_mines,
            self.max_rescan_rounds,
            self.completion_loiter[0],
            self.completion_loiter[1],
        )

    def _pose_cb(self, msg):
        with self.lock:
            self.pose = msg

    def _mine_map_cb(self, msg):
        """Track completion state and request a bounded candidate hover."""
        confirmed = {int(mine.id) for mine in msg.mines if mine.confirmed}
        unconfirmed = {int(mine.id) for mine in msg.mines if not mine.confirmed}
        with self.lock:
            self.map_received = True
            self.map_revision = int(msg.revision)
            self.map_last_wall = time.monotonic()
            self.confirmed_ids = confirmed
            self.unconfirmed_ids = unconfirmed

            # Confirmation/pruning of every candidate being verified ends the
            # hold immediately; there is no reason to consume its wall budget.
            if (
                self.hold_candidate_ids
                and not (self.hold_candidate_ids & unconfirmed)
            ):
                self.hold_pose = None
                self.hold_until_wall = 0.0
                self.hold_candidate_ids.clear()
            if self.candidate_hold_duration <= 0.0:
                return
            new_ids = unconfirmed - self.seen_candidate_ids
            if not new_ids:
                return
            self.seen_candidate_ids.update(new_ids)
            self.hold_candidate_ids.update(new_ids)
            # Do not hold the latest UAV pose here: CPU inference can finish
            # hundreds of milliseconds after capture, by which time the UAV
            # may be metres away. MineMap is already in this survey's
            # map_frame, so hover over the candidate cluster itself.
            candidates = [
                mine for mine in msg.mines
                if int(mine.id) in new_ids and not mine.confirmed
            ]
            if not candidates:
                return
            hold = PoseStamped()
            hold.header.stamp = rospy.Time.now()
            hold.header.frame_id = self.frame_id
            hold.pose.position.x = sum(mine.position.x for mine in candidates) / len(candidates)
            hold.pose.position.y = sum(mine.position.y for mine in candidates) / len(candidates)
            hold.pose.position.z = self.flight_height
            hold.pose.orientation.w = 1.0
            self.hold_pose = hold
            self.hold_until_wall = time.monotonic() + self.candidate_hold_duration
        rospy.logwarn(
            "[MineSurvey] new candidate(s) %s; hover %.1fs for multi-frame confirmation",
            sorted(new_ids),
            self.candidate_hold_duration,
        )

    def _arbiter_status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            active = bool(payload.get("manual_active", False))
        except (TypeError, ValueError):
            return
        now = time.monotonic()
        with self.lock:
            if active == self.manual_override:
                return
            if active:
                self.manual_override_started_wall = now
            else:
                if self.manual_override_started_wall is not None:
                    paused = max(0.0, now - self.manual_override_started_wall)
                    if self.hold_pose is not None:
                        self.hold_until_wall += paused
                self.manual_override_started_wall = None
            self.manual_override = active
        rospy.loginfo(
            "[MineSurvey] %s manual UAV goal override",
            "paused for" if active else "resumed after",
        )

    def _manual_override_active(self):
        with self.lock:
            return bool(self.manual_override)

    def _snapshot(self):
        with self.lock:
            return self.pose

    def _hold_snapshot(self):
        with self.lock:
            if self.hold_pose is None:
                return None, []
            if time.monotonic() >= self.hold_until_wall:
                self.hold_pose = None
                self.hold_candidate_ids.clear()
                return None, []
            return copy.deepcopy(self.hold_pose), sorted(self.hold_candidate_ids)

    def _map_snapshot(self):
        with self.lock:
            age = None
            if self.map_last_wall is not None:
                age = max(0.0, time.monotonic() - self.map_last_wall)
            return {
                "received": bool(self.map_received),
                "revision": int(self.map_revision),
                "age": age,
                "confirmed_ids": set(self.confirmed_ids),
                "candidate_ids": set(self.unconfirmed_ids),
            }

    def _completion_snapshot(self):
        snapshot = self._map_snapshot()
        ready = survey_completion_ready(
            snapshot["received"],
            snapshot["confirmed_ids"],
            snapshot["candidate_ids"],
            self.minimum_confirmed_mines,
        )
        return ready, snapshot

    def _diagnostic_text(self, snapshot=None):
        if snapshot is None:
            snapshot = self._map_snapshot()
        candidates = sorted(snapshot["candidate_ids"])
        candidate_text = (
            ",".join(str(value) for value in candidates) if candidates else "none"
        )
        age = snapshot["age"]
        age_text = "none" if age is None else "{:.1f}s".format(age)
        return "confirmed={}/{} candidates={} revision={} map_age={}".format(
            len(snapshot["confirmed_ids"]),
            self.minimum_confirmed_mines,
            candidate_text,
            snapshot["revision"],
            age_text,
        )

    def _publish_waiting(self, reason, extra=""):
        detail = self._diagnostic_text()
        if extra:
            detail = "{} {}".format(detail, extra)
        self.status_pub.publish(
            String(data="WAITING reason={} {}".format(reason, detail))
        )

    def _publish_goal(self, index, total, x, y, phase, pass_number):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = self.flight_height
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.status_pub.publish(
            String(
                data=(
                    "{} pass={} waypoint={}/{} goal=({:.2f},{:.2f}) {}".format(
                        phase,
                        pass_number,
                        index + 1,
                        total,
                        x,
                        y,
                        self._diagnostic_text(),
                    )
                )
            )
        )

    def _publish_candidate_hold(self, pose, candidate_ids):
        goal = copy.deepcopy(pose)
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose.position.z = max(goal.pose.position.z, self.flight_height)
        self.goal_pub.publish(goal)
        self.status_pub.publish(
            String(
                data=(
                    "VERIFYING_CANDIDATE ids={} hover=({:.2f},{:.2f}) {}".format(
                        ",".join(str(value) for value in candidate_ids),
                        goal.pose.position.x,
                        goal.pose.position.y,
                        self._diagnostic_text(),
                    )
                )
            )
        )

    def _wait_before_rescan(self, reason, extra=""):
        remaining = self.rescan_wait_duration
        previous_wall = time.monotonic()
        last_status = 0.0
        while not rospy.is_shutdown() and remaining > 0.0:
            ready, _snapshot = self._completion_snapshot()
            if ready:
                return True
            now = time.monotonic()
            if not self._manual_override_active():
                remaining -= max(0.0, now - previous_wall)
            previous_wall = now
            if now - last_status >= self.status_period:
                countdown = "next_rescan_in={:.1f}s".format(max(0.0, remaining))
                self._publish_waiting(
                    reason, "{} {}".format(extra, countdown).strip()
                )
                last_status = now
            time.sleep(0.1)
        ready, _snapshot = self._completion_snapshot()
        return ready

    def _run_route(self, route, phase, pass_number):
        total = len(route)
        direction = "reverse" if (pass_number % 2) else "forward"
        rospy.logwarn(
            "[MineSurvey] %s pass=%d direction=%s %s",
            phase,
            pass_number,
            direction,
            self._diagnostic_text(),
        )
        for index, (x, y) in enumerate(route):
            started = time.monotonic()
            last_publish = 0.0
            reached = False
            while not rospy.is_shutdown():
                ready, _snapshot = self._completion_snapshot()
                if ready:
                    return True
                now = time.monotonic()
                if self._manual_override_active():
                    started = now
                    time.sleep(0.1)
                    continue
                hold_pose, hold_ids = self._hold_snapshot()
                if hold_pose is not None:
                    # Verification dwell and manual override do not consume a
                    # waypoint timeout budget.
                    started = now
                    if now - last_publish >= 1.0:
                        self._publish_candidate_hold(hold_pose, hold_ids)
                        last_publish = now
                    time.sleep(0.1)
                    continue
                if now - last_publish >= 2.0:
                    self._publish_goal(index, total, x, y, phase, pass_number)
                    last_publish = now
                pose = self._snapshot()
                if pose is not None:
                    distance = math.hypot(
                        pose.pose.position.x - x, pose.pose.position.y - y
                    )
                    if distance <= self.reach_radius:
                        reached = True
                        break
                if now - started >= self.goal_timeout:
                    rospy.logwarn(
                        "[MineSurvey] %s pass=%d waypoint %d/%d timed out; "
                        "continuing coverage",
                        phase,
                        pass_number,
                        index + 1,
                        total,
                    )
                    break
                time.sleep(0.2)
            if rospy.is_shutdown():
                return False
            if reached:
                rospy.loginfo(
                    "[MineSurvey] reached %s pass=%d waypoint=%d/%d "
                    "(%.2f, %.2f), dwell %.1fs",
                    phase,
                    pass_number,
                    index + 1,
                    total,
                    x,
                    y,
                    self.dwell,
                )
                dwell_remaining = self.dwell
                previous_wall = time.monotonic()
                while not rospy.is_shutdown() and dwell_remaining > 0.0:
                    ready, _snapshot = self._completion_snapshot()
                    if ready:
                        return True
                    now = time.monotonic()
                    if not self._manual_override_active():
                        dwell_remaining -= max(0.0, now - previous_wall)
                    previous_wall = now
                    time.sleep(0.1)
        ready, _snapshot = self._completion_snapshot()
        return ready

    def _move_to_completion_loiter(self):
        """Leave the mine coverage strip before releasing the UGV mission."""
        x, y = self.completion_loiter
        started = time.monotonic()
        last_publish = 0.0
        dwell_remaining = self.completion_loiter_dwell
        previous_wall = time.monotonic()
        while not rospy.is_shutdown():
            ready, _snapshot = self._completion_snapshot()
            if not ready:
                rospy.logwarn(
                    "[MineSurvey] completion revoked while moving to loiter: %s",
                    self._diagnostic_text(),
                )
                return False
            now = time.monotonic()
            if self._manual_override_active():
                started = now
                previous_wall = now
                time.sleep(0.1)
                continue
            if now - last_publish >= 1.0:
                goal = PoseStamped()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = self.frame_id
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = self.flight_height
                goal.pose.orientation.w = 1.0
                self.goal_pub.publish(goal)
                self.status_pub.publish(
                    String(
                        data=(
                            "LOITER goal=({:.2f},{:.2f}) dwell={:.1f}s {}".format(
                                x,
                                y,
                                max(0.0, dwell_remaining),
                                self._diagnostic_text(),
                            )
                        )
                    )
                )
                last_publish = now
            pose = self._snapshot()
            reached = False
            if pose is not None:
                reached = (
                    math.hypot(
                        pose.pose.position.x - x, pose.pose.position.y - y
                    )
                    <= self.reach_radius
                )
            if reached:
                dwell_remaining -= max(0.0, now - previous_wall)
                if dwell_remaining <= 0.0:
                    return True
            else:
                dwell_remaining = self.completion_loiter_dwell
            previous_wall = now
            if now - started >= self.completion_loiter_timeout:
                rospy.logerr(
                    "[MineSurvey] completion loiter timed out; withholding COMPLETE"
                )
                return False
            time.sleep(0.1)
        return False

    def _attempt_completion(self):
        ready, _snapshot = self._completion_snapshot()
        if not ready or not self._move_to_completion_loiter():
            return False
        ready, _snapshot = self._completion_snapshot()
        if not ready:
            return False
        # Keep this payload exact: detector, arbiter and mission manager use it
        # as a latched protocol token.
        self.status_pub.publish(String(data="COMPLETE"))
        rospy.logwarn(
            "[MineSurvey] mapping criteria COMPLETE at safe loiter: %s",
            self._diagnostic_text(),
        )
        return True

    def _run(self):
        last_wait_status = 0.0
        while not rospy.is_shutdown():
            pose = self._snapshot()
            if pose is not None and pose.pose.position.z >= self.start_altitude:
                break
            now = time.monotonic()
            if now - last_wait_status >= self.status_period:
                self._publish_waiting("awaiting_takeoff")
                last_wait_status = now
            time.sleep(0.2)
        if rospy.is_shutdown():
            return
        deadline = time.monotonic() + self.start_delay
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            time.sleep(0.1)

        # Initial coverage is followed by a bounded number of explicitly
        # labelled reverse/forward rescans. Exhausting those rounds never means
        # success: cooled maintenance coverage continues until the camera map
        # criteria genuinely pass.
        pass_index = 0
        mapping_ready = self._run_route(
            ordered_route(self.waypoints, pass_index), "ACTIVE", pass_index
        )
        if mapping_ready and self._attempt_completion():
            return

        for rescan_round in range(1, self.max_rescan_rounds + 1):
            mapping_ready = self._wait_before_rescan(
                "mapping_incomplete",
                "bounded_rescan={}/{}".format(
                    rescan_round, self.max_rescan_rounds
                ),
            )
            if mapping_ready and self._attempt_completion():
                return
            pass_index += 1
            mapping_ready = self._run_route(
                ordered_route(self.waypoints, pass_index),
                "RESCAN",
                pass_index,
            )
            if mapping_ready and self._attempt_completion():
                return

        maintenance_pass = 0
        while not rospy.is_shutdown():
            maintenance_pass += 1
            mapping_ready = self._wait_before_rescan(
                "bounded_rescans_exhausted",
                "maintenance_pass={}".format(maintenance_pass),
            )
            if mapping_ready and self._attempt_completion():
                return
            pass_index += 1
            mapping_ready = self._run_route(
                ordered_route(self.waypoints, pass_index),
                "RESCAN",
                pass_index,
            )
            if mapping_ready and self._attempt_completion():
                return


def main():
    rospy.init_node("mine_survey_waypoints")
    MineSurveyWaypoints()
    rospy.spin()


if __name__ == "__main__":
    main()
