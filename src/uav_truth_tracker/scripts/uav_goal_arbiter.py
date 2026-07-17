#!/usr/bin/env python3
"""Forward manual UAV goals; optionally arbitrate a resumable survey."""

import copy
import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class UavGoalArbiter:
    def __init__(self):
        self.manual_only = bool(rospy.get_param("~manual_only", True))
        self.manual_topic = rospy.get_param("~manual_goal_topic", "/uav/manual_goal")
        self.survey_topic = rospy.get_param("~survey_goal_topic", "/uav/survey_goal")
        self.output_topic = rospy.get_param("~output_goal_topic", "/uav/goal")
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose"
        )
        self.survey_status_topic = rospy.get_param(
            "~survey_status_topic", "/mine_survey/status"
        )
        self.manual_reach_radius = max(
            float(rospy.get_param("~manual_reach_radius", 1.0)), 0.1
        )
        self.manual_stable_duration = max(
            float(rospy.get_param("~manual_stable_duration", 5.0)), 0.5
        )
        self.manual_stable_speed = max(
            float(rospy.get_param("~manual_stable_speed", 0.25)), 0.01
        )
        self.lock = threading.RLock()
        self.manual_goal = None
        self.survey_goal = None
        self.active_goal = None
        self.pose = None
        self.previous_pose_sample = None
        self.estimated_speed = math.inf
        self.manual_active = False
        self.manual_stable_since = None
        self.survey_complete = self.manual_only
        self.last_publish_wall = 0.0

        self.goal_pub = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/uav/goal_arbiter/status", String, queue_size=2, latch=True
        )
        rospy.Subscriber(
            self.manual_topic, PoseStamped, self._manual_cb, queue_size=5
        )
        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=10)
        if not self.manual_only:
            rospy.Subscriber(
                self.survey_topic, PoseStamped, self._survey_cb, queue_size=5
            )
            rospy.Subscriber(
                self.survey_status_topic, String,
                self._survey_status_cb, queue_size=5,
            )
        rospy.Service("/uav/resume_survey", Trigger, self._resume_cb)
        self.worker = threading.Thread(target=self._wall_loop, daemon=True)
        self.worker.start()
        if self.manual_only:
            self._publish_status("WAITING_MANUAL", "manual goals only")
            rospy.logwarn(
                "[UavGoalArbiter] MANUAL-ONLY: /uav/survey_goal is disabled; "
                "waiting for /uav/manual_goal"
            )
        else:
            self._publish_status("WAITING", "waiting for manual or survey goal")

    def _valid_goal(self, msg):
        values = (
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        return bool(msg.header.frame_id) and all(math.isfinite(v) for v in values)

    def _manual_cb(self, msg):
        if not self._valid_goal(msg):
            rospy.logwarn("[UavGoalArbiter] ignored malformed manual goal")
            return
        with self.lock:
            self.manual_goal = copy.deepcopy(msg)
            self.manual_active = True
            self.manual_stable_since = None
            goal = copy.deepcopy(self.manual_goal)
        self._forward(goal, "MANUAL", "manual goal has priority")

    def _survey_cb(self, msg):
        if self.manual_only:
            rospy.logwarn_throttle(
                5.0, "[UavGoalArbiter] ignored survey goal in manual-only mode"
            )
            self._publish_status("WAITING_MANUAL", "survey goal rejected")
            return
        if not self._valid_goal(msg):
            return
        with self.lock:
            self.survey_goal = copy.deepcopy(msg)
            self.survey_complete = False
            manual_active = self.manual_active
            goal = copy.deepcopy(self.survey_goal)
        if manual_active:
            self._publish_status(
                "MANUAL", "survey goal buffered until manual dwell completes"
            )
            return
        self._forward(goal, "SURVEY", "automatic coverage goal")

    def _survey_status_cb(self, msg):
        if self.manual_only:
            return
        if msg.data.strip().upper() != "COMPLETE":
            return
        with self.lock:
            self.survey_complete = True
            self.survey_goal = None
        self._publish_status("SURVEY_COMPLETE", "no unfinished survey goal")

    def _pose_cb(self, msg):
        now = time.monotonic()
        with self.lock:
            if self.previous_pose_sample is not None:
                previous_wall, previous = self.previous_pose_sample
                elapsed = max(now - previous_wall, 1e-6)
                dx = msg.pose.position.x - previous.pose.position.x
                dy = msg.pose.position.y - previous.pose.position.y
                dz = msg.pose.position.z - previous.pose.position.z
                self.estimated_speed = math.sqrt(dx * dx + dy * dy + dz * dz) / elapsed
            self.previous_pose_sample = (now, copy.deepcopy(msg))
            self.pose = copy.deepcopy(msg)

    def _forward(self, goal, mode, detail):
        goal = copy.deepcopy(goal)
        goal.header.stamp = rospy.Time.now()
        with self.lock:
            self.active_goal = copy.deepcopy(goal)
            self.last_publish_wall = time.monotonic()
        self.goal_pub.publish(goal)
        self._publish_status(mode, detail)

    def _publish_status(self, mode, detail):
        with self.lock:
            payload = {
                "mode": mode,
                "detail": detail,
                "manual_only": self.manual_only,
                "manual_active": self.manual_active,
                "survey_buffered": self.survey_goal is not None,
                "survey_complete": self.survey_complete,
                "estimated_speed_m_s": (
                    self.estimated_speed
                    if math.isfinite(self.estimated_speed) else None
                ),
                "manual_stable_elapsed_s": (
                    0.0 if self.manual_stable_since is None
                    else time.monotonic() - self.manual_stable_since
                ),
            }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _resume_locked(self):
        self.manual_active = False
        self.manual_stable_since = None
        return copy.deepcopy(self.survey_goal)

    def _resume_cb(self, _request):
        if self.manual_only:
            self._publish_status(
                "WAITING_MANUAL", "survey resume rejected in manual-only mode"
            )
            return TriggerResponse(False, "manual-only mode: survey is disabled")
        with self.lock:
            goal = self._resume_locked()
        if goal is None:
            self._publish_status("IDLE", "no unfinished survey goal to resume")
            return TriggerResponse(False, "no unfinished survey goal")
        self._forward(goal, "SURVEY", "survey manually resumed")
        return TriggerResponse(True, "resumed buffered survey goal")

    def _update_manual_dwell(self):
        with self.lock:
            if not self.manual_active or self.manual_goal is None or self.pose is None:
                return
            dx = self.pose.pose.position.x - self.manual_goal.pose.position.x
            dy = self.pose.pose.position.y - self.manual_goal.pose.position.y
            # RViz 2D Nav Goal 的 z 固定为 0，而全局规划器会把 UAV 目标抬到
            # 巡航高度。到达判定若比较 z，会永远差约 4m 并每 2s 重发旧目标。
            distance = math.hypot(dx, dy)
            stable = (
                distance <= self.manual_reach_radius
                and self.estimated_speed <= self.manual_stable_speed
            )
            now = time.monotonic()
            if stable:
                if self.manual_stable_since is None:
                    self.manual_stable_since = now
                elapsed = now - self.manual_stable_since
            else:
                self.manual_stable_since = None
                elapsed = 0.0
            if elapsed < self.manual_stable_duration:
                return
            if self.manual_only:
                # 手动点到达并稳定后停止周期重发，无人机原地悬停等
                # 下一个手动点，不会恢复任何自动航线。
                self.manual_active = False
                self.manual_stable_since = None
                self.active_goal = None
                goal = None
                manual_complete = True
            else:
                goal = self._resume_locked()
                manual_complete = False
        if manual_complete:
            self._publish_status(
                "MANUAL_COMPLETE", "manual goal complete; hovering for next manual goal"
            )
            return
        if goal is not None:
            self._forward(
                goal, "SURVEY",
                "manual goal stable for {:.1f}s; survey resumed".format(
                    self.manual_stable_duration
                ),
            )
        else:
            self._publish_status(
                "MANUAL_COMPLETE", "manual dwell complete; no survey pending"
            )

    def _wall_loop(self):
        while not rospy.is_shutdown():
            self._update_manual_dwell()
            with self.lock:
                goal = copy.deepcopy(self.active_goal)
                # 手动模式的 publisher 已经 latch，一次发布即可。周期重发会让
                # 全局规划器每 2s 把同一目标当成新点重置路径/EGO 轨迹。
                due = (
                    not self.manual_only
                    and time.monotonic() - self.last_publish_wall >= 2.0
                )
            if goal is not None and due:
                goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(goal)
                with self.lock:
                    self.last_publish_wall = time.monotonic()
            time.sleep(0.1)


def main():
    rospy.init_node("uav_goal_arbiter")
    UavGoalArbiter()
    rospy.spin()


if __name__ == "__main__":
    main()
