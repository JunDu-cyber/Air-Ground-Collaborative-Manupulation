#!/usr/bin/env python3
"""Simulation-only MineGrasp action server.

Modes let the complete mission be tested before the teammate's real arm server
is ready: success, failure, fatal, and timeout.
"""

import math
import time
from typing import Optional, Tuple

import actionlib
import rospy
from gazebo_msgs.srv import DeleteModel, GetModelState, GetWorldProperties
from std_msgs.msg import Bool

from mobile_manipulator.msg import (
    MineGraspAction,
    MineGraspFeedback,
    MineGraspResult,
)


class MockMineGraspServer:
    def __init__(self) -> None:
        self.action_name = rospy.get_param("~action_name", "/mine_grasp")
        self.mode = str(rospy.get_param("~mode", "success")).lower()
        self.delay = max(0.0, float(rospy.get_param("~delay", 2.0)))
        self.delete_model = bool(rospy.get_param("~delete_gazebo_model", True))
        self.model_prefix = rospy.get_param("~model_prefix", "landmine_")
        self.delete_match_radius = float(
            rospy.get_param("~delete_match_radius", 0.75)
        )
        self.retained_pub = rospy.Publisher(
            "/mine_grasp/retained", Bool, queue_size=1, latch=True
        )
        self.retained_pub.publish(Bool(data=False))
        self.server = actionlib.SimpleActionServer(
            self.action_name,
            MineGraspAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        rospy.logwarn(
            "[MockMineGrasp] SIMULATION ONLY action=%s mode=%s delete_model=%s",
            self.action_name,
            self.mode,
            self.delete_model,
        )

    def _feedback(self, stage: int, progress: float, detail: str) -> None:
        feedback = MineGraspFeedback()
        feedback.stage = stage
        feedback.progress = progress
        feedback.detail = detail
        self.server.publish_feedback(feedback)

    def _sleep_preemptible(self, duration: float) -> bool:
        deadline = time.monotonic() + duration
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.server.is_preempt_requested():
                result = MineGraspResult()
                result.outcome = MineGraspResult.CANCELLED
                result.success = False
                result.message = "mock action preempted"
                self.server.set_preempted(result, result.message)
                return False
            time.sleep(0.05)
        return not rospy.is_shutdown()

    def _execute(self, goal) -> None:
        rospy.loginfo(
            "[MockMineGrasp] goal M%03d at (%.2f, %.2f), mode=%s",
            goal.mine_id,
            goal.mine_pose.pose.position.x,
            goal.mine_pose.pose.position.y,
            self.mode,
        )
        if goal.operation == goal.PLACE:
            self._execute_place(goal)
            return
        if self.mode == "timeout":
            self._feedback(MineGraspFeedback.WAITING, 0.0, "intentional timeout mode")
            while not rospy.is_shutdown():
                if not self._sleep_preemptible(0.2):
                    return
            return

        stages = [
            (MineGraspFeedback.LOCALIZING, 0.20, "mock localizing"),
            (MineGraspFeedback.PLANNING, 0.45, "mock planning"),
            (MineGraspFeedback.GRASPING, 0.75, "mock grasping"),
            (MineGraspFeedback.VERIFYING, 0.95, "mock verifying retention"),
        ]
        per_stage = self.delay / max(1, len(stages))
        for stage, progress, detail in stages:
            self._feedback(stage, progress, detail)
            if not self._sleep_preemptible(per_stage):
                return

        result = MineGraspResult()
        if self.mode == "success":
            result.outcome = MineGraspResult.SUCCESS
            result.success = True
            result.message = "mock PICK and transport lock verification succeeded"
            self.retained_pub.publish(Bool(data=True))
            self.server.set_succeeded(result, result.message)
        elif self.mode == "fatal":
            result.outcome = MineGraspResult.FATAL_FAILURE
            result.success = False
            result.message = "intentional fatal mock failure"
            self.server.set_aborted(result, result.message)
        else:
            result.outcome = MineGraspResult.RETRYABLE_FAILURE
            result.success = False
            result.message = "intentional retryable mock failure"
            self.server.set_aborted(result, result.message)

    def _execute_place(self, goal) -> None:
        result = MineGraspResult()
        self._feedback(
            MineGraspFeedback.PLACING, 0.7, "mock controlled placement"
        )
        if not self._sleep_preemptible(self.delay):
            return
        if self.mode != "success":
            result.outcome = MineGraspResult.RETRYABLE_FAILURE
            result.success = False
            result.message = "intentional mock PLACE failure"
            self.server.set_aborted(result, result.message)
            return
        if self.delete_model:
            deleted, message = self._delete_nearest_model(goal)
            if not deleted:
                result.outcome = MineGraspResult.RETRYABLE_FAILURE
                result.success = False
                result.message = "mock PLACE could not remove Gazebo mine: " + message
                self.server.set_aborted(result, result.message)
                return
        self.retained_pub.publish(Bool(data=False))
        result.outcome = MineGraspResult.SUCCESS
        result.success = True
        result.message = "mock PLACE released at HOME depot"
        self.server.set_succeeded(result, result.message)
    def _delete_nearest_model(self, goal) -> Tuple[bool, str]:
        try:
            rospy.wait_for_service("/gazebo/get_world_properties", timeout=2.0)
            rospy.wait_for_service("/gazebo/get_model_state", timeout=2.0)
            rospy.wait_for_service("/gazebo/delete_model", timeout=2.0)
            world = rospy.ServiceProxy(
                "/gazebo/get_world_properties", GetWorldProperties
            )()
            get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            target_x = goal.mine_pose.pose.position.x
            target_y = goal.mine_pose.pose.position.y
            nearest: Optional[Tuple[float, str]] = None
            for name in world.model_names:
                if not name.startswith(self.model_prefix):
                    continue
                state = get_state(name, "world")
                if not state.success:
                    continue
                distance = math.hypot(
                    state.pose.position.x - target_x,
                    state.pose.position.y - target_y,
                )
                if nearest is None or distance < nearest[0]:
                    nearest = (distance, name)
            if nearest is None:
                return False, "no landmine model found"
            if nearest[0] > self.delete_match_radius:
                return False, (
                    f"nearest model {nearest[1]} is {nearest[0]:.2f} m away"
                )
            response = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)(
                nearest[1]
            )
            if not response.success:
                return False, response.status_message
            rospy.logwarn(
                "[MockMineGrasp] deleted %s (distance %.2f m)",
                nearest[1],
                nearest[0],
            )
            return True, nearest[1]
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return False, str(exc)


def main() -> None:
    rospy.init_node("mock_mine_grasp_server")
    MockMineGraspServer()
    rospy.spin()


if __name__ == "__main__":
    main()
