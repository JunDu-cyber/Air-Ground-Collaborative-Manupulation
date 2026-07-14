#!/usr/bin/env python3

import copy
import math
import threading
import time
import unittest

import actionlib
import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from mobile_manipulator.msg import (
    MineGraspAction,
    MineGraspResult,
    MineMission,
    MineMissionEntry,
)
from uav_truth_tracker.msg import MineMap, MineMapEntry


class FakeMissionWorld:
    def __init__(self, mode):
        self.mode = mode
        self.lock = threading.Lock()
        self.robot_pose = PoseStamped()
        self.robot_pose.header.frame_id = "map"
        self.robot_pose.pose.orientation.w = 1.0
        self.arm_visits = []
        self.place_visits = []
        self.navigation_goals = []
        self.arm_robot_poses = []
        self.arm_attempts = {}
        self.fine_cmd = Twist()
        self.last_update_wall = time.monotonic()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_timer = rospy.Timer(rospy.Duration(0.02), self._broadcast_tf)
        self.plan_service = rospy.Service(
            "/move_base/make_plan", GetPlan, self._make_plan
        )
        self.move_server = actionlib.SimpleActionServer(
            "/move_base", MoveBaseAction, self._move, auto_start=False
        )
        self.arm_server = None
        self.move_server.start()
        self.fine_sub = rospy.Subscriber(
            "/mine_mission/fine_cmd_vel", Twist, self._fine_cmd_cb, queue_size=1
        )
        # server_gate intentionally leaves the entire Action namespace absent.
        # Constructing a stopped SimpleActionServer is insufficient because its
        # publishers can already make wait_for_server() appear connected.
        if self.mode != "server_gate":
            self.start_arm_server()
        self.retained_pub = rospy.Publisher(
            "/mine_grasp/retained", Bool, queue_size=1, latch=True
        )
        self.retained_pub.publish(Bool(data=False))
        self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=1, latch=True)
        imu = Imu()
        imu.orientation.w = 1.0
        self.imu_pub.publish(imu)

        self.map_pub = rospy.Publisher(
            "/terrain_map", OccupancyGrid, queue_size=1, latch=True
        )
        self.mine_pub = rospy.Publisher(
            "/mine_detection/map", MineMap, queue_size=1, latch=True
        )

    def _broadcast_tf(self, _event):
        with self.lock:
            now_wall = time.monotonic()
            dt = min(max(now_wall - self.last_update_wall, 0.0), 0.05)
            self.last_update_wall = now_wall
            q = self.robot_pose.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.robot_pose.pose.position.x += self.fine_cmd.linear.x * math.cos(yaw) * dt
            self.robot_pose.pose.position.y += self.fine_cmd.linear.x * math.sin(yaw) * dt
            yaw += self.fine_cmd.angular.z * dt
            quaternion = quaternion_from_euler(0.0, 0.0, yaw)
            self.robot_pose.pose.orientation.x = quaternion[0]
            self.robot_pose.pose.orientation.y = quaternion[1]
            self.robot_pose.pose.orientation.z = quaternion[2]
            self.robot_pose.pose.orientation.w = quaternion[3]
            pose = copy.deepcopy(self.robot_pose)
        transforms = []
        for child in ("base_link", "base_footprint"):
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"
            transform.child_frame_id = child
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = pose.pose.orientation
            transforms.append(transform)
        try:
            self.tf_broadcaster.sendTransform(transforms)
        except rospy.ROSException:
            # rostest closes publishers before timers during process teardown.
            pass

    def _fine_cmd_cb(self, msg):
        with self.lock:
            self.fine_cmd = copy.deepcopy(msg)

    @staticmethod
    def _make_plan(request):
        response = GetPlanResponse()
        response.plan = Path()
        response.plan.header.frame_id = "map"
        response.plan.poses = [copy.deepcopy(request.start), copy.deepcopy(request.goal)]
        return response

    def _move(self, goal):
        with self.lock:
            self.navigation_goals.append(copy.deepcopy(goal.target_pose))
            self.robot_pose = copy.deepcopy(goal.target_pose)
        if self.mode == "active_at_goal":
            # Reproduce move_base continuing to rotate even though TF says the
            # parking geometry is already correct. The manager must cancel it
            # after its stable-arrival window and proceed to PICK.
            while not rospy.is_shutdown():
                if self.move_server.is_preempt_requested():
                    self.move_server.set_preempted(MoveBaseResult())
                    return
                time.sleep(0.01)
            return
        # Let the updated transform reach the manager before reporting arrival.
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.move_server.is_preempt_requested():
                self.move_server.set_preempted(MoveBaseResult())
                return
            time.sleep(0.01)
        self.move_server.set_succeeded(MoveBaseResult())

    def _arm(self, goal):
        if goal.operation == goal.PLACE:
            with self.lock:
                self.place_visits.append(goal.mine_id)
            time.sleep(0.1)
            result = MineGraspResult()
            result.outcome = MineGraspResult.SUCCESS
            result.success = True
            result.message = "fake controlled place success"
            self.retained_pub.publish(Bool(data=False))
            self.arm_server.set_succeeded(result)
            return
        with self.lock:
            self.arm_visits.append(goal.mine_id)
            self.arm_robot_poses.append(copy.deepcopy(self.robot_pose))
            attempt = self.arm_attempts.get(goal.mine_id, 0) + 1
            self.arm_attempts[goal.mine_id] = attempt
        if self.mode == "safe_deferred" and goal.mine_id == 1 and attempt == 1:
            # Reproduce the executor contract for an empty failed PICK whose
            # measured recovery reached look.  This is safe to defer, unlike a
            # timeout/unverified arm pose or an object that remains retained.
            time.sleep(0.1)
            result = MineGraspResult()
            result.outcome = MineGraspResult.RETRYABLE_FAILURE
            result.success = False
            result.message = (
                '{"failure_reason":"CONTROLLER_UNSETTLED",'
                '"recovery_attempted":true,"recovery_success":true,'
                '"retained":false}'
            )
            self.retained_pub.publish(Bool(data=False))
            self.arm_server.set_aborted(result)
            return
        if self.mode in (
            "success", "server_gate", "active_at_goal", "safe_deferred"
        ):
            time.sleep(0.1)
            result = MineGraspResult()
            result.outcome = MineGraspResult.SUCCESS
            result.success = True
            result.message = "fake verified success"
            self.retained_pub.publish(Bool(data=True))
            self.arm_server.set_succeeded(result)
            return

        # Timeout mode intentionally never succeeds.  A cancelled arm pose is
        # unverified, so the manager must keep the base locked and request
        # manual reset without a second alignment/motion attempt.
        while not rospy.is_shutdown():
            if self.arm_server.is_preempt_requested():
                result = MineGraspResult()
                result.outcome = MineGraspResult.CANCELLED
                result.success = False
                result.message = "fake timeout preempted"
                self.arm_server.set_preempted(result)
                return
            time.sleep(0.02)

    def start_arm_server(self):
        if self.arm_server is not None:
            return
        self.arm_server = actionlib.SimpleActionServer(
            "/mine_grasp", MineGraspAction, self._arm, auto_start=False
        )
        self.arm_server.start()

    def publish_inputs(self):
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.1
        grid.info.width = 140
        grid.info.height = 140
        grid.info.origin.position.x = -2.0
        grid.info.origin.position.y = -7.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        self.map_pub.publish(grid)

        mines = MineMap()
        mines.header.frame_id = "map"
        mines.revision = 1
        positions = [(1, 2.0, 0.0)]
        if self.mode in ("success", "server_gate", "safe_deferred"):
            positions.append((2, 5.0, 0.0))
        for mine_id, x, y in positions:
            mine = MineMapEntry()
            mine.id = mine_id
            mine.position.x = x
            mine.position.y = y
            mine.position.z = 0.05
            mine.confidence = 0.95
            mine.observation_count = 4
            mine.confirmed = True
            mines.mines.append(mine)
        self.mine_pub.publish(mines)


class MineMissionIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.mode = rospy.get_param("~mode", "success")
        self.world = FakeMissionWorld(self.mode)
        self.status = None
        self.active = None
        self.lock_values = []
        self.condition = threading.Condition()
        rospy.Subscriber(
            "/mine_mission/status", MineMission, self._status_cb, queue_size=10
        )
        rospy.Subscriber(
            "/mine_hazard/active", PoseArray, self._active_cb, queue_size=10
        )
        rospy.Subscriber(
            "/mine_mission/base_lock", Bool, self._lock_cb, queue_size=10
        )
        time.sleep(0.4)
        self.world.publish_inputs()

    def tearDown(self):
        self.world.tf_timer.shutdown()

    def _status_cb(self, msg):
        with self.condition:
            self.status = msg
            self.condition.notify_all()

    def _active_cb(self, msg):
        with self.condition:
            self.active = msg
            self.condition.notify_all()

    def _lock_cb(self, msg):
        with self.condition:
            self.lock_values.append(msg.data)
            self.condition.notify_all()

    def _wait_for(self, predicate, timeout=20.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while not predicate() and time.monotonic() < deadline:
                self.condition.wait(timeout=0.1)
        return predicate()

    def test_mission_contract(self):
        success_mode = self.mode in (
            "success", "server_gate", "active_at_goal", "safe_deferred"
        )
        if self.mode == "server_gate":
            received = self._wait_for(
                lambda: self.status is not None and len(self.status.entries) == 2,
                timeout=5.0,
            )
            self.assertTrue(received, "confirmed mines were not ingested")
            time.sleep(1.0)
            with self.world.lock:
                self.assertEqual(
                    self.world.navigation_goals,
                    [],
                    "UGV moved before the manipulation Action existed",
                )
            self.assertTrue(self.lock_values and self.lock_values[-1])
            self.world.start_arm_server()

        if success_mode:
            expected_ids = (
                [1, 2]
                if self.mode in ("success", "server_gate", "safe_deferred")
                else [1]
            )
            finished = self._wait_for(
                lambda: self.status is not None
                and len(self.status.entries) == len(expected_ids)
                and all(
                    entry.state == MineMissionEntry.CLEARED
                    for entry in self.status.entries
                )
            )
            self.assertTrue(finished, "expected mines did not reach CLEARED")
            if self.mode == "safe_deferred":
                with self.world.lock:
                    arm_visits = list(self.world.arm_visits)
                    place_visits = list(self.world.place_visits)
                    arm_poses = copy.deepcopy(self.world.arm_robot_poses)
                self.assertEqual(
                    arm_visits,
                    [1, 2, 1],
                    "a safe empty M001 failure must yield to M002 before retry",
                )
                self.assertEqual(place_visits, [2, 1])
                self.assertGreater(
                    math.hypot(
                        arm_poses[2].pose.position.x
                        - arm_poses[0].pose.position.x,
                        arm_poses[2].pose.position.y
                        - arm_poses[0].pose.position.y,
                    ),
                    0.25,
                    "deferred retry replayed the same parking direction",
                )
            else:
                self.assertEqual(
                    self.world.arm_visits[:len(expected_ids)], expected_ids
                )
                self.assertEqual(
                    self.world.place_visits[:len(expected_ids)], expected_ids
                )
            self.assertTrue(
                self._wait_for(lambda: self.active is not None and not self.active.poses)
            )
        else:
            finished = self._wait_for(
                lambda: self.status is not None
                and len(self.status.entries) == 1
                and self.status.entries[0].state
                == MineMissionEntry.MANUAL_REQUIRED
            )
            self.assertTrue(finished, "timeout did not reach MANUAL_REQUIRED")
            self.assertEqual(self.world.arm_visits, [1])
            self.assertTrue(
                self._wait_for(
                    lambda: self.active is not None and len(self.active.poses) == 1
                )
            )

        # Every arm call must occur with the independent base lock asserted.
        # Success releases it for the next task; an exhausted PICK keeps it
        # locked and inhibits later mines until explicit recovery/reset.
        self.assertIn(True, self.lock_values)
        if success_mode:
            self.assertTrue(self._wait_for(lambda: self.lock_values[-1] is False))
        else:
            self.assertTrue(self.lock_values[-1])
        if self.mode == "safe_deferred":
            # M001 failed PICK has no return leg.  M002 and the retried M001
            # each have one delivery leg, giving navigation order P1,P2,D2,P1,D1.
            pick_goals = [
                self.world.navigation_goals[index] for index in (0, 1, 3)
            ]
        else:
            pick_goals = (
                self.world.navigation_goals[::2]
                if success_mode
                else self.world.navigation_goals
            )
        for goal in pick_goals:
            mine_x = 2.0 if goal.pose.position.x < 3.5 else 5.0
            self.assertAlmostEqual(
                math.hypot(goal.pose.position.x - mine_x, goal.pose.position.y),
                0.84,
                delta=0.02,
            )
        if success_mode:
            for mine_id, robot in zip(self.world.arm_visits, self.world.arm_robot_poses):
                mine_x = 2.0 if mine_id == 1 else 5.0
                dx = mine_x - robot.pose.position.x
                dy = -robot.pose.position.y
                self.assertAlmostEqual(math.hypot(dx, dy), 0.67, delta=0.025)
                q = robot.pose.orientation
                yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                desired = math.atan2(dy, dx)
                yaw_error = math.atan2(
                    math.sin(desired - yaw), math.cos(desired - yaw)
                )
                self.assertLessEqual(abs(math.degrees(yaw_error)), 3.2)


if __name__ == "__main__":
    rospy.init_node("mine_mission_integration_test")
    rostest.rosrun(
        "mobile_manipulator",
        "mine_mission_integration_test",
        MineMissionIntegrationTest,
    )
