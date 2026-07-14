#!/usr/bin/env python3
"""One-shot UR5 controller hold/sag and base-static gate.

This deliberately tests the configured gravity-on Gazebo/controller model; it
does not change PID, mass, gravity, transmissions, or controller parameters.
"""

import json
import math
import sys
import threading
import time

import actionlib
import rospy
import tf.transformations as tft
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTolerance,
    JointTrajectoryControllerState,
)
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from mine_grasp_minimal.motion_stability import (
    JointSpanWindow,
    classify_stability,
    maximum_absolute,
)


class ControllerHoldTest:
    def __init__(self):
        self.lock = threading.Lock()
        self.arm_names = list(rospy.get_param("~arm_joint_names", [
            "ur5_shoulder_pan_joint", "ur5_shoulder_lift_joint",
            "ur5_elbow_joint", "ur5_wrist_1_joint",
            "ur5_wrist_2_joint", "ur5_wrist_3_joint",
        ]))
        self.test_positions = list(rospy.get_param(
            "~m002_pregrasp_joint_positions",
            rospy.get_param(
                "~test_joint_positions", [0.0, 3.25, 2.75, -1.1, 0.1, 0.0]
            ),
        ))
        configured_targets = rospy.get_param("~test_targets", [])
        if configured_targets:
            self.test_targets = [
                {
                    "name": str(item["name"]),
                    "positions": [float(value) for value in item["positions"]],
                }
                for item in configured_targets
            ]
        else:
            self.test_targets = [
                {
                    "name": "look",
                    "positions": [
                        float(value) for value in rospy.get_param(
                            "~look_joint_positions"
                        )
                    ],
                },
                {
                    "name": "m002_pregrasp",
                    "positions": [float(value) for value in self.test_positions],
                },
            ]
        if any(len(item["positions"]) != len(self.arm_names)
               for item in self.test_targets):
            raise rospy.ROSInitException(
                "each controller hold target must provide all arm joints"
            )
        self.repetitions = max(int(rospy.get_param("~repetitions", 10)), 1)
        self.motion_duration = float(rospy.get_param("~motion_duration", 8.0))
        self.hold_duration = float(rospy.get_param("~hold_duration", 30.0))
        self.max_error = float(rospy.get_param(
            "~terminal_joint_error_tolerance", 0.01
        ))
        self.max_velocity = float(rospy.get_param(
            "~terminal_joint_velocity_tolerance", 0.05
        ))
        self.max_span = float(rospy.get_param(
            "~terminal_joint_span_tolerance", 0.002
        ))
        self.stability_duration = float(rospy.get_param(
            "~terminal_stability_duration", 0.5
        ))
        self.settle_timeout = min(max(float(rospy.get_param(
            "~motion_settle_timeout", 3.0
        )), self.stability_duration), 3.0)
        self.goal_tolerance = float(rospy.get_param(
            "~explicit_fjt_goal_tolerance", 0.01
        ))
        if self.goal_tolerance < 0.005:
            self.goal_tolerance = 0.01
        self.max_drift = float(rospy.get_param("~maximum_joint_drift", 0.01))
        self.action_wall_timeout_scale = min(max(float(rospy.get_param(
            "~action_wall_timeout_scale", 3.0
        )), 1.0), 5.0)
        self.action_result_grace = min(max(float(rospy.get_param(
            "~action_result_grace", 1.0
        )), 0.1), 2.0)
        self.max_roll = float(rospy.get_param("~max_base_roll_deg", 5.0))
        self.max_pitch = float(rospy.get_param("~max_base_pitch_deg", 5.0))
        self.wheel_threshold = float(rospy.get_param("~wheel_lift_threshold", 0.015))
        self.arm_state = None
        self.joint_state = None
        self.imu = None
        self.links = None
        self.max_controller_error = 0.0
        self.max_roll_seen = 0.0
        self.max_pitch_seen = 0.0
        self.wheel_baseline = {}
        self.max_wheel_rise = 0.0
        self.gripper_min = None
        self.gripper_max = None
        self.gripper_velocity_peak = 0.0
        self.gripper_velocity_square_sum = 0.0
        self.gripper_samples = 0

        self.report_pub = rospy.Publisher(
            "/mine_grasp/controller_hold_report", String, queue_size=1, latch=True
        )
        self.cmd_pubs = [
            rospy.Publisher(topic, Twist, queue_size=1)
            for topic in rospy.get_param("~cmd_vel_topics", [
                "/husky_velocity_controller/cmd_vel", "/cmd_vel"
            ])
        ]
        rospy.Subscriber(rospy.get_param("~arm_state_topic",
                                        "/ur5_arm_controller/state"),
                         JointTrajectoryControllerState, self._arm_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~gripper_state_topic",
                                        "/gripper_controller/state"),
                         JointTrajectoryControllerState, self._gripper_cb,
                         queue_size=1)
        rospy.Subscriber(rospy.get_param("~joint_states_topic", "/joint_states"),
                         JointState, self._joint_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~imu_topic", "/imu/data"),
                         Imu, self._imu_cb, queue_size=1)
        rospy.Subscriber("/gazebo/link_states", LinkStates,
                         self._links_cb, queue_size=1)
        self.arm_client = actionlib.SimpleActionClient(
            rospy.get_param("~arm_action",
                            "/ur5_arm_controller/follow_joint_trajectory"),
            FollowJointTrajectoryAction,
        )
        self.list_controllers = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers
        )

    def _arm_cb(self, msg):
        with self.lock:
            self.arm_state = msg
            if msg.error.positions:
                self.max_controller_error = max(
                    self.max_controller_error,
                    max(abs(value) for value in msg.error.positions),
                )

    def _joint_cb(self, msg):
        with self.lock:
            self.joint_state = msg

    def _gripper_cb(self, msg):
        if not msg.actual.positions:
            return
        position = float(msg.actual.positions[0])
        velocity = (
            float(msg.actual.velocities[0])
            if msg.actual.velocities else 0.0
        )
        with self.lock:
            self.gripper_min = (
                position if self.gripper_min is None
                else min(self.gripper_min, position)
            )
            self.gripper_max = (
                position if self.gripper_max is None
                else max(self.gripper_max, position)
            )
            self.gripper_velocity_peak = max(
                self.gripper_velocity_peak, abs(velocity)
            )
            self.gripper_velocity_square_sum += velocity * velocity
            self.gripper_samples += 1

    def _imu_cb(self, msg):
        quaternion = [msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w]
        roll, pitch, _ = tft.euler_from_quaternion(quaternion)
        with self.lock:
            self.imu = msg
            self.max_roll_seen = max(self.max_roll_seen, abs(math.degrees(roll)))
            self.max_pitch_seen = max(self.max_pitch_seen, abs(math.degrees(pitch)))

    def _links_cb(self, msg):
        heights = {}
        for name, pose in zip(msg.name, msg.pose):
            short = name.split("::")[-1]
            if short in {
                "front_left_wheel_link", "front_right_wheel_link",
                "rear_left_wheel_link", "rear_right_wheel_link",
            }:
                heights[short] = float(pose.position.z)
        with self.lock:
            self.links = msg
            if self.wheel_baseline:
                for name, baseline in self.wheel_baseline.items():
                    if name in heights:
                        self.max_wheel_rise = max(
                            self.max_wheel_rise, heights[name] - baseline
                        )

    def _zero_base(self):
        command = Twist()
        for publisher in self.cmd_pubs:
            publisher.publish(command)

    def _positions(self):
        with self.lock:
            state = self.joint_state
        if state is None:
            return None
        values = dict(zip(state.name, state.position))
        if any(name not in values for name in self.arm_names):
            return None
        return [float(values[name]) for name in self.arm_names]

    def _measurement(self):
        with self.lock:
            state = self.arm_state
            joint_state = self.joint_state
        if state is not None and state.actual.positions:
            positions = dict(zip(state.joint_names, state.actual.positions))
            velocities = dict(zip(state.joint_names, state.actual.velocities))
            if all(name in positions for name in self.arm_names):
                return (
                    [float(positions[name]) for name in self.arm_names],
                    ([float(velocities[name]) for name in self.arm_names]
                     if all(name in velocities for name in self.arm_names)
                     else None),
                    state.header.stamp.to_sec(),
                )
        if joint_state is None:
            return None, None, None
        positions = dict(zip(joint_state.name, joint_state.position))
        velocities = dict(zip(joint_state.name, joint_state.velocity))
        if not all(name in positions for name in self.arm_names):
            return None, None, None
        return (
            [float(positions[name]) for name in self.arm_names],
            ([float(velocities[name]) for name in self.arm_names]
             if all(name in velocities for name in self.arm_names)
             else None),
            joint_state.header.stamp.to_sec(),
        )

    @staticmethod
    def _angle_error(actual, desired):
        return math.atan2(math.sin(actual - desired), math.cos(actual - desired))

    def _terminal_gate(self, desired):
        deadline = time.monotonic() + self.settle_timeout
        window = JointSpanWindow(self.stability_duration)
        previous = None
        last = {}
        while not rospy.is_shutdown() and time.monotonic() <= deadline:
            self._zero_base()
            now = time.monotonic()
            positions, velocities, sample_stamp = self._measurement()
            if positions is None:
                time.sleep(0.05)
                continue
            if (previous is not None
                    and sample_stamp <= previous[0] + 1e-9):
                time.sleep(0.02)
                continue
            if velocities is None and previous is not None:
                elapsed = max(sample_stamp - previous[0], 1e-6)
                velocities = [
                    self._angle_error(value, old) / elapsed
                    for value, old in zip(positions, previous[1])
                ]
            previous = (sample_stamp, list(positions))
            window.add(sample_stamp, positions)
            max_error = max(
                abs(self._angle_error(actual, target))
                for actual, target in zip(positions, desired)
            )
            max_velocity = (
                None if velocities is None else maximum_absolute(velocities)
            )
            max_span = window.maximum_span()
            stable, code, detail = classify_stability(
                max_joint_error=max_error,
                max_joint_velocity=max_velocity,
                max_joint_span=max_span,
                window_ready=window.ready,
                tcp_position_error=None,
                tcp_orientation_error=None,
                joint_error_tolerance=self.max_error,
                joint_velocity_tolerance=self.max_velocity,
                joint_span_tolerance=self.max_span,
                tcp_position_tolerance=0.0,
                tcp_orientation_tolerance=0.0,
                require_tcp=False,
            )
            with self.lock:
                imu = self.imu
            roll_deg = pitch_deg = None
            if imu is not None:
                quaternion = [
                    imu.orientation.x, imu.orientation.y,
                    imu.orientation.z, imu.orientation.w,
                ]
                roll, pitch, _ = tft.euler_from_quaternion(quaternion)
                roll_deg = abs(math.degrees(roll))
                pitch_deg = abs(math.degrees(pitch))
            last = {
                "max_joint_error_rad": max_error,
                "max_joint_velocity_rad_s": max_velocity,
                "max_joint_span_rad": max_span,
                "stability_window_s": window.coverage,
                "roll_deg": roll_deg,
                "pitch_deg": pitch_deg,
                "classification": code,
                "detail": detail,
            }
            attitude_ok = (
                roll_deg is not None and pitch_deg is not None
                and roll_deg <= self.max_roll and pitch_deg <= self.max_pitch
            )
            if stable and attitude_ok:
                return True, last
            if not attitude_ok:
                last["classification"] = "BASE_UNSTABLE"
                last["detail"] = "base roll/pitch exceeds five-degree gate"
            time.sleep(0.05)
        return False, last

    def _execute_target(self, target, repetition):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        goal.trajectory.joint_names = list(self.arm_names)
        point = JointTrajectoryPoint()
        point.positions = list(target["positions"])
        point.velocities = [0.0] * len(self.arm_names)
        point.time_from_start = rospy.Duration(self.motion_duration)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = rospy.Duration(2.0)
        for name in self.arm_names:
            tolerance = JointTolerance()
            tolerance.name = name
            tolerance.position = self.goal_tolerance
            goal.goal_tolerance.append(tolerance)

        self.arm_client.send_goal(goal)
        action_deadline = time.monotonic() + (
            self.motion_duration + 8.0
        ) * self.action_wall_timeout_scale
        terminal_since = None
        terminal_states = {
            GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
            GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST,
        }
        while time.monotonic() < action_deadline:
            self._zero_base()
            state = self.arm_client.get_state()
            if state in terminal_states:
                if (state == GoalStatus.LOST
                        or self.arm_client.get_result() is not None):
                    break
                if terminal_since is None:
                    terminal_since = time.monotonic()
                if time.monotonic() - terminal_since >= self.action_result_grace:
                    break
            time.sleep(0.02)
        else:
            self.arm_client.cancel_goal()
            return {
                "target": target["name"],
                "repetition": repetition,
                "passed": False,
                "classification": "MOTION_TIMEOUT",
                "detail": "arm test motion timed out",
            }

        result = self.arm_client.get_result()
        error_code = None if result is None else int(result.error_code)
        if result is None or error_code not in (
                result.SUCCESSFUL, result.GOAL_TOLERANCE_VIOLATED):
            return {
                "target": target["name"],
                "repetition": repetition,
                "passed": False,
                "fjt_error_code": error_code,
                "classification": "TRUE_POSITION_ERROR",
                "detail": (
                    "controller rejected trajectory: "
                    + ("no result" if result is None else result.error_string)
                ),
            }

        stable, evidence = self._terminal_gate(target["positions"])
        record = {
            "target": target["name"],
            "repetition": repetition,
            "positions": list(target["positions"]),
            "fjt_error_code": error_code,
            "fjt_error_string": "" if result is None else result.error_string,
            "explicit_goal_tolerance_rad": self.goal_tolerance,
            "accepted_goal_tolerance_violation": bool(
                stable and error_code == result.GOAL_TOLERANCE_VIOLATED
            ),
            "passed": stable,
        }
        record.update(evidence)
        return record

    def _wheel_heights(self):
        with self.lock:
            msg = self.links
        result = {}
        if msg is None:
            return result
        for name, pose in zip(msg.name, msg.pose):
            short = name.split("::")[-1]
            if "wheel_link" in short:
                result[short] = float(pose.position.z)
        return result

    def run(self):
        report = {
            "controllers_running": False,
            "motion_completed": False,
            "hold_duration": self.hold_duration,
            "repetitions_per_target": self.repetitions,
            "targets": [item["name"] for item in self.test_targets],
            "terminal_limits": {
                "joint_error_rad": self.max_error,
                "joint_velocity_rad_s": self.max_velocity,
                "joint_span_rad": self.max_span,
                "stability_duration_s": self.stability_duration,
                "settle_timeout_s": self.settle_timeout,
            },
            "trials": [],
            "maximum_joint_error_rad": None,
            "maximum_joint_drift_rad": None,
            "maximum_roll_deg": None,
            "maximum_pitch_deg": None,
            "wheel_lifted": None,
            "passed": False,
            "failure": "",
        }
        try:
            rospy.wait_for_service("/controller_manager/list_controllers", timeout=6.0)
            states = {item.name: item.state for item in self.list_controllers().controller}
            report["controller_states"] = states
            report["controllers_running"] = (
                states.get("ur5_arm_controller") == "running"
                and states.get("gripper_controller") == "running"
            )
            if not report["controllers_running"]:
                raise RuntimeError("UR5 or gripper controller is not running")
            if not self.arm_client.wait_for_server(rospy.Duration(6.0)):
                raise RuntimeError("arm FollowJointTrajectory action unavailable")
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if self._positions() is not None and self.imu is not None:
                    break
                time.sleep(0.05)
            else:
                raise RuntimeError("joint state or IMU unavailable")
            self.wheel_baseline = self._wheel_heights()

            for repetition in range(1, self.repetitions + 1):
                for target in self.test_targets:
                    trial = self._execute_target(target, repetition)
                    report["trials"].append(trial)
                    if not trial["passed"]:
                        raise RuntimeError(
                            "{} repetition {} failed {}: {}".format(
                                target["name"], repetition,
                                trial.get("classification", "UNKNOWN"),
                                trial.get("detail", "terminal gate rejected"),
                            )
                        )
            report["motion_completed"] = True
            start_positions = self._positions()
            if start_positions is None:
                raise RuntimeError("joint state disappeared after motion")

            # The controller-state tracking error during a commanded move is
            # expected to be non-zero.  Reset here so this field measures only
            # terminal hold/sag behavior.
            with self.lock:
                self.max_controller_error = 0.0
            hold_start = time.monotonic()
            while time.monotonic() - hold_start < self.hold_duration:
                self._zero_base()
                time.sleep(0.05)
            final_positions = self._positions()
            if final_positions is None:
                raise RuntimeError("joint state disappeared during hold")
            drift = max(abs(a - b) for a, b in zip(start_positions, final_positions))
            report.update({
                "maximum_joint_error_rad": self.max_controller_error,
                "maximum_joint_drift_rad": drift,
                "maximum_roll_deg": self.max_roll_seen,
                "maximum_pitch_deg": self.max_pitch_seen,
                "maximum_wheel_rise_m": self.max_wheel_rise,
                "wheel_lifted": self.max_wheel_rise > self.wheel_threshold,
                "gripper_position_min_rad": self.gripper_min,
                "gripper_position_max_rad": self.gripper_max,
                "gripper_velocity_peak_rad_s": self.gripper_velocity_peak,
                "gripper_velocity_rms_rad_s": (
                    math.sqrt(
                        self.gripper_velocity_square_sum / self.gripper_samples
                    ) if self.gripper_samples else None
                ),
                "gripper_samples": self.gripper_samples,
            })
            gripper_stable = (
                self.gripper_min is not None
                and self.gripper_max is not None
                and self.gripper_min >= -0.001
                and self.gripper_max <= 0.726
                and self.gripper_velocity_peak <= 0.02
            )
            report["passed"] = (
                report["motion_completed"]
                and all(item["passed"] for item in report["trials"])
                and self.max_controller_error <= self.max_error
                and drift <= self.max_drift
                and self.max_roll_seen <= self.max_roll
                and self.max_pitch_seen <= self.max_pitch
                and not report["wheel_lifted"]
                and gripper_stable
            )
            if not report["passed"]:
                report["failure"] = "hold, sag, attitude, or wheel-lift limit exceeded"
        except Exception as exc:
            report["failure"] = str(exc)
        text = json.dumps(report, ensure_ascii=False, sort_keys=True)
        self.report_pub.publish(String(data=text))
        rospy.loginfo("[ControllerHoldTest] %s", text)
        return report["passed"]


def main():
    rospy.init_node("controller_hold_test")
    test = ControllerHoldTest()
    passed = test.run()
    time.sleep(0.2)
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
