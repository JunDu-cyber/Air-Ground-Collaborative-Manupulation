#!/usr/bin/env python3
"""Repeatable ten-trial manager for the independent phase-1 grasp pipeline.

The randomized Gazebo placement is test-fixture information only.  It is never
sent to the localizer or executor.  The executor receives exactly the wrist
RGB-D target, while Gazebo/link states are used here only to reset and score.
"""

import collections
import csv
import json
import math
import os
import random
import threading
import time

import rospy
import tf.transformations as tft
from gazebo_msgs.msg import LinkStates, ModelState
from gazebo_msgs.srv import GetModelState, SetModelState, SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


CSV_FIELDS = [
    "trial",
    "fixture_spawn_x",
    "fixture_spawn_y",
    "fixture_spawn_yaw_deg",
    "detection_success",
    "target_confidence",
    "target_std_x",
    "target_std_y",
    "target_std_z",
    "ik_success",
    "collision_checked",
    "collision_pairs",
    "target_planar_reach",
    "candidate_yaw_deg",
    "candidate_tilt_deg",
    "grasped",
    "lifted",
    "dropped",
    "moveit_attached",
    "gazebo_attached",
    "gripper_first_contact_command",
    "gripper_first_contact_position",
    "gripper_contact_command",
    "gripper_contact_position",
    "gripper_secure_hold_duration",
    "measured_lift_height",
    "relative_translation_max",
    "relative_rotation_max_deg",
    "max_roll_deg",
    "max_pitch_deg",
    "wheel_lifted",
    "arm_sag_max_rad",
    "total_duration",
    "success",
    "failure_reason",
    "failure_detail",
]


class GraspTestManager:
    def __init__(self):
        self.test_count = int(rospy.get_param("~test_count", 10))
        self.random = random.Random(int(rospy.get_param("~test_random_seed", 41)))
        self.spawn_x = [float(value) for value in
                        rospy.get_param("~test_spawn_x", [0.607, 0.647])]
        self.spawn_y = [float(value) for value in
                        rospy.get_param("~test_spawn_y", [-0.099, -0.059])]
        self.spawn_yaw = [math.radians(float(value)) for value in
                          rospy.get_param("~test_spawn_yaw_deg", [-15.0, 15.0])]
        self.spawn_z = float(rospy.get_param("~test_spawn_z", 0.10))
        self.settle_duration = float(rospy.get_param("~test_settle_duration", 2.0))
        self.model_name = rospy.get_param("~mine_model_name", "landmine_test")
        self.sdf_path = os.path.abspath(os.path.expanduser(
            rospy.get_param("~landmine_sdf_path")
        ))
        self.result_directory = os.path.abspath(os.path.expanduser(
            rospy.get_param("~result_directory", "mine_grasp_results")
        ))
        self.wheel_names = list(rospy.get_param("~wheel_link_names", [
            "front_left_wheel_link", "front_right_wheel_link",
            "rear_left_wheel_link", "rear_right_wheel_link",
        ]))
        self.wheel_lift_threshold = float(
            rospy.get_param("~wheel_lift_threshold", 0.015)
        )
        if self.test_count < 1:
            raise rospy.ROSInitException("test_count must be positive")
        if not os.path.isfile(self.sdf_path):
            raise rospy.ROSInitException("landmine SDF missing: {}".format(self.sdf_path))
        with open(self.sdf_path, "r", encoding="utf-8") as stream:
            self.sdf_xml = stream.read()

        self.lock = threading.RLock()
        self.worker = None
        self.stop_event = threading.Event()
        self.records = []
        self.link_states = None
        self.wheel_baseline = {}
        self.wheel_peak_delta = 0.0
        self.trial_active = False
        self.run_directory = ""

        self.progress_pub = rospy.Publisher(
            "/mine_grasp/test_progress", String, queue_size=10, latch=True
        )
        self.summary_pub = rospy.Publisher(
            "/mine_grasp/test_summary", String, queue_size=1, latch=True
        )
        rospy.Subscriber("/gazebo/link_states", LinkStates,
                         self._link_states_cb, queue_size=1)

        self.execute = rospy.ServiceProxy("/mine_grasp/execute", Trigger)
        self.reset_executor = rospy.ServiceProxy(
            "/mine_grasp/reset_executor", Trigger
        )
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.get_model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState
        )
        self.set_model_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState
        )

        self.start_service = rospy.Service(
            "/start_grasp_test", Trigger, self._start_cb
        )
        self.reset_service = rospy.Service(
            "/reset_grasp_test", Trigger, self._reset_cb
        )
        self._publish_progress({"state": "READY", "planned_trials": self.test_count})

    def _link_states_cb(self, msg):
        with self.lock:
            self.link_states = msg
            if not self.trial_active or not self.wheel_baseline:
                return
            current = self._wheel_heights_locked(msg)
            for name, baseline in self.wheel_baseline.items():
                if name in current:
                    self.wheel_peak_delta = max(
                        self.wheel_peak_delta, current[name] - baseline
                    )

    def _wheel_heights_locked(self, states):
        result = {}
        if states is None:
            return result
        for full_name, pose in zip(states.name, states.pose):
            short = full_name.split("::")[-1]
            if short in self.wheel_names:
                result[short] = float(pose.position.z)
        return result

    def _publish_progress(self, payload):
        payload = dict(payload)
        payload["stamp"] = rospy.Time.now().to_sec()
        text = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.progress_pub.publish(String(data=text))
        rospy.loginfo("[GraspTests] %s", text)

    def _start_cb(self, _request):
        with self.lock:
            if self.worker is not None and self.worker.is_alive():
                return TriggerResponse(False, "a grasp test run is already active")
        physical = bool(rospy.get_param(
            "/mine_grasp_executor/physical_grasp_enabled", False
        ))
        if not physical:
            return TriggerResponse(
                False,
                "executor is in dry-run mode; pass physical_grasp_enabled:=true "
                "only after the dry-run gate passes",
            )
        missing = []
        for service in (
            "/mine_grasp/execute", "/mine_grasp/reset_executor",
            "/gazebo/spawn_sdf_model", "/gazebo/get_model_state",
            "/gazebo/set_model_state",
        ):
            try:
                rospy.wait_for_service(service, timeout=2.0)
            except rospy.ROSException:
                missing.append(service)
        if missing:
            return TriggerResponse(False, "services unavailable: {}".format(", ".join(missing)))
        self.stop_event.clear()
        self.worker = threading.Thread(target=self._run_tests, name="grasp-tests")
        self.worker.daemon = True
        self.worker.start()
        return TriggerResponse(
            True, "started {} sequential physical grasp trials".format(self.test_count)
        )

    def _reset_cb(self, _request):
        self.stop_event.set()
        with self.lock:
            active = self.worker is not None and self.worker.is_alive()
        if active:
            self._publish_progress({"state": "STOP_REQUESTED"})
            return TriggerResponse(
                True, "stop requested; reset will occur after the active trial returns"
            )
        try:
            rospy.wait_for_service("/mine_grasp/reset_executor", timeout=2.0)
            response = self.reset_executor()
            return TriggerResponse(response.success, response.message)
        except Exception as exc:
            return TriggerResponse(False, str(exc))

    def _new_run_directory(self):
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        path = os.path.join(self.result_directory, "run_{}".format(stamp))
        suffix = 1
        candidate = path
        while os.path.exists(candidate):
            candidate = "{}_{}".format(path, suffix)
            suffix += 1
        os.makedirs(candidate, exist_ok=False)
        return candidate

    def _run_tests(self):
        self.records = []
        self.run_directory = self._new_run_directory()
        csv_path = os.path.join(self.run_directory, "trials.csv")
        try:
            with open(csv_path, "w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
                writer.writeheader()
                stream.flush()
                for index in range(1, self.test_count + 1):
                    if self.stop_event.is_set() or rospy.is_shutdown():
                        break
                    self._publish_progress({
                        "state": "RUNNING", "trial": index,
                        "planned_trials": self.test_count,
                    })
                    record = self._run_one(index)
                    if record is None:
                        break
                    self.records.append(record)
                    writer.writerow({key: record.get(key, "") for key in CSV_FIELDS})
                    stream.flush()
                    self._write_json_atomic(
                        os.path.join(self.run_directory,
                                     "trial_{:02d}.json".format(index)),
                        record,
                    )
            if self.stop_event.is_set():
                self._publish_progress({
                    "state": "STOPPED", "completed_trials": len(self.records)
                })
            summary = self._summarize()
            self._write_json_atomic(
                os.path.join(self.run_directory, "summary.json"), summary
            )
            self.summary_pub.publish(String(data=json.dumps(
                summary, ensure_ascii=False, sort_keys=True
            )))
            self._publish_progress({
                "state": "COMPLETE" if len(self.records) == self.test_count else "STOPPED",
                "completed_trials": len(self.records),
                "result_directory": self.run_directory,
            })
        except Exception as exc:
            rospy.logerr("grasp test manager failed: %s", exc)
            self._publish_progress({"state": "FAILED", "detail": str(exc)})
        finally:
            with self.lock:
                self.trial_active = False
            if self.stop_event.is_set():
                try:
                    self.reset_executor()
                except Exception:
                    pass

    def _run_one(self, index):
        record = {key: "" for key in CSV_FIELDS}
        record["trial"] = index
        setup_started = time.monotonic()
        try:
            reset = self.reset_executor()
            if not reset.success:
                raise RuntimeError("executor reset failed: {}".format(reset.message))
            if self.stop_event.is_set():
                return None
            pose, spawn_yaw = self._random_spawn_pose()
            record.update({
                "fixture_spawn_x": float(pose.position.x),
                "fixture_spawn_y": float(pose.position.y),
                "fixture_spawn_yaw_deg": math.degrees(spawn_yaw),
            })
            self._place_fixture(pose)
            self._wall_wait(self.settle_duration)
            if self.stop_event.is_set():
                return None
            with self.lock:
                self.wheel_baseline = self._wheel_heights_locked(self.link_states)
                self.wheel_peak_delta = 0.0
                self.trial_active = True
            grasp_response = self.execute()
            with self.lock:
                self.trial_active = False
                wheel_lifted = self.wheel_peak_delta > self.wheel_lift_threshold
            try:
                report = json.loads(grasp_response.message)
            except (TypeError, ValueError):
                report = {
                    "success": grasp_response.success,
                    "failure_reason": "PREGRASP_FAILED",
                    "failure_detail": grasp_response.message,
                }
            # Preserve the complete executor report in each JSON file.  The
            # CSV remains a compact view selected through CSV_FIELDS.
            fixture = {
                key: record[key] for key in (
                    "trial", "fixture_spawn_x", "fixture_spawn_y",
                    "fixture_spawn_yaw_deg",
                ) if key in record
            }
            record.update(report)
            record.update(fixture)
            record["collision_pairs"] = ", ".join(sorted({
                "{}<->{}".format(item.get("body_1", "?"), item.get("body_2", "?"))
                for item in report.get("collision_contacts", [])
            }))
            record["grasped"] = bool(
                report.get("nonempty_grasp", False)
                and report.get("gazebo_attached", False)
            )
            record["wheel_lifted"] = bool(wheel_lifted)
            record["success"] = bool(
                grasp_response.success and report.get("success", False)
                and not wheel_lifted
            )
            if wheel_lifted:
                record["failure_reason"] = "BASE_UNSTABLE"
                record["failure_detail"] = "wheel height rose more than {:.3f} m".format(
                    self.wheel_lift_threshold
                )
        except Exception as exc:
            with self.lock:
                self.trial_active = False
            record.update({
                "success": False,
                "failure_reason": "PREGRASP_FAILED",
                "failure_detail": "test fixture error: {}".format(exc),
                "total_duration": round(time.monotonic() - setup_started, 3),
                "wheel_lifted": False,
            })
        return record

    def _place_fixture(self, pose):
        """Spawn once, then reposition without deleting plugin-owned objects."""
        current = self.get_model_state(self.model_name, "world")
        if not current.success:
            response = self.spawn_model(
                self.model_name, self.sdf_xml, "", pose, "world"
            )
            if not response.success:
                raise RuntimeError(
                    "spawn failed: {}".format(response.status_message)
                )
            return
        state = ModelState()
        state.model_name = self.model_name
        state.pose = pose
        state.reference_frame = "world"
        response = self.set_model_state(state)
        if not response.success:
            raise RuntimeError(
                "fixture reposition failed: {}".format(response.status_message)
            )

    def _random_spawn_pose(self):
        pose = Pose()
        pose.position.x = self.random.uniform(min(self.spawn_x), max(self.spawn_x))
        pose.position.y = self.random.uniform(min(self.spawn_y), max(self.spawn_y))
        pose.position.z = self.spawn_z
        yaw = self.random.uniform(min(self.spawn_yaw), max(self.spawn_yaw))
        quaternion = tft.quaternion_from_euler(0.0, 0.0, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose, yaw

    def _wall_wait(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.stop_event.is_set():
                return
            time.sleep(min(0.05, deadline - time.monotonic()))

    def _summarize(self):
        count = len(self.records)
        detection = sum(bool(row.get("detection_success")) for row in self.records)
        success = sum(bool(row.get("success")) for row in self.records)
        failures = collections.Counter(
            str(row.get("failure_reason") or "NONE")
            for row in self.records if not row.get("success")
        )
        summary = {
            "planned_trials": self.test_count,
            "completed_trials": count,
            "detection_successes": detection,
            "grasp_lift_successes": success,
            "detection_success_rate": float(detection) / count if count else 0.0,
            "grasp_lift_success_rate": float(success) / count if count else 0.0,
            "detection_target_met": count > 0 and float(detection) / count >= 0.90,
            "grasp_target_met": count > 0 and float(success) / count >= 0.80,
            "failure_counts": dict(sorted(failures.items())),
            "result_directory": self.run_directory,
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime()),
        }
        return summary

    @staticmethod
    def _write_json_atomic(path, data):
        temporary = path + ".writing"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)


def main():
    rospy.init_node("grasp_test_manager")
    GraspTestManager()
    rospy.spin()


if __name__ == "__main__":
    main()
