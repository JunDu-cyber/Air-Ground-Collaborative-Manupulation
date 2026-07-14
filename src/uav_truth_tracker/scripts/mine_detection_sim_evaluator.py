#!/usr/bin/env python3
"""Gazebo-only evaluator for the UAV mine map.

ModelStates is used solely to score map-frame localization after inference.  It
is never connected to the detector or fusion inputs, so passing this evaluator
does not leak simulation truth into the operational pipeline.
"""

import json
import math
import os
import re
import threading
from pathlib import Path

import rospy
import yaml
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

from uav_truth_tracker.msg import MineMap


class MineDetectionSimEvaluator:
    def __init__(self):
        self.truth_regex = re.compile(
            rospy.get_param("~truth_model_regex", r"^landmine_[0-9]+$")
        )
        self.map_topic = rospy.get_param("~map_topic", "/mine_detection/map")
        self.status_topic = rospy.get_param(
            "~status_topic", "/mine_detection/sim_evaluation"
        )
        self.max_match_distance = max(
            float(rospy.get_param("~max_match_distance", 0.75)), 0.05
        )
        self.pass_position_error = max(
            float(rospy.get_param("~pass_position_error", 0.35)), 0.01
        )
        self.output_file = Path(
            os.path.abspath(
                os.path.expanduser(
                    rospy.get_param(
                        "~output_file", "~/.ros/mine_detection_evaluation.yaml"
                    )
                )
            )
        )
        # Seed from the simulation layout when available. During a successful
        # disposal mission the mock arm deletes Gazebo models; evaluation must
        # still compare against the original field instead of misclassifying
        # already-cleared detections as false positives.
        self.truth = {}
        layout = rospy.get_param("/mine_field/layout", {})
        for mine in layout.get("mines", []) if isinstance(layout, dict) else []:
            name = str(mine.get("name", ""))
            if self.truth_regex.match(name):
                self.truth[name] = (
                    float(mine.get("x", 0.0)),
                    float(mine.get("y", 0.0)),
                    float(mine.get("z", 0.0)),
                )
        self.mine_map = None
        self.lock = threading.RLock()
        self.last_report_json = ""

        self.publisher = rospy.Publisher(
            self.status_topic, String, queue_size=2, latch=True
        )
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._truth_cb, queue_size=1
        )
        rospy.Subscriber(self.map_topic, MineMap, self._map_cb, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(1.0), self._timer_cb)
        rospy.logwarn(
            "[MineSimEvaluator] Gazebo truth is enabled for scoring only: regex=%s",
            self.truth_regex.pattern,
        )

    def _truth_cb(self, msg):
        truth = {}
        for name, pose in zip(msg.name, msg.pose):
            if self.truth_regex.match(name):
                truth[name] = (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                )
        with self.lock:
            # Union, do not replace: removed models are completed work, not a
            # change to the original scoring field.
            self.truth.update(truth)

    def _map_cb(self, msg):
        with self.lock:
            self.mine_map = msg

    @staticmethod
    def _distance_xy(truth, entry):
        return math.hypot(
            truth[0] - entry.position.x,
            truth[1] - entry.position.y,
        )

    def _report(self):
        confirmed = []
        revision = 0
        frame_id = ""
        if self.mine_map is not None:
            revision = int(self.mine_map.revision)
            frame_id = self.mine_map.header.frame_id
            confirmed = [entry for entry in self.mine_map.mines if entry.confirmed]

        pairs = []
        for truth_name, truth_position in self.truth.items():
            for prediction_index, prediction in enumerate(confirmed):
                distance = self._distance_xy(truth_position, prediction)
                if distance <= self.max_match_distance:
                    pairs.append(
                        (distance, truth_name, prediction_index, truth_position)
                    )
        pairs.sort(key=lambda item: (item[0], item[1], item[2]))
        used_truth = set()
        used_predictions = set()
        matches = []
        for distance, truth_name, prediction_index, truth_position in pairs:
            if truth_name in used_truth or prediction_index in used_predictions:
                continue
            prediction = confirmed[prediction_index]
            used_truth.add(truth_name)
            used_predictions.add(prediction_index)
            matches.append(
                {
                    "truth_name": truth_name,
                    "mine_id": int(prediction.id),
                    "error_xy": float(distance),
                    "truth": {
                        "x": truth_position[0],
                        "y": truth_position[1],
                        "z": truth_position[2],
                    },
                    "prediction": {
                        "x": float(prediction.position.x),
                        "y": float(prediction.position.y),
                        "z": float(prediction.position.z),
                        "confidence": float(prediction.confidence),
                        "observation_count": int(prediction.observation_count),
                    },
                }
            )

        missed = sorted(set(self.truth) - used_truth)
        false_confirmed = [
            {
                "mine_id": int(prediction.id),
                "x": float(prediction.position.x),
                "y": float(prediction.position.y),
                "z": float(prediction.position.z),
                "confidence": float(prediction.confidence),
            }
            for index, prediction in enumerate(confirmed)
            if index not in used_predictions
        ]
        errors = [match["error_xy"] for match in matches]
        all_truth_seen = bool(self.truth) and len(matches) == len(self.truth)
        passed = (
            all_truth_seen
            and not false_confirmed
            and max(errors or [float("inf")]) <= self.pass_position_error
        )
        return {
            "state": "PASS" if passed else "IN_PROGRESS",
            "truth_is_scoring_only": True,
            "frame_id": frame_id,
            "map_revision": revision,
            "truth_count": len(self.truth),
            "confirmed_count": len(confirmed),
            "matched_count": len(matches),
            "missed_truth": missed,
            "false_confirmed": false_confirmed,
            "mean_error_xy": (
                float(sum(errors) / len(errors)) if errors else None
            ),
            "max_error_xy": float(max(errors)) if errors else None,
            "pass_position_error": self.pass_position_error,
            "matches": matches,
        }

    def _write(self, report):
        try:
            self.output_file.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.output_file.with_suffix(
                self.output_file.suffix + ".tmp"
            )
            with temporary.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
            os.replace(str(temporary), str(self.output_file))
        except Exception as exc:
            rospy.logerr_throttle(
                2.0,
                "[MineSimEvaluator] failed to write %s: %s",
                self.output_file,
                exc,
            )

    def _timer_cb(self, _event):
        with self.lock:
            report = self._report()
        report_json = json.dumps(report, sort_keys=True)
        self.publisher.publish(String(data=report_json))
        if report_json != self.last_report_json:
            self._write(report)
            self.last_report_json = report_json
        rospy.loginfo_throttle(
            2.0,
            "[MineSimEvaluator] state=%s truth=%d confirmed=%d matched=%d "
            "missed=%d false=%d mean_err=%s max_err=%s",
            report["state"],
            report["truth_count"],
            report["confirmed_count"],
            report["matched_count"],
            len(report["missed_truth"]),
            len(report["false_confirmed"]),
            "-" if report["mean_error_xy"] is None else "{:.3f}".format(report["mean_error_xy"]),
            "-" if report["max_error_xy"] is None else "{:.3f}".format(report["max_error_xy"]),
        )


def main():
    rospy.init_node("mine_detection_sim_evaluator")
    MineDetectionSimEvaluator()
    rospy.spin()


if __name__ == "__main__":
    main()
