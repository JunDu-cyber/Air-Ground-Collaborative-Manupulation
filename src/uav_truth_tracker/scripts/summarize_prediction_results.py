#!/usr/bin/env python3
"""Summarize prediction CSV logs into one comparison CSV."""

import argparse
import csv
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import analyze_prediction_log  # noqa: E402


SUMMARY_FIELDS = [
    "file",
    "estimator_model",
    "target_mode",
    "uav1_speed",
    "z_amplitude",
    "capture_success",
    "capture_time",
    "min_distance_3d",
    "mean_distance_3d",
    "mean_prediction_error_3d",
    "p90_prediction_error_3d",
    "max_prediction_error_3d",
    "mean_reachable_margin",
    "percent_reachable_margin_positive",
    "terminal_mean_distance_xy",
    "terminal_mean_distance_z_signed",
    "terminal_mean_abs_z_error",
    "terminal_min_distance_3d",
    "terminal_mean_prediction_error_3d",
    "terminal_mean_reachable_margin",
    "close_xy_below_ratio",
    "close_xy_mean_distance_z_signed",
    "close_xy_mean_abs_z_error",
    "close_xy_max_underpass_z",
    "close_xy_mean_v_cmd_z",
    "close_3d_25_good_window_count",
    "close_3d_35_good_window_count",
    "close_3d_50_good_window_count",
    "close_3d_100_good_window_count",
    "close_3d_35_total_time",
    "close_3d_50_total_time",
    "close_3d_35_first_good_time",
    "mean_detection_error_3d",
    "p90_detection_error_3d",
    "max_detection_error_3d",
    "detection_rate",
    "target_lost_count",
    "target_lost_duration_total",
]


def first_value(rows, key):
    for row in rows:
        value = row.get(key, "")
        if value not in ("", None):
            return value
    return ""


def summarize_file(path, vxy_max, vz_max, terminal_window):
    rows = analyze_prediction_log.load_rows(path)
    summary, _metrics = analyze_prediction_log.compute_metrics(
        rows,
        vxy_max=vxy_max,
        vz_max=vz_max,
        terminal_window=terminal_window,
    )
    row = {field: summary.get(field, "") for field in SUMMARY_FIELDS}
    row["file"] = path
    row["uav1_speed"] = first_value(rows, "uav1_speed")
    row["z_amplitude"] = first_value(rows, "z_amplitude")
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir", nargs="?", default=os.path.expanduser("~/uav_intercept_logs"))
    parser.add_argument("--output", default=None)
    parser.add_argument("--vxy-max", type=float, default=1.8)
    parser.add_argument("--vz-max", type=float, default=0.8)
    parser.add_argument("--terminal-window", type=float, default=3.0)
    args = parser.parse_args()

    log_dir = os.path.expanduser(args.log_dir)
    output = args.output or os.path.join(log_dir, "prediction_model_summary.csv")
    csv_files = []
    for root, _dirs, files in os.walk(log_dir):
        for name in files:
            if not name.endswith(".csv"):
                continue
            if name == os.path.basename(output) or name.startswith("prediction_model_summary"):
                continue
            if name.endswith("_summary.csv"):
                continue
            else:
                csv_files.append(os.path.join(root, name))
    csv_files.sort()

    rows = []
    for path in csv_files:
        try:
            rows.append(
                summarize_file(path, args.vxy_max, args.vz_max, args.terminal_window)
            )
        except Exception as exc:
            print("Skipping {}: {}".format(path, exc), file=sys.stderr)

    os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
    with open(output, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    print("Wrote {} rows to {}".format(len(rows), output))


if __name__ == "__main__":
    main()
