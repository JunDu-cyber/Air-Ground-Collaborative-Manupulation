#!/usr/bin/env python3
"""Analyze target-estimator chase CSV logs."""

import argparse
import csv
import math
import os
import statistics
from collections import defaultdict


def to_float(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def finite(value):
    try:
        return math.isfinite(value)
    except (TypeError, ValueError):
        return False


def mean(values):
    return statistics.mean(values) if values else float("nan")


def percentile(values, q):
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * q))
    return ordered[min(max(idx, 0), len(ordered) - 1)]


def stdev(values):
    return statistics.pstdev(values) if len(values) >= 2 else float("nan")


def load_rows(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def interpolate(times, xs, ys, zs, t):
    if not times or t < times[0] or t > times[-1]:
        return None
    lo = 0
    hi = len(times) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if times[mid] < t:
            lo = mid + 1
        else:
            hi = mid - 1
    idx = lo
    if idx <= 0:
        return xs[0], ys[0], zs[0]
    if idx >= len(times):
        return xs[-1], ys[-1], zs[-1]
    t0 = times[idx - 1]
    t1 = times[idx]
    if t1 <= t0:
        return xs[idx], ys[idx], zs[idx]
    alpha = (t - t0) / (t1 - t0)
    return (
        xs[idx - 1] + alpha * (xs[idx] - xs[idx - 1]),
        ys[idx - 1] + alpha * (ys[idx] - ys[idx - 1]),
        zs[idx - 1] + alpha * (zs[idx] - zs[idx - 1]),
    )


def row_distance_xy(row):
    value = to_float(row, "distance_xy")
    if finite(value):
        return value
    dx = to_float(row, "uav1_x") - to_float(row, "uav0_x")
    dy = to_float(row, "uav1_y") - to_float(row, "uav0_y")
    return math.sqrt(dx * dx + dy * dy) if finite(dx) and finite(dy) else float("nan")


def row_distance_3d(row):
    value = to_float(row, "distance_3d")
    if finite(value):
        return value
    dx = to_float(row, "uav1_x") - to_float(row, "uav0_x")
    dy = to_float(row, "uav1_y") - to_float(row, "uav0_y")
    dz = to_float(row, "uav1_z") - to_float(row, "uav0_z")
    if not all(finite(v) for v in (dx, dy, dz)):
        return float("nan")
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def row_signed_z_error(row):
    dz = to_float(row, "uav1_z") - to_float(row, "uav0_z")
    return dz if finite(dz) else float("nan")


def row_cmd_z(row):
    for key in ("v_cmd_z", "cmd_vz"):
        value = to_float(row, key)
        if finite(value):
            return value
    return float("nan")


def capture_success_value(row):
    return str(row.get("capture_success", "0")) in ("1", "True", "true")


def bool_value(row, key):
    return str(row.get(key, "0")).lower() in ("1", "true", "yes")


def detection_metrics(rows, valid_age_threshold=0.5):
    if not rows or "detection_valid" not in rows[0]:
        return {}, {}

    errors_3d = []
    errors_xy = []
    errors_z = []
    confidences = []
    point_counts = []
    detection_series = []
    lost_count = 0
    lost_duration = 0.0
    longest_gap = 0.0
    current_gap = 0.0
    predict_only_duration = 0.0
    previous_lost = False
    previous_time = None

    for row in rows:
        t = to_float(row, "time")
        age = to_float(row, "detection_age")
        valid = bool_value(row, "detection_valid")
        fresh = valid and (not finite(age) or age <= valid_age_threshold)
        detection_series.append((t, fresh))
        confidence = to_float(row, "detection_confidence")
        if finite(confidence):
            confidences.append(confidence)
        point_count = to_float(row, "detection_point_count")
        if finite(point_count):
            point_counts.append(point_count)
        if fresh:
            e3 = to_float(row, "detection_error_3d")
            exy = to_float(row, "detection_error_xy")
            ez = to_float(row, "detection_error_z")
            if finite(e3):
                errors_3d.append(e3)
            if finite(exy):
                errors_xy.append(exy)
            if finite(ez):
                errors_z.append(abs(ez))

        lost = not fresh
        dt = max(t - previous_time, 0.0) if finite(t) and finite(previous_time) else 0.0
        if lost and not previous_lost:
            lost_count += 1
        if lost:
            lost_duration += dt
            current_gap += dt
            longest_gap = max(longest_gap, current_gap)
        else:
            current_gap = 0.0
        if bool_value(row, "predict_only"):
            predict_only_duration += dt
        previous_lost = lost
        previous_time = t if finite(t) else previous_time

    valid_samples = sum(1 for _, fresh in detection_series if fresh)
    total_time = 0.0
    times = [t for t, _fresh in detection_series if finite(t)]
    if len(times) >= 2:
        total_time = max(times[-1] - times[0], 0.0)
    summary = {
        "mean_detection_error_3d": mean(errors_3d),
        "p90_detection_error_3d": percentile(errors_3d, 0.9),
        "max_detection_error_3d": max(errors_3d) if errors_3d else float("nan"),
        "mean_detection_error_xy": mean(errors_xy),
        "mean_detection_error_z": mean(errors_z),
        "detection_rate": (
            100.0 * valid_samples / len(detection_series)
            if detection_series
            else float("nan")
        ),
        "target_lost_count": lost_count,
        "target_lost_duration_total": lost_duration,
        "total_lost_duration": lost_duration,
        "longest_detection_gap": longest_gap,
        "mean_detection_confidence": mean(confidences),
        "mean_detection_point_count": mean(point_counts),
        "min_detection_point_count": min(point_counts) if point_counts else float("nan"),
        "predict_only_duration_total": predict_only_duration,
        "predict_only_ratio": (
            100.0 * predict_only_duration / total_time if total_time > 1e-6 else float("nan")
        ),
    }
    metrics = {
        "detection_time": [
            to_float(row, "time") for row in rows if finite(to_float(row, "time"))
        ],
        "detection_error_3d": [
            to_float(row, "detection_error_3d") for row in rows
            if finite(to_float(row, "time"))
        ],
        "detection_valid": detection_series,
    }
    return summary, metrics


def fusion_observation_metrics(rows):
    if not rows or "target_fused_valid" not in rows[0]:
        return {}, {}

    times = [to_float(row, "time") for row in rows if finite(to_float(row, "time"))]
    total_time = max(times[-1] - times[0], 0.0) if len(times) >= 2 else 0.0
    previous_time = None
    visual_lost_duration = 0.0
    lidar_fallback_duration = 0.0
    full_lost_duration = 0.0
    source_counts = defaultdict(int)

    def valid_rate(key):
        samples = [bool_value(row, key) for row in rows if key in row]
        return 100.0 * sum(1 for value in samples if value) / len(samples) if samples else float("nan")

    def mean_error(key, valid_key):
        values = [
            to_float(row, key)
            for row in rows
            if bool_value(row, valid_key) and finite(to_float(row, key))
        ]
        return mean(values)

    for row in rows:
        t = to_float(row, "time")
        dt = max(t - previous_time, 0.0) if finite(t) and finite(previous_time) else 0.0
        previous_time = t if finite(t) else previous_time
        source = str(row.get("target_fused_source", "none")).lower() or "none"
        if not bool_value(row, "target_fused_valid"):
            source = "none"
        source_counts[source] += 1
        if not bool_value(row, "target_visual_valid"):
            visual_lost_duration += dt
        if source == "lidar":
            lidar_fallback_duration += dt
        if source == "none":
            full_lost_duration += dt

    total_samples = sum(source_counts.values())
    def source_ratio(source):
        return 100.0 * source_counts[source] / total_samples if total_samples else float("nan")

    summary = {
        "visual_detection_rate": valid_rate("target_visual_valid"),
        "lidar_detection_rate": valid_rate("target_lidar_valid"),
        "fused_detection_rate": valid_rate("target_fused_valid"),
        "visual_mean_error_3d": mean_error(
            "target_visual_error_3d", "target_visual_valid"
        ),
        "lidar_mean_error_3d": mean_error(
            "target_lidar_error_3d", "target_lidar_valid"
        ),
        "fused_mean_error_3d": mean_error(
            "target_fused_error_3d", "target_fused_valid"
        ),
        "target_estimator_mean_error_3d": mean(
            [
                to_float(row, "target_estimator_error_3d")
                for row in rows
                if finite(to_float(row, "target_estimator_error_3d"))
            ]
        ),
        "source_ratio_visual": source_ratio("visual"),
        "source_ratio_lidar": source_ratio("lidar"),
        "source_ratio_fused": source_ratio("fused"),
        "source_ratio_none": source_ratio("none"),
        "visual_lost_duration_total": visual_lost_duration,
        "lidar_fallback_duration_total": lidar_fallback_duration,
        "full_lost_duration_total": full_lost_duration,
        "visual_lost_count": max(
            [
                int(to_float(row, "visual_lost_count"))
                for row in rows
                if finite(to_float(row, "visual_lost_count"))
            ]
            or [0]
        ),
        "lidar_fallback_count": max(
            [
                int(to_float(row, "lidar_fallback_count"))
                for row in rows
                if finite(to_float(row, "lidar_fallback_count"))
            ]
            or [0]
        ),
        "fused_none_duration": max(
            [
                to_float(row, "fused_none_duration")
                for row in rows
                if finite(to_float(row, "fused_none_duration"))
            ]
            or [0.0]
        ),
    }
    metrics = {
        "fusion_time": [to_float(row, "time") for row in rows],
        "visual_valid": [bool_value(row, "target_visual_valid") for row in rows],
        "lidar_valid": [bool_value(row, "target_lidar_valid") for row in rows],
        "fused_valid": [bool_value(row, "target_fused_valid") for row in rows],
        "fused_source": [
            str(row.get("target_fused_source", "none")).lower() or "none"
            for row in rows
        ],
    }
    if total_time <= 1e-6:
        summary["visual_lost_duration_total"] = float("nan")
        summary["lidar_fallback_duration_total"] = float("nan")
        summary["full_lost_duration_total"] = float("nan")
    return summary, metrics


def association_metrics(rows):
    if not rows or "candidate_count" not in rows[0]:
        return {}

    candidate_counts = []
    range_valid_counts = []
    association_margins = []
    selected_scores = []
    selected_distance_to_gate = []
    candidate_nonzero_no_detection = 0
    gate_reject_suspected = 0
    wrong_cluster_suspected = 0
    low_margin_count = 0
    detection_by_state = defaultdict(lambda: [0, 0])
    errors_by_state = defaultdict(list)
    lost_duration = 0.0
    longest_lost = 0.0
    current_lost = 0.0
    previous_time = None
    max_reacquire_count = 0

    for row in rows:
        t = to_float(row, "time")
        dt = max(t - previous_time, 0.0) if finite(t) and finite(previous_time) else 0.0
        previous_time = t if finite(t) else previous_time

        state = str(row.get("detector_state", "")).upper() or "UNKNOWN"
        valid = bool_value(row, "detection_valid")
        candidate_count = to_float(row, "candidate_count")
        if finite(candidate_count):
            candidate_counts.append(candidate_count)
        range_valid = to_float(row, "range_valid_cluster_count")
        if finite(range_valid):
            range_valid_counts.append(range_valid)
        margin = to_float(row, "association_margin")
        if finite(margin):
            association_margins.append(margin)
            if margin < 0.25:
                low_margin_count += 1
        selected_score = to_float(row, "selected_cluster_score")
        if finite(selected_score):
            selected_scores.append(selected_score)
        gate_distance = to_float(row, "selected_distance_to_gate")
        if finite(gate_distance):
            selected_distance_to_gate.append(gate_distance)

        if finite(candidate_count) and candidate_count > 0 and not valid:
            candidate_nonzero_no_detection += 1
            if bool_value(row, "use_estimator_gating") or state in ("TRACK", "COAST", "ACQUIRE"):
                gate_reject_suspected += 1

        detection_by_state[state][1] += 1
        if valid:
            detection_by_state[state][0] += 1
            err = to_float(row, "detection_error_3d")
            if finite(err):
                errors_by_state[state].append(err)
                if err > 1.0 and finite(gate_distance) and gate_distance < 0.5:
                    wrong_cluster_suspected += 1

        if state == "LOST":
            lost_duration += dt
            current_lost += dt
            longest_lost = max(longest_lost, current_lost)
        else:
            current_lost = 0.0

        reacquire_count = to_float(row, "reacquire_count")
        if finite(reacquire_count):
            max_reacquire_count = max(max_reacquire_count, int(reacquire_count))

    detection_rate_by_state = []
    for state, (valid_count, total_count) in sorted(detection_by_state.items()):
        rate = 100.0 * valid_count / total_count if total_count else float("nan")
        detection_rate_by_state.append("{}:{:.1f}%".format(state, rate))
    detection_error_by_state = []
    for state, values in sorted(errors_by_state.items()):
        detection_error_by_state.append("{}:{:.2f}m".format(state, mean(values)))

    return {
        "mean_candidate_count": mean(candidate_counts),
        "mean_range_valid_cluster_count": mean(range_valid_counts),
        "candidate_nonzero_but_no_detection_count": candidate_nonzero_no_detection,
        "gate_reject_suspected_count": gate_reject_suspected,
        "mean_association_margin": mean(association_margins),
        "low_association_margin_count": low_margin_count,
        "mean_selected_cluster_score": mean(selected_scores),
        "mean_selected_distance_to_gate": mean(selected_distance_to_gate),
        "reacquire_count_total": max_reacquire_count,
        "lost_duration_total": lost_duration,
        "longest_lost_duration": longest_lost,
        "detection_rate_by_state": ", ".join(detection_rate_by_state),
        "detection_error_by_state": ", ".join(detection_error_by_state),
        "selected_wrong_cluster_suspected_count": wrong_cluster_suspected,
    }


def close_3d_window_metrics(rows, threshold, min_duration=0.15):
    windows = []
    start = None
    last_time = None
    min_distance = float("nan")

    for row in rows:
        t = to_float(row, "time")
        distance = row_distance_3d(row)
        if not finite(t) or not finite(distance):
            continue
        inside = distance <= threshold
        if inside and start is None:
            start = t
            min_distance = distance
        if inside:
            min_distance = min(min_distance, distance)
        if not inside and start is not None:
            windows.append((start, last_time, max(last_time - start, 0.0), min_distance))
            start = None
            min_distance = float("nan")
        last_time = t

    if start is not None and last_time is not None:
        windows.append((start, last_time, max(last_time - start, 0.0), min_distance))

    good_windows = [window for window in windows if window[2] >= min_duration]
    durations = [window[2] for window in windows]
    good_durations = [window[2] for window in good_windows]
    return {
        "count": len(windows),
        "good_count": len(good_windows),
        "total_time": sum(durations) if durations else 0.0,
        "good_total_time": sum(good_durations) if good_durations else 0.0,
        "first_good_time": good_windows[0][0] if good_windows else float("nan"),
        "best_min_distance": (
            min(window[3] for window in windows) if windows else float("nan")
        ),
    }


def compute_metrics(rows, vxy_max=1.8, vz_max=0.8, terminal_window=3.0):
    times = [to_float(r, "time") for r in rows]
    uav1_x = [to_float(r, "uav1_x") for r in rows]
    uav1_y = [to_float(r, "uav1_y") for r in rows]
    uav1_z = [to_float(r, "uav1_z") for r in rows]

    per_time = []
    e3d = []
    exy = []
    ez = []
    signed_pred_z_error = []
    reachable_margin = []

    for row in rows:
        t = to_float(row, "time")
        t_go = to_float(row, "t_go")
        pred = (to_float(row, "pred_x"), to_float(row, "pred_y"), to_float(row, "pred_z"))
        uav0 = (to_float(row, "uav0_x"), to_float(row, "uav0_y"), to_float(row, "uav0_z"))
        if not all(finite(v) for v in (t, t_go) + pred + uav0):
            continue
        future = interpolate(times, uav1_x, uav1_y, uav1_z, t + t_go)
        if future is None:
            continue
        dx = pred[0] - future[0]
        dy = pred[1] - future[1]
        dz = pred[2] - future[2]
        pred_error_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
        pred_error_xy = math.sqrt(dx * dx + dy * dy)
        pred_error_z = abs(dz)

        t_xy_required = math.sqrt(
            (pred[0] - uav0[0]) ** 2 + (pred[1] - uav0[1]) ** 2
        ) / max(vxy_max, 1e-6)
        t_z_required = abs(pred[2] - uav0[2]) / max(vz_max, 1e-6)
        margin = t_go - max(t_xy_required, t_z_required)

        e3d.append(pred_error_3d)
        exy.append(pred_error_xy)
        ez.append(pred_error_z)
        signed_pred_z_error.append(dz)
        reachable_margin.append(margin)
        per_time.append(
            {
                "time": t,
                "prediction_error_3d": pred_error_3d,
                "prediction_error_xy": pred_error_xy,
                "prediction_error_z": pred_error_z,
                "prediction_error_z_signed": dz,
                "reachable_margin": margin,
            }
        )

    capture_rows = [r for r in rows if capture_success_value(r)]
    capture_success = bool(capture_rows)
    capture_time = to_float(capture_rows[0], "time") if capture_rows else float("nan")
    last_time = times[-1] if times else float("nan")
    terminal_end = capture_time if capture_success else last_time
    terminal_start = terminal_end - terminal_window if finite(terminal_end) else float("nan")
    terminal_rows = [
        row
        for row in rows
        if finite(to_float(row, "time"))
        and finite(terminal_start)
        and terminal_start <= to_float(row, "time") <= terminal_end
    ]
    terminal_times = set(to_float(row, "time") for row in terminal_rows)
    terminal_metrics = [m for m in per_time if m["time"] in terminal_times]

    distances = [row_distance_3d(r) for r in rows if finite(row_distance_3d(r))]
    distance_xy_values = [row_distance_xy(r) for r in rows if finite(row_distance_xy(r))]
    terminal_distance_xy = [
        row_distance_xy(r) for r in terminal_rows if finite(row_distance_xy(r))
    ]
    terminal_distance_3d = [
        row_distance_3d(r) for r in terminal_rows if finite(row_distance_3d(r))
    ]
    terminal_z_signed = [
        row_signed_z_error(r) for r in terminal_rows if finite(row_signed_z_error(r))
    ]
    terminal_cmd_z = [row_cmd_z(r) for r in terminal_rows if finite(row_cmd_z(r))]
    close_xy_threshold = 0.5
    close_rows = [
        r
        for r in terminal_rows
        if finite(row_distance_xy(r)) and row_distance_xy(r) <= close_xy_threshold
    ]
    close_z_signed = [
        row_signed_z_error(r) for r in close_rows if finite(row_signed_z_error(r))
    ]
    close_cmd_z = [row_cmd_z(r) for r in close_rows if finite(row_cmd_z(r))]
    close_3d_25 = close_3d_window_metrics(rows, 0.25)
    close_3d_35 = close_3d_window_metrics(rows, 0.35)
    close_3d_50 = close_3d_window_metrics(rows, 0.50)
    close_3d_100 = close_3d_window_metrics(rows, 1.00)

    summary = {
        "estimator_model": rows[0].get("estimator_model", "") if rows else "",
        "target_mode": rows[0].get("target_mode", "") if rows else "",
        "capture_success": capture_success,
        "capture_time": capture_time,
        "min_distance_3d": min(distances) if distances else float("nan"),
        "mean_distance_3d": mean(distances),
        "min_distance_xy": min(distance_xy_values) if distance_xy_values else float("nan"),
        "mean_prediction_error_3d": mean(e3d),
        "p90_prediction_error_3d": percentile(e3d, 0.9),
        "max_prediction_error_3d": max(e3d) if e3d else float("nan"),
        "mean_prediction_error_xy": mean(exy),
        "mean_prediction_error_z": mean(ez),
        "std_prediction_error_3d": stdev(e3d),
        "mean_reachable_margin": mean(reachable_margin),
        "percent_reachable_margin_positive": (
            100.0
            * sum(1 for v in reachable_margin if v >= 0.0)
            / len(reachable_margin)
            if reachable_margin
            else float("nan")
        ),
        "terminal_window": terminal_window,
        "terminal_start_time": terminal_start,
        "terminal_end_time": terminal_end,
        "terminal_mean_distance_xy": mean(terminal_distance_xy),
        "terminal_min_distance_xy": (
            min(terminal_distance_xy) if terminal_distance_xy else float("nan")
        ),
        "terminal_mean_distance_z_signed": mean(terminal_z_signed),
        "terminal_mean_abs_z_error": mean([abs(v) for v in terminal_z_signed]),
        "terminal_min_distance_3d": (
            min(terminal_distance_3d) if terminal_distance_3d else float("nan")
        ),
        "terminal_mean_prediction_error_3d": mean(
            [m["prediction_error_3d"] for m in terminal_metrics]
        ),
        "terminal_mean_prediction_error_xy": mean(
            [m["prediction_error_xy"] for m in terminal_metrics]
        ),
        "terminal_mean_prediction_error_z": mean(
            [m["prediction_error_z"] for m in terminal_metrics]
        ),
        "terminal_mean_reachable_margin": mean(
            [m["reachable_margin"] for m in terminal_metrics]
        ),
        "terminal_percent_reachable_margin_positive": (
            100.0
            * sum(1 for m in terminal_metrics if m["reachable_margin"] >= 0.0)
            / len(terminal_metrics)
            if terminal_metrics
            else float("nan")
        ),
        "terminal_mean_v_cmd_z": mean(terminal_cmd_z),
        "terminal_max_abs_v_cmd_z": (
            max(abs(v) for v in terminal_cmd_z) if terminal_cmd_z else float("nan")
        ),
        "close_xy_threshold": close_xy_threshold,
        "close_xy_sample_count": len(close_rows),
        "close_xy_below_ratio": (
            100.0 * sum(1 for v in close_z_signed if v > 0.0) / len(close_z_signed)
            if close_z_signed
            else float("nan")
        ),
        "close_xy_mean_distance_z_signed": mean(close_z_signed),
        "close_xy_mean_abs_z_error": mean([abs(v) for v in close_z_signed]),
        "close_xy_max_underpass_z": (
            max(close_z_signed) if close_z_signed else float("nan")
        ),
        "close_xy_mean_v_cmd_z": mean(close_cmd_z),
        "close_3d_25_window_count": close_3d_25["count"],
        "close_3d_25_good_window_count": close_3d_25["good_count"],
        "close_3d_25_total_time": close_3d_25["total_time"],
        "close_3d_25_first_good_time": close_3d_25["first_good_time"],
        "close_3d_35_window_count": close_3d_35["count"],
        "close_3d_35_good_window_count": close_3d_35["good_count"],
        "close_3d_35_total_time": close_3d_35["total_time"],
        "close_3d_35_first_good_time": close_3d_35["first_good_time"],
        "close_3d_50_window_count": close_3d_50["count"],
        "close_3d_50_good_window_count": close_3d_50["good_count"],
        "close_3d_50_total_time": close_3d_50["total_time"],
        "close_3d_50_first_good_time": close_3d_50["first_good_time"],
        "close_3d_100_window_count": close_3d_100["count"],
        "close_3d_100_good_window_count": close_3d_100["good_count"],
        "close_3d_100_total_time": close_3d_100["total_time"],
        "close_3d_100_first_good_time": close_3d_100["first_good_time"],
    }

    det_summary, det_metrics = detection_metrics(rows)
    summary.update(det_summary)
    fusion_summary, fusion_metrics = fusion_observation_metrics(rows)
    summary.update(fusion_summary)
    summary.update(association_metrics(rows))

    optional_yaw = terminal_optional_yaw_metrics(terminal_rows)
    summary.update(optional_yaw)
    summary["diagnosis"] = build_diagnosis(summary)

    metrics = {
        "per_time": per_time,
        "error_time": [m["time"] for m in per_time],
        "e3d": e3d,
        "exy": exy,
        "ez": ez,
        "signed_ez": signed_pred_z_error,
        "reachable_margin": reachable_margin,
        "terminal_rows": terminal_rows,
        "detection": det_metrics,
        "fusion": fusion_metrics,
    }
    return summary, metrics


def terminal_optional_yaw_metrics(terminal_rows):
    yaw_error_keys = ("yaw_error", "yaw_err")
    yaw_aligned_keys = ("yaw_aligned",)
    yaw_gating_keys = ("yaw_gating", "yaw_blocked", "yaw_gate_active")
    out = {}

    yaw_errors = []
    for row in terminal_rows:
        for key in yaw_error_keys:
            value = to_float(row, key)
            if finite(value):
                yaw_errors.append(abs(value))
                break
    if yaw_errors:
        out["terminal_mean_yaw_error"] = mean(yaw_errors)

    def bool_ratio(keys):
        values = []
        for row in terminal_rows:
            for key in keys:
                if key in row:
                    values.append(str(row.get(key, "")).lower() in ("1", "true", "yes"))
                    break
        return 100.0 * sum(1 for v in values if v) / len(values) if values else None

    gating_ratio = bool_ratio(yaw_gating_keys)
    aligned_ratio = bool_ratio(yaw_aligned_keys)
    if gating_ratio is not None:
        out["terminal_yaw_gating_ratio"] = gating_ratio
    if aligned_ratio is not None:
        out["terminal_yaw_aligned_ratio"] = aligned_ratio
    return out


def build_diagnosis(summary):
    notes = []
    terminal_prediction = summary.get("terminal_mean_prediction_error_3d", float("nan"))
    terminal_margin = summary.get("terminal_mean_reachable_margin", float("nan"))
    abs_z = summary.get("terminal_mean_abs_z_error", float("nan"))
    cmd_z = summary.get("terminal_mean_v_cmd_z", float("nan"))
    distance_xy = summary.get("terminal_mean_distance_xy", float("nan"))
    min_d3 = summary.get("terminal_min_distance_3d", float("nan"))
    close_below_ratio = summary.get("close_xy_below_ratio", float("nan"))
    close_underpass_z = summary.get("close_xy_max_underpass_z", float("nan"))
    detection_rate = summary.get("detection_rate", float("nan"))
    mean_det_conf = summary.get("mean_detection_confidence", float("nan"))
    mean_points = summary.get("mean_detection_point_count", float("nan"))
    predict_only_ratio = summary.get("predict_only_ratio", float("nan"))
    lost_duration = summary.get("target_lost_duration_total", float("nan"))
    candidate_no_detection = summary.get(
        "candidate_nonzero_but_no_detection_count", float("nan")
    )
    gate_reject = summary.get("gate_reject_suspected_count", float("nan"))
    mean_margin = summary.get("mean_association_margin", float("nan"))
    wrong_cluster = summary.get("selected_wrong_cluster_suspected_count", float("nan"))
    longest_lost = summary.get("longest_lost_duration", float("nan"))

    if finite(detection_rate) and detection_rate < 70.0:
        notes.append("LiDAR detector is the bottleneck. Improve accumulation/gating/reflector.")
    if finite(detection_rate) and detection_rate >= 70.0 and finite(terminal_prediction) and terminal_prediction > 1.0:
        notes.append("Estimator tuning or measurement covariance may be limiting.")
    if finite(mean_points) and mean_points < 5.0:
        notes.append("LiDAR target observability is poor. Increase samples/accumulation before tuning Kalman.")
    if finite(predict_only_ratio) and predict_only_ratio > 10.0 and summary.get("capture_success"):
        notes.append("Estimator predict-only mode is working.")
    if finite(lost_duration) and lost_duration > 1.0:
        notes.append("Detector reacquisition/search should be improved.")
    if finite(candidate_no_detection) and candidate_no_detection > 10:
        notes.append("Association/gating/filtering is too strict.")
    if finite(gate_reject) and gate_reject > 5:
        notes.append("Estimator gating may be rejecting available clusters.")
    if finite(wrong_cluster) and wrong_cluster > 0:
        notes.append("Estimator gating may have drifted and locked onto a wrong cluster.")
    if finite(mean_margin) and mean_margin < 0.25:
        notes.append("Multiple clusters are ambiguous; consider top-K hypothesis tracking.")
    if finite(longest_lost) and longest_lost > 1.0:
        notes.append("LOST state persists too long; reacquisition scoring should be checked.")
    if finite(mean_det_conf) and mean_det_conf < 0.35:
        notes.append("Detection confidence is low; estimator should treat measurements conservatively.")
    if finite(terminal_prediction) and terminal_prediction > 1.0:
        notes.append("Terminal bias likely comes from prediction error.")
    if finite(terminal_margin) and terminal_margin < 0.0:
        notes.append(
            "Terminal bias likely comes from unreachable or too aggressive intercept point."
        )
    if (
        finite(abs_z)
        and abs_z > 0.25
        and finite(cmd_z)
        and abs(cmd_z) < 0.05
    ):
        notes.append("Vertical correction may be gated or too weak.")
    if finite(distance_xy) and distance_xy < 0.5 and finite(abs_z) and abs_z > 0.25:
        notes.append("Horizontal interception is good, vertical tracking is the limiting factor.")
    if (
        finite(terminal_prediction)
        and terminal_prediction < 0.5
        and finite(min_d3)
        and min_d3 > 0.5
    ):
        notes.append("Guidance/control terminal behavior may be limiting capture.")
    if (
        finite(close_below_ratio)
        and close_below_ratio > 70.0
        and finite(close_underpass_z)
        and close_underpass_z > 0.15
    ):
        notes.append(
            "Close-pass data shows UAV0 often passes below UAV1; increase vertical correction or acceleration limit."
        )
    if not notes:
        notes.append("No dominant terminal-bias source detected from these thresholds.")
    return " ".join(notes)


def plot_outputs(out_dir, times, metrics, rows, terminal_window):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except Exception as exc:
        print("matplotlib unavailable, skipping plots: {}".format(exc))
        return

    os.makedirs(out_dir, exist_ok=True)

    def save_line(filename, x, series, title, ylabel):
        plt.figure()
        for label, values in series:
            plt.plot(x, values, label=label)
        plt.title(title)
        plt.xlabel("time [s]")
        plt.ylabel(ylabel)
        if len(series) > 1:
            plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, filename))
        plt.close()

    save_line(
        "distance_3d_vs_time.png",
        times,
        [("distance_3d", [row_distance_3d(r) for r in rows])],
        "Distance 3D vs Time",
        "distance [m]",
    )
    save_line(
        "prediction_error_3d_vs_time.png",
        metrics["error_time"],
        [("prediction_error_3d", metrics["e3d"])],
        "Prediction Error 3D vs Time",
        "error [m]",
    )
    save_line(
        "prediction_error_xy_z_vs_time.png",
        metrics["error_time"],
        [("xy", metrics["exy"]), ("z", metrics["ez"])],
        "Prediction Error XY/Z vs Time",
        "error [m]",
    )
    save_line(
        "t_go_vs_time.png",
        times,
        [("t_go", [to_float(r, "t_go") for r in rows])],
        "Time-to-go vs Time",
        "time [s]",
    )
    save_line(
        "z_error_vs_time.png",
        times,
        [("signed_z_error", [row_signed_z_error(r) for r in rows])],
        "Signed Z Error vs Time",
        "uav1_z - uav0_z [m]",
    )
    save_line(
        "reachable_margin_vs_time.png",
        metrics["error_time"],
        [("reachable_margin", metrics["reachable_margin"])],
        "Reachable Margin vs Time",
        "margin [s]",
    )

    terminal_rows = metrics["terminal_rows"]
    save_line(
        "terminal_distance_zoom.png",
        [to_float(r, "time") for r in terminal_rows],
        [
            ("distance_xy", [row_distance_xy(r) for r in terminal_rows]),
            ("abs_z_error", [abs(row_signed_z_error(r)) for r in terminal_rows]),
            ("distance_3d", [row_distance_3d(r) for r in terminal_rows]),
        ],
        "Terminal Distance Zoom ({:.1f}s)".format(terminal_window),
        "distance [m]",
    )

    plt.figure()
    plt.scatter(
        [row_signed_z_error(r) for r in rows],
        [row_cmd_z(r) for r in rows],
        s=8,
        alpha=0.5,
    )
    plt.title("Vertical Command vs Signed Z Error")
    plt.xlabel("signed_z_error = uav1_z - uav0_z [m]")
    plt.ylabel("v_cmd_z [m/s]")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "v_cmd_z_vs_z_error.png"))
    plt.close()

    plt.figure()
    plt.hist(metrics["e3d"], bins=40)
    plt.title("Prediction Error 3D Histogram")
    plt.xlabel("prediction_error_3d [m]")
    plt.ylabel("count")
    if metrics["e3d"]:
        text = "mean={:.2f}\np90={:.2f}\nmax={:.2f}".format(
            mean(metrics["e3d"]),
            percentile(metrics["e3d"], 0.9),
            max(metrics["e3d"]),
        )
        plt.gca().text(
            0.98,
            0.95,
            text,
            transform=plt.gca().transAxes,
            ha="right",
            va="top",
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8},
        )
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "prediction_error_histogram.png"))
    plt.close()

    detection = metrics.get("detection", {})
    det_errors = [
        value
        for value in detection.get("detection_error_3d", [])
        if finite(value)
    ]
    det_times = [
        to_float(row, "time")
        for row in rows
        if finite(to_float(row, "time"))
    ]
    has_detection_fields = bool(rows) and "detection_valid" in rows[0]
    if has_detection_fields:
        save_line(
            "detection_valid_vs_time.png",
            det_times,
            [("detection_valid", [1.0 if bool_value(r, "detection_valid") else 0.0 for r in rows])],
            "Detection Valid vs Time",
            "valid",
        )
        if "detection_confidence" in rows[0]:
            save_line(
                "detection_confidence_vs_time.png",
                det_times,
                [("detection_confidence", [to_float(r, "detection_confidence") for r in rows])],
                "Detection Confidence vs Time",
                "confidence",
            )
        if "detection_point_count" in rows[0]:
            save_line(
                "detection_point_count_vs_time.png",
                det_times,
                [("detection_point_count", [to_float(r, "detection_point_count") for r in rows])],
                "Detection Point Count vs Time",
                "points",
            )
        if "observation_age" in rows[0]:
            save_line(
                "observation_age_vs_time.png",
                det_times,
                [("observation_age", [to_float(r, "observation_age") for r in rows])],
                "Observation Age vs Time",
                "age [s]",
            )
        if "tracking_state" in rows[0]:
            state_value = {"TRACKING": 2.0, "PREDICT_ONLY": 1.0, "LOST": 0.0}
            save_line(
                "tracking_state_timeline.png",
                det_times,
                [
                    (
                        "tracking_state",
                        [state_value.get(str(r.get("tracking_state", "")).upper(), float("nan")) for r in rows],
                    )
                ],
                "Tracking State Timeline",
                "LOST=0, PREDICT_ONLY=1, TRACKING=2",
            )
        if "detector_state" in rows[0]:
            detector_state_value = {
                "LOST": -1.0,
                "SEARCH": 0.0,
                "REACQUIRE": 0.5,
                "ACQUIRE": 1.0,
                "COAST": 1.5,
                "TRACK": 2.0,
            }
            save_line(
                "detector_state_timeline.png",
                det_times,
                [
                    (
                        "detector_state",
                        [
                            detector_state_value.get(
                                str(r.get("detector_state", "")).upper(),
                                float("nan"),
                            )
                            for r in rows
                        ],
                    )
                ],
                "Detector State Timeline",
                "LOST=-1, SEARCH=0, REACQUIRE=0.5, ACQUIRE=1, COAST=1.5, TRACK=2",
            )
        if "candidate_count" in rows[0]:
            save_line(
                "candidate_count_vs_time.png",
                det_times,
                [("candidate_count", [to_float(r, "candidate_count") for r in rows])],
                "Candidate Count vs Time",
                "candidates",
            )
            plt.figure()
            plt.scatter(
                [to_float(r, "candidate_count") for r in rows],
                [1.0 if bool_value(r, "detection_valid") else 0.0 for r in rows],
                s=8,
                alpha=0.5,
            )
            plt.title("Candidate Count vs Detection Valid")
            plt.xlabel("candidate_count")
            plt.ylabel("detection_valid")
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(os.path.join(out_dir, "candidate_count_vs_detection_valid.png"))
            plt.close()
        if "gate_radius" in rows[0]:
            save_line(
                "gate_radius_vs_time.png",
                det_times,
                [("gate_radius", [to_float(r, "gate_radius") for r in rows])],
                "Gate Radius vs Time",
                "radius [m]",
            )
        if "selected_cluster_score" in rows[0]:
            save_line(
                "selected_score_vs_time.png",
                det_times,
                [
                    (
                        "selected_cluster_score",
                        [to_float(r, "selected_cluster_score") for r in rows],
                    )
                ],
                "Selected Cluster Score vs Time",
                "score",
            )
        if "association_margin" in rows[0]:
            save_line(
                "association_margin_vs_time.png",
                det_times,
                [("association_margin", [to_float(r, "association_margin") for r in rows])],
                "Association Margin vs Time",
                "score margin",
            )
        save_line(
            "detection_error_vs_time.png",
            det_times,
            [
                (
                    "detection_error_3d",
                    [to_float(r, "detection_error_3d") for r in rows],
                )
            ],
            "Detection Error vs Time",
            "error [m]",
        )
        if "detector_state" in rows[0] and det_errors:
            states = sorted(
                set(str(r.get("detector_state", "")).upper() for r in rows)
            )
            values = []
            labels = []
            for state in states:
                state_values = [
                    to_float(r, "detection_error_3d")
                    for r in rows
                    if str(r.get("detector_state", "")).upper() == state
                    and finite(to_float(r, "detection_error_3d"))
                ]
                if state_values:
                    labels.append(state)
                    values.append(mean(state_values))
            if values:
                plt.figure()
                plt.bar(labels, values)
                plt.title("Detection Error by Detector State")
                plt.xlabel("detector_state")
                plt.ylabel("mean detection_error_3d [m]")
                plt.grid(True, axis="y")
                plt.tight_layout()
                plt.savefig(os.path.join(out_dir, "detection_error_by_state.png"))
                plt.close()

        plt.figure()
        plt.hist(det_errors, bins=40)
        plt.title("Detection Error 3D Histogram")
        plt.xlabel("detection_error_3d [m]")
        plt.ylabel("count")
        if det_errors:
            text = "mean={:.2f}\np90={:.2f}\nmax={:.2f}".format(
                mean(det_errors),
                percentile(det_errors, 0.9),
                max(det_errors),
            )
            plt.gca().text(
                0.98,
                0.95,
                text,
                transform=plt.gca().transAxes,
                ha="right",
                va="top",
                bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8},
            )
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, "detection_error_histogram.png"))
        plt.close()

        plt.figure()
        plt.plot(
            [to_float(r, "uav1_x") for r in rows],
            [to_float(r, "uav1_y") for r in rows],
            label="truth",
        )
        plt.scatter(
            [to_float(r, "target_observation_x") for r in rows],
            [to_float(r, "target_observation_y") for r in rows],
            s=5,
            alpha=0.5,
            label="observation",
        )
        plt.title("Observation vs Truth XY")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, "observation_vs_truth_xy.png"))
        plt.close()

        save_line(
            "observation_vs_truth_z.png",
            det_times,
            [
                ("truth_z", [to_float(r, "uav1_z") for r in rows]),
                (
                    "observation_z",
                    [to_float(r, "target_observation_z") for r in rows],
                ),
            ],
            "Observation vs Truth Z",
            "z [m]",
        )

    has_fusion_fields = bool(rows) and "target_fused_valid" in rows[0]
    if has_fusion_fields:
        fusion_times = [
            to_float(row, "time")
            for row in rows
            if finite(to_float(row, "time"))
        ]
        save_line(
            "visual_valid_vs_time.png",
            fusion_times,
            [("visual_valid", [1.0 if bool_value(r, "target_visual_valid") else 0.0 for r in rows])],
            "Visual Valid vs Time",
            "valid",
        )
        save_line(
            "lidar_valid_vs_time.png",
            fusion_times,
            [("lidar_valid", [1.0 if bool_value(r, "target_lidar_valid") else 0.0 for r in rows])],
            "LiDAR Valid vs Time",
            "valid",
        )
        save_line(
            "fused_valid_vs_time.png",
            fusion_times,
            [("fused_valid", [1.0 if bool_value(r, "target_fused_valid") else 0.0 for r in rows])],
            "Fused Valid vs Time",
            "valid",
        )
        save_line(
            "visual_lidar_fused_error_vs_time.png",
            fusion_times,
            [
                ("visual", [to_float(r, "target_visual_error_3d") for r in rows]),
                ("lidar", [to_float(r, "target_lidar_error_3d") for r in rows]),
                ("fused", [to_float(r, "target_fused_error_3d") for r in rows]),
                (
                    "estimator",
                    [to_float(r, "target_estimator_error_3d") for r in rows],
                ),
            ],
            "Visual/LiDAR/Fused Error vs Time",
            "error [m]",
        )
        source_value = {"none": 0.0, "lidar": 1.0, "visual": 2.0, "fused": 3.0}
        source_series = [
            source_value.get(str(r.get("target_fused_source", "none")).lower(), 0.0)
            if bool_value(r, "target_fused_valid")
            else 0.0
            for r in rows
        ]
        save_line(
            "fused_source_timeline.png",
            fusion_times,
            [("source", source_series)],
            "Fused Source Timeline",
            "none=0, lidar=1, visual=2, fused=3",
        )
        save_line(
            "detection_source_vs_time.png",
            fusion_times,
            [
                ("visual_valid", [1.0 if bool_value(r, "target_visual_valid") else 0.0 for r in rows]),
                ("lidar_valid", [1.0 if bool_value(r, "target_lidar_valid") else 0.0 for r in rows]),
                ("fused_source", source_series),
            ],
            "Detection Source vs Time",
            "valid/source",
        )

    try:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(
            [to_float(r, "uav0_x") for r in rows],
            [to_float(r, "uav0_y") for r in rows],
            [to_float(r, "uav0_z") for r in rows],
            label="uav0",
        )
        ax.plot(
            [to_float(r, "uav1_x") for r in rows],
            [to_float(r, "uav1_y") for r in rows],
            [to_float(r, "uav1_z") for r in rows],
            label="uav1",
        )
        ax.plot(
            [to_float(r, "pred_x") for r in rows],
            [to_float(r, "pred_y") for r in rows],
            [to_float(r, "pred_z") for r in rows],
            label="pred",
        )
        ax.set_title("3D Trajectory")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, "trajectory_3d.png"))
        plt.close()
    except Exception as exc:
        print("failed to generate trajectory_3d.png: {}".format(exc))


def write_summary(out_dir, summary):
    os.makedirs(out_dir, exist_ok=True)
    overall_keys = [
        "capture_success",
        "capture_time",
        "min_distance_3d",
        "mean_distance_3d",
        "mean_prediction_error_3d",
        "p90_prediction_error_3d",
        "max_prediction_error_3d",
        "mean_reachable_margin",
        "percent_reachable_margin_positive",
        "mean_detection_error_3d",
        "p90_detection_error_3d",
        "max_detection_error_3d",
        "detection_rate",
        "mean_detection_confidence",
        "mean_detection_point_count",
        "min_detection_point_count",
        "target_lost_count",
        "target_lost_duration_total",
        "longest_detection_gap",
        "predict_only_duration_total",
        "predict_only_ratio",
        "visual_detection_rate",
        "lidar_detection_rate",
        "fused_detection_rate",
        "visual_mean_error_3d",
        "lidar_mean_error_3d",
        "fused_mean_error_3d",
        "target_estimator_mean_error_3d",
        "source_ratio_visual",
        "source_ratio_lidar",
        "source_ratio_fused",
        "source_ratio_none",
        "visual_lost_count",
        "lidar_fallback_count",
        "visual_lost_duration_total",
        "lidar_fallback_duration_total",
        "full_lost_duration_total",
        "fused_none_duration",
        "mean_candidate_count",
        "mean_range_valid_cluster_count",
        "candidate_nonzero_but_no_detection_count",
        "gate_reject_suspected_count",
        "mean_association_margin",
        "low_association_margin_count",
        "mean_selected_cluster_score",
        "mean_selected_distance_to_gate",
        "reacquire_count_total",
        "lost_duration_total",
        "longest_lost_duration",
        "detection_rate_by_state",
        "detection_error_by_state",
        "selected_wrong_cluster_suspected_count",
    ]
    terminal_keys = [
        "terminal_window",
        "terminal_mean_distance_xy",
        "terminal_min_distance_xy",
        "terminal_mean_distance_z_signed",
        "terminal_mean_abs_z_error",
        "terminal_min_distance_3d",
        "terminal_mean_prediction_error_3d",
        "terminal_mean_prediction_error_xy",
        "terminal_mean_prediction_error_z",
        "terminal_mean_reachable_margin",
        "terminal_percent_reachable_margin_positive",
        "terminal_mean_v_cmd_z",
        "terminal_max_abs_v_cmd_z",
        "terminal_mean_yaw_error",
        "terminal_yaw_gating_ratio",
        "terminal_yaw_aligned_ratio",
    ]
    close_keys = [
        "close_xy_threshold",
        "close_xy_sample_count",
        "close_xy_below_ratio",
        "close_xy_mean_distance_z_signed",
        "close_xy_mean_abs_z_error",
        "close_xy_max_underpass_z",
        "close_xy_mean_v_cmd_z",
        "close_3d_25_window_count",
        "close_3d_25_good_window_count",
        "close_3d_25_total_time",
        "close_3d_25_first_good_time",
        "close_3d_35_window_count",
        "close_3d_35_good_window_count",
        "close_3d_35_total_time",
        "close_3d_35_first_good_time",
        "close_3d_50_window_count",
        "close_3d_50_good_window_count",
        "close_3d_50_total_time",
        "close_3d_50_first_good_time",
        "close_3d_100_window_count",
        "close_3d_100_good_window_count",
        "close_3d_100_total_time",
        "close_3d_100_first_good_time",
    ]
    with open(os.path.join(out_dir, "summary.txt"), "w") as handle:
        handle.write("[Overall Metrics]\n")
        handle.write("estimator_model: {}\n".format(summary.get("estimator_model", "")))
        handle.write("target_mode: {}\n".format(summary.get("target_mode", "")))
        for key in overall_keys:
            handle.write("{}: {}\n".format(key, summary.get(key, float("nan"))))
        handle.write("\n[Terminal Window Metrics]\n")
        for key in terminal_keys:
            if key in summary:
                handle.write("{}: {}\n".format(key, summary[key]))
        handle.write("\n[Close XY Metrics]\n")
        for key in close_keys:
            if key in summary:
                handle.write("{}: {}\n".format(key, summary[key]))
        handle.write("\n[Diagnosis]\n")
        handle.write("{}\n".format(summary.get("diagnosis", "")))


def analyze(path, vxy_max, vz_max, out_dir, terminal_window=3.0, make_plots=True):
    rows = load_rows(path)
    summary, metrics = compute_metrics(rows, vxy_max, vz_max, terminal_window)
    if make_plots:
        times = [to_float(r, "time") for r in rows]
        plot_outputs(out_dir, times, metrics, rows, terminal_window)
    if out_dir:
        write_summary(out_dir, summary)
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    parser.add_argument("--vxy-max", type=float, default=1.8)
    parser.add_argument("--vz-max", type=float, default=0.8)
    parser.add_argument("--terminal-window", type=float, default=3.0)
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args()

    out_dir = args.out_dir
    if out_dir is None:
        base, _ = os.path.splitext(args.csv_file)
        out_dir = base + "_analysis"

    summary = analyze(
        args.csv_file,
        args.vxy_max,
        args.vz_max,
        out_dir,
        terminal_window=args.terminal_window,
    )
    print("Summary:")
    for key, value in summary.items():
        print("{}: {}".format(key, value))


if __name__ == "__main__":
    main()
