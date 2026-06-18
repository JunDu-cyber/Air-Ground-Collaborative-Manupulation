#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/home/lnwuu/ego_ws}"
LOG_DIR="${LOG_DIR:-$HOME/uav_intercept_logs}"
TEST_DURATION="${TEST_DURATION:-60}"
TARGET_MODE="${TARGET_MODE:-circle_z_sine}"
UAV1_SPEED="${UAV1_SPEED:-0.6}"
Z_AMPLITUDE="${Z_AMPLITUDE:-0.5}"
MODEL="${MODEL:-kalman_cv}"

RUN_STAGE1=true
PRINT_KALMAN_GRID=false

usage() {
  cat <<'EOF'
Usage: run_kalman_tuning_sweep.sh [--stage1] [--print-kalman-grid]

Default stage1 recommended t_go/reachability configurations:
  A: xy=1.5 z=0.8 max_t=2.5
  B: xy=1.4 z=0.8 max_t=2.5
  C: xy=1.6 z=0.8 max_t=2.5
  D: xy=1.5 z=0.6 max_t=2.5
  E: xy=1.5 z=1.0 max_t=2.5
  F: xy=1.5 z=0.8 max_t=3.0

Environment variables:
  MODEL=kalman_cv|kalman_ca
  TEST_DURATION=60
  LOG_DIR=$HOME/uav_intercept_logs
  TARGET_MODE=circle_z_sine
  UAV1_SPEED=0.6
  Z_AMPLITUDE=0.5
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --stage1)
      RUN_STAGE1=true
      ;;
    --print-kalman-grid)
      PRINT_KALMAN_GRID=true
      RUN_STAGE1=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [ "$MODEL" != "kalman_cv" ] && [ "$MODEL" != "kalman_ca" ]; then
  echo "MODEL must be kalman_cv or kalman_ca, got: $MODEL" >&2
  exit 2
fi

if [ -f "$WS_DIR/devel/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/devel/setup.bash"
fi

mkdir -p "$LOG_DIR"

print_kalman_grid() {
  cat <<EOF
# Stage 2 Kalman parameter scan candidates. Run a small subset manually after
# identifying whether terminal bias is prediction, reachability, or control limited.

for kf_process_noise_vel in 0.5 1.0 1.5; do
  for kf_measurement_noise in 0.04 0.06 0.10; do
    for kf_velocity_blend in 0.35 0.55 0.75; do
      for velocity_fit_window in 0.4 0.5 0.7; do
        echo roslaunch uav_truth_tracker intercept_estimator_velocity_chase.launch \\
          target_mode:=$TARGET_MODE \\
          uav1_speed:=$UAV1_SPEED \\
          z_amplitude:=$Z_AMPLITUDE \\
          estimator_model:=$MODEL \\
          kf_process_noise_vel:=\$kf_process_noise_vel \\
          kf_measurement_noise:=\$kf_measurement_noise \\
          kf_velocity_blend:=\$kf_velocity_blend \\
          velocity_fit_window:=\$velocity_fit_window \\
          start_evaluator:=true \\
          evaluator_log_dir:=$LOG_DIR
      done
    done
  done
done
EOF
}

if [ "$PRINT_KALMAN_GRID" = true ]; then
  print_kalman_grid
  exit 0
fi

CONFIGS=(
  "A 1.5 0.8 2.5"
  "B 1.4 0.8 2.5"
  "C 1.6 0.8 2.5"
  "D 1.5 0.6 2.5"
  "E 1.5 1.0 2.5"
  "F 1.5 0.8 3.0"
)

if [ "$RUN_STAGE1" = true ]; then
  echo "Running stage1 reachability sweep for MODEL=$MODEL"
  echo "Logs: $LOG_DIR"
  for entry in "${CONFIGS[@]}"; do
    read -r NAME XY_SPEED Z_SPEED MAX_T <<<"$entry"
    echo
    echo "=== $NAME: xy=$XY_SPEED z=$Z_SPEED max_t=$MAX_T model=$MODEL ==="
    timeout --preserve-status "$TEST_DURATION" roslaunch uav_truth_tracker intercept_estimator_velocity_chase.launch \
      target_mode:="$TARGET_MODE" \
      uav1_speed:="$UAV1_SPEED" \
      z_amplitude:="$Z_AMPLITUDE" \
      estimator_model:="$MODEL" \
      assumed_chaser_speed_xy:="$XY_SPEED" \
      assumed_chaser_speed_z:="$Z_SPEED" \
      max_prediction_time:="$MAX_T" \
      start_evaluator:=true \
      evaluator_log_dir:="$LOG_DIR" || STATUS=$?

    STATUS="${STATUS:-0}"
    if [ "$STATUS" -ne 0 ] && [ "$STATUS" -ne 143 ] && [ "$STATUS" -ne 124 ]; then
      echo "Sweep config $NAME exited with status $STATUS"
    fi
    unset STATUS
    sleep 3
  done
fi

echo
echo "Summarize with:"
echo "  rosrun uav_truth_tracker summarize_prediction_results.py $LOG_DIR"
