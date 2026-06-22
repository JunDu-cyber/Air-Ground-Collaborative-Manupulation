#!/usr/bin/env bash
set -euo pipefail

# 默认取本脚本所在工作区(scripts/../../.. = catkin 根)，不写死某台机器的路径
WS_DIR="${WS_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
LOG_DIR="${LOG_DIR:-$HOME/uav_intercept_logs}"
TEST_DURATION="${TEST_DURATION:-60}"
TARGET_MODE="${TARGET_MODE:-circle_z_sine}"
UAV1_SPEED="${UAV1_SPEED:-0.6}"
Z_AMPLITUDE="${Z_AMPLITUDE:-0.5}"

FORMAL_ONLY=true
INCLUDE_DEBUG=false

usage() {
  cat <<'EOF'
Usage: run_estimator_model_tests.sh [--formal-only] [--include-debug]

Default formal models:
  kalman_cv kalman_ca

Debug/legacy models added by --include-debug:
  current linear_observation learned_circle circle_configured_debug
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --formal-only)
      FORMAL_ONLY=true
      ;;
    --include-debug)
      INCLUDE_DEBUG=true
      FORMAL_ONLY=false
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

MODELS=(kalman_cv kalman_ca)
if [ "$INCLUDE_DEBUG" = true ]; then
  MODELS+=(current linear_observation learned_circle circle_configured_debug)
fi

mkdir -p "$LOG_DIR"

if [ -f "$WS_DIR/devel/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$WS_DIR/devel/setup.bash"
fi

echo "Writing CSV logs to: $LOG_DIR"
echo "Target mode: $TARGET_MODE, uav1_speed: $UAV1_SPEED, z_amplitude: $Z_AMPLITUDE"
echo "Formal only: $FORMAL_ONLY, include debug: $INCLUDE_DEBUG"
echo "Models: ${MODELS[*]}"
echo "Each model duration: ${TEST_DURATION}s"

for MODEL in "${MODELS[@]}"; do
  echo
  echo "=== estimator_model=$MODEL ==="
  timeout --preserve-status "$TEST_DURATION" roslaunch uav_truth_tracker intercept_estimator_velocity_chase.launch \
    target_mode:="$TARGET_MODE" \
    uav1_speed:="$UAV1_SPEED" \
    z_amplitude:="$Z_AMPLITUDE" \
    estimator_model:="$MODEL" \
    start_evaluator:=true \
    evaluator_log_dir:="$LOG_DIR" || STATUS=$?

  STATUS="${STATUS:-0}"
  if [ "$STATUS" -ne 0 ] && [ "$STATUS" -ne 143 ] && [ "$STATUS" -ne 124 ]; then
    echo "Model $MODEL exited with status $STATUS"
  fi
  unset STATUS
  sleep 3
done

echo
echo "Done. Analyze a CSV with:"
echo "  rosrun uav_truth_tracker analyze_prediction_log.py <csv_file> --vxy-max 1.8 --vz-max 0.8 --terminal-window 3.0"
