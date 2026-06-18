#!/bin/bash
set -euo pipefail

# Enhanced multi-world YOLO dataset collection for improved detection confidence.
# Uses UAV1 truth odom for real-time tracking + auto-labeling.

EGO_WS="${EGO_WS:-$HOME/ego_ws}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
DATASET_DIR="${YOLO_DATASET_DIR:-$HOME/uav_yolo_dataset}"
IMAGES_PER_WORLD="${IMAGES_PER_WORLD:-1500}"
COLLECTION_TIMEOUT="${COLLECTION_TIMEOUT:-1200}"
POLL_INTERVAL="${POLL_INTERVAL:-5}"
PX4_GAZEBO_GUI="${PX4_GAZEBO_GUI:-false}"
Z_AMPLITUDE="${Z_AMPLITUDE:-0.5}"

LIGHT_WORLD="$EGO_WS/src/uav_truth_tracker/worlds/lidar_clutter_light.world"
PX4_WORLDS="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"

# --- World + trajectory combinations for diverse data ---
# Each entry: WORLD_PATH|TARGET_MODE|UAV1_SPEED|FOLLOW_DISTANCE
COMBOS=(
  "$PX4_WORLDS/empty.world|circle_z_sine|0.8|2.5"
  "$PX4_WORLDS/empty.world|figure8_3d|0.6|3.5"
  "$LIGHT_WORLD|circle_z_sine|1.0|2.0"
  "$LIGHT_WORLD|figure8_3d|0.8|4.0"
  "$PX4_WORLDS/warehouse.world|circle_z_sine|0.6|2.5"
  "$PX4_WORLDS/baylands.world|circle_z_sine|0.8|3.0"
)

# Allow user override
if (( $# > 0 )); then
  COMBOS=()
  for w in "$@"; do
    COMBOS+=("$w|circle_z_sine|0.8|2.5")
  done
fi

count_labels() {
  find "$DATASET_DIR/labels" -type f -name '*.txt' 2>/dev/null | wc -l
}

if [[ ! -x "$EGO_WS/one_key_intercept.sh" ]]; then
  echo "ERROR: one-key script not found: $EGO_WS/one_key_intercept.sh"
  exit 1
fi

mkdir -p "$DATASET_DIR"

echo "=============================================="
echo "  Enhanced Multi-World UAV YOLO Collection"
echo "=============================================="
echo "Dataset: $DATASET_DIR"
echo "Images per combo: $IMAGES_PER_WORLD"
echo "Gazebo GUI: $PX4_GAZEBO_GUI"
echo "Total combos: ${#COMBOS[@]}"
echo "Estimated total: $((${#COMBOS[@]} * IMAGES_PER_WORLD)) images"
echo ""

combo_idx=0
for combo in "${COMBOS[@]}"; do
  IFS='|' read -r world target_mode uav1_speed follow_distance <<< "$combo"
  combo_idx=$((combo_idx + 1))

  if [[ ! -f "$world" ]]; then
    echo "WARNING: skipping missing world: $world"
    continue
  fi

  tag="$(basename "$world" .world)_${target_mode}_s${uav1_speed}"
  start_count="$(count_labels)"
  target_count=$((start_count + IMAGES_PER_WORLD))

  echo ""
  echo "[$combo_idx/${#COMBOS[@]}] World: $(basename "$world")"
  echo "  Trajectory: $target_mode | Speed: $uav1_speed | Follow: ${follow_distance}m"
  echo "  Tag: $tag | Target: $target_count labels"

  (
    cd "$EGO_WS"
    WORLD="$world" \
    PX4_GAZEBO_GUI="$PX4_GAZEBO_GUI" \
    YOLO_DATASET_DIR="$DATASET_DIR" \
    YOLO_MAX_IMAGES=0 \
    YOLO_MAX_NEW_IMAGES="$IMAGES_PER_WORLD" \
    YOLO_DATASET_TAG="$tag" \
    TARGET_MODE="$target_mode" \
    UAV1_SPEED="$uav1_speed" \
    Z_AMPLITUDE="$Z_AMPLITUDE" \
    FOLLOW_DISTANCE="$follow_distance" \
      ./one_key_intercept.sh yolo_collect_truth
  )

  waited=0
  while (( waited < COLLECTION_TIMEOUT )); do
    current_count="$(count_labels)"
    remaining=$((target_count - current_count))
    if (( remaining <= 0 )); then remaining=0; fi
    echo "  [$tag] labels: $current_count / $target_count (remaining: $remaining)"
    if (( current_count >= target_count )); then
      break
    fi
    sleep "$POLL_INTERVAL"
    waited=$((waited + POLL_INTERVAL))
  done

  if (( $(count_labels) < target_count )); then
    echo "WARNING: collection timed out for combo: $tag (continuing to next)"
  fi

  # Kill remaining processes before next world
  echo "  Cleaning up processes..."
  pkill -f "px4.*sitl" 2>/dev/null || true
  pkill -f "gzserver" 2>/dev/null || true
  pkill -f "gzclient" 2>/dev/null || true
  sleep 5
done

echo ""
echo "=============================================="
echo "  Collection Complete"
echo "=============================================="
echo "Dataset: $DATASET_DIR"
echo "Total labels: $(count_labels)"
echo ""
echo "Positive: $(find "$DATASET_DIR/labels" -name '*.txt' -not -empty | wc -l)"
echo "Negative: $(find "$DATASET_DIR/labels" -name '*.txt' -empty | wc -l)"
echo ""
echo "Next step - train:"
echo "  python3 $EGO_WS/src/uav_truth_tracker/scripts/train_uav_yolo.py \\"
echo "    $DATASET_DIR --epochs 150 --device 0"
