#!/usr/bin/env bash
# ============================================================================
# Egocentric UGV navigation stack setup (CMU local_planner + DLIO).
#
# Clones + patches + builds the LiDAR-inertial odometry + CMU local_planner
# stack used by egocentric_nav.launch. Idempotent: safe to re-run.
#
# After this, run the egocentric nav demo with:
#   roslaunch mobile_manipulator egocentric_nav.launch \
#       nav:=cmu odom_source:=dlio cost_source:=terrain_analysis gui:=false
#   (GPU + an X display on :0 are required for the Velodyne sensor)
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR"
SRC="$WS_DIR/src"
AEDE="$SRC/autonomous_exploration_development_environment"

echo "============================================"
echo "  Egocentric UGV navigation stack setup"
echo "============================================"

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "  ERROR: source /opt/ros/noetic/setup.bash first"; exit 1
fi

# ---- 1. apt deps -----------------------------------------------------------
echo "[1/5] apt dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
  ros-noetic-velodyne-description ros-noetic-velodyne-gazebo-plugins \
  ros-noetic-sensor-filters ros-noetic-filters \
  ros-noetic-geometric-shapes ros-noetic-moveit-core ros-noetic-moveit-ros-perception \
  ros-noetic-fcl python3-catkin-tools

# ---- 2. clone upstream -----------------------------------------------------
echo "[2/5] cloning upstream into src/..."
cd "$SRC"
[ -d autonomous_exploration_development_environment ] || \
  git clone --depth 1 -b noetic https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git
[ -d direct_lidar_inertial_odometry ] || \
  git clone --depth 1 https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git
[ -d robot_body_filter ] || \
  git clone --depth 1 https://github.com/peci1/robot_body_filter.git
# FAST_LIO is the optional STEP 2 A/B baseline. Build it with BUILD_FASTLIO=1.
# It needs (a) livox_ros_driver for the CustomMsg type (the driver auto-bootstraps
# the Livox-SDK at build time) and (b) its own ikd-Tree / IKFoM_toolkit submodules.
if [ "${BUILD_FASTLIO:-0}" = "1" ]; then
  [ -d livox_ros_driver ] || git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git
  if [ ! -d FAST_LIO ]; then
    git clone --depth 1 --recursive https://github.com/hku-mars/FAST_LIO.git
  fi
  ( cd FAST_LIO && git submodule update --init --recursive )   # ikd-Tree, IKFoM_toolkit
  rm -f FAST_LIO/CATKIN_IGNORE
else
  [ -d FAST_LIO ] || git clone --depth 1 https://github.com/hku-mars/FAST_LIO.git || true
  [ -d FAST_LIO ] && touch FAST_LIO/CATKIN_IGNORE
fi

# ---- 3. CMU AEDE: build only the subset we need ----------------------------
echo "[3/5] trimming CMU AEDE to the needed packages..."
KEEP="local_planner terrain_analysis terrain_analysis_ext sensor_scan_generation loam_interface"
for d in "$AEDE"/src/*/; do
  pkg="$(basename "$d")"
  if echo "$KEEP" | grep -qw "$pkg"; then rm -f "$d/CATKIN_IGNORE"; else touch "$d/CATKIN_IGNORE"; fi
done

# ---- 4. patches ------------------------------------------------------------
echo "[4/5] applying source patches..."

# 4a. CMU world frame map -> odom (egocentric: no map frame). "map" appears in
#     these files ONLY as frame_ids, so a global replace is safe + idempotent.
for f in loam_interface/src/loamInterface.cpp \
         sensor_scan_generation/src/sensorScanGeneration.cpp \
         terrain_analysis/src/terrainAnalysis.cpp; do
  sed -i 's/"map"/"odom"/g' "$AEDE/src/$f"
done

# 4b. DLIO: don't prepend an empty-namespace "/" to frame names when running in
#     the global namespace, so frames stay exactly odom/base_link/velodyne/imu.
DLIO_ODOM="$SRC/direct_lidar_inertial_odometry/src/dlio/odom.cc"
if ! grep -q 'if (!ns.empty())' "$DLIO_ODOM"; then
  perl -0pi -e 's/(\n)(  this->odom_frame = ns \+ "\/" \+ this->odom_frame;\n  this->baselink_frame = ns \+ "\/" \+ this->baselink_frame;\n  this->lidar_frame = ns \+ "\/" \+ this->lidar_frame;\n  this->imu_frame = ns \+ "\/" \+ this->imu_frame;)/$1  if (!ns.empty()) {$2\n  }/' "$DLIO_ODOM"
  echo "  patched DLIO ns guard"
else
  echo "  DLIO ns guard already present"
fi

# ---- 5. build --------------------------------------------------------------
echo "[5/5] building (catkin build — this workspace uses catkin_tools)..."
cd "$WS_DIR"
PKGS="loam_interface local_planner terrain_analysis terrain_analysis_ext \
  sensor_scan_generation direct_lidar_inertial_odometry robot_body_filter \
  terrain_cost_adapter"
[ "${BUILD_FASTLIO:-0}" = "1" ] && PKGS="$PKGS livox_ros_driver fast_lio"
catkin build $PKGS -j"$(nproc --ignore=2)"

echo ""
echo "============================================"
echo "  Done. Source the workspace and launch:"
echo "    source devel/setup.bash"
echo "    DISPLAY=:0 roslaunch mobile_manipulator egocentric_nav.launch nav:=cmu gui:=false"
echo "============================================"
