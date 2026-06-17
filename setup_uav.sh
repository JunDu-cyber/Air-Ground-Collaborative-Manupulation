#!/usr/bin/env bash
# ============================================================================
# Air-Ground UAV 仿真环境一键安装（含 PX4 SITL）
#
# 安装完成后队友即可启动 PX4 + MAVROS + EGO-Planner UAV 建图。
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

echo "============================================"
echo "  Air-Ground UAV 环境安装"
echo "============================================"
echo "工作空间: $WS_DIR"
echo "PX4 目录: $PX4_DIR"
echo ""

# ---- ROS 依赖 ----
echo "[1/6] 安装 ROS 依赖..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-octomap-ros ros-noetic-octomap-msgs ros-noetic-octomap-server \
    ros-noetic-grid-map-visualization \
    ros-noetic-mavros ros-noetic-mavros-extras \
    ros-noetic-robot-localization ros-noetic-move-base ros-noetic-amcl \
    ros-noetic-twist-mux ros-noetic-nodelet \
    ros-noetic-rviz ros-noetic-tf2-ros ros-noetic-rqt ros-noetic-rqt-reconfigure \
    python3-pip python3-catkin-tools

# ---- ego-planner ----
echo "[2/6] 克隆 EGO-Planner..."
cd "$WS_DIR/src"
if [ ! -d ego-planner ]; then
    git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
fi

# ---- PX4 SITL ----
echo "[3/6] 安装 PX4 SITL（首次约需 15-30 分钟）..."
if [ ! -d "$PX4_DIR" ]; then
    cd "$HOME"
    git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.14.3 --depth 1
    cd "$PX4_DIR"
    git submodule update --init --recursive --depth 1
    bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
    make px4_sitl_default gazebo-classic
else
    echo "  PX4 已存在，跳过 clone。"
fi

# ---- Install geographiclib datasets（MAVROS 需要） ----
echo "[4/6] 安装 MAVROS 地理数据集..."
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh 2>/dev/null || true

# ---- UAV 模型 ----
echo "[5/6] 设置 Gazebo 模型路径..."
mkdir -p "$HOME/.gazebo/models"
# 设置环境变量
if ! grep -q "PX4-Autopilot" "$HOME/.bashrc" 2>/dev/null; then
    cat >> "$HOME/.bashrc" << 'BASHEOF'

# PX4 + Gazebo 路径
export PX4_DIR="$HOME/PX4-Autopilot"
export GAZEBO_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
export GAZEBO_MODEL_PATH="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH"
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default" 2>/dev/null || true
BASHEOF
fi

# ---- 编译 ----
echo "[6/6] 编译 workspace..."
# 移除 OctoMap 依赖包的临时屏蔽
rm -f "$WS_DIR/src/forest_avoidance_utils/CATKIN_IGNORE"
cd "$WS_DIR"
catkin_make -j$(nproc) 2>&1 | tail -5
source devel/setup.bash

# ---- 输出目录 ----
mkdir -p "$HOME/pointcloud_maps"

echo ""
echo "============================================"
echo "  安装完成！"
echo "============================================"
echo ""
echo "=== 启动 UAV 建图（PX4 + MAVROS + EGO-Planner） ==="
echo "  cd $WS_DIR && bash one_key_takeoff.sh"
echo ""
echo "=== PCD → 高程图 ==="
echo "  source devel/setup.bash"
echo "  roslaunch mobile_manipulator pcd_to_elevation.launch \\"
echo "      pcd_file:=~/pointcloud_maps/uav_points_map_latest.pcd"
echo ""
