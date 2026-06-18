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

# ---- 前置检查 ----
echo "[0/7] 前置检查..."

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "  错误: ROS 环境未加载，请先 source /opt/ros/noetic/setup.bash"
    exit 1
fi
if [ "$ROS_DISTRO" != "noetic" ]; then
    echo "  警告: 当前 ROS 发行版为 '$ROS_DISTRO'，本脚本针对 noetic 编写，可能不兼容"
fi
if [ ! -d "$WS_DIR/src" ]; then
    echo "  错误: 工作空间 src/ 目录不存在: $WS_DIR/src"
    exit 1
fi

# 磁盘空间检查（PX4 + build 约需 5 GB）
AVAIL_GB=$(df -BG "$WS_DIR" 2>/dev/null | awk 'NR==2 {print $4}' | sed 's/G//')
if [[ "$AVAIL_GB" =~ ^[0-9]+$ ]] && [ "$AVAIL_GB" -lt 10 ]; then
    echo "  警告: 可用磁盘空间仅 ${AVAIL_GB} GB，建议至少 10 GB"
fi

echo ""

# ---- ROS 依赖 ----
echo "[1/7] 安装 ROS 依赖..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-octomap-ros ros-noetic-octomap-msgs ros-noetic-octomap-server \
    ros-noetic-grid-map-visualization \
    ros-noetic-mavros ros-noetic-mavros-extras \
    ros-noetic-robot-localization ros-noetic-move-base ros-noetic-amcl \
    ros-noetic-twist-mux ros-noetic-nodelet \
    ros-noetic-rviz ros-noetic-tf2-ros ros-noetic-rqt ros-noetic-rqt-reconfigure \
    python3-pip python3-catkin-tools \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# ---- ego-planner ----
echo "[2/7] 克隆 EGO-Planner..."
cd "$WS_DIR/src"
if [ ! -d ego-planner ]; then
    git clone https://github.com/ZJU-FAST-Lab/ego-planner.git --depth 1
fi

# ---- rosdep ----
echo "[3/7] 安装工作空间依赖 (rosdep)..."
cd "$WS_DIR"
rosdep update 2>/dev/null || true
if ! rosdep install --from-paths src --ignore-src -r -y; then
    echo "  警告: rosdep 部分依赖缺失，构建可能失败（请检查上方输出）"
fi

# ---- PX4 SITL ----
echo "[4/7] 安装 PX4 SITL（首次约需 15-30 分钟）..."
if [ ! -d "$PX4_DIR" ]; then
    cd "$HOME"
    git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.14.3 --depth 1
    cd "$PX4_DIR"
    git submodule update --init --recursive --depth 1
    # 修复 PX4 v1.14.3 pip 兼容性 (.* wildcard 已被 pip 移除)
    sed -i 's/matplotlib>=3\.0\.\*/matplotlib>=3.0,<4.0/' Tools/setup/requirements.txt
    # 修复 shallow-clone 时缺少 NuttX 标签导致构建失败
    sed -i 's/\(nuttx_git_tag = \)re\.findall(\(.*\))\[-1\]/\1(re.findall(\2) or ["nuttx-0.0.0"])[-1]/' src/lib/version/px_update_git_header.py

    bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

    # 预检 Gazebo（--no-sim-tools 跳过了 Gazebo 安装）
    if ! command -v gzserver &>/dev/null && ! command -v gazebo &>/dev/null; then
        echo "  错误: 未找到 gazebo/gzserver。请先安装："
        echo "    sudo apt-get install -y ros-noetic-gazebo-ros-pkgs"
        echo "  或重新运行 PX4 ubuntu.sh 去掉 --no-sim-tools 标志"
        exit 1
    fi

    make px4_sitl_default gazebo-classic
    # 从 Jinja 模板生成所有 SDF 文件（make 不总是生成它们）
    SITL_DIR=Tools/simulation/gazebo-classic/sitl_gazebo-classic
    find "$SITL_DIR/models" -name "*.sdf.jinja" | while read jinja; do
        out="${jinja%.jinja}"
        [ -f "$out" ] || python3 "$SITL_DIR/scripts/jinja_gen.py" "$jinja" "$SITL_DIR" --output-file "$out"
    done
    # iris_depth_camera airframe: reuse iris airframe (same flight dynamics)
    ATF_DIR=build/px4_sitl_default/etc/init.d-posix/airframes
    if [ ! -e "$ATF_DIR/1021_gazebo-classic_iris_depth_camera" ]; then
        ln -s 10015_gazebo-classic_iris "$ATF_DIR/1021_gazebo-classic_iris_depth_camera"
    fi
else
    echo "  PX4 已存在，跳过 clone。"
    # 如已存在但 SDF 未生成（例如之前构建中断），补全生成
    SITL_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    find "$SITL_DIR/models" -name "*.sdf.jinja" | while read jinja; do
        out="${jinja%.jinja}"
        [ -f "$out" ] || python3 "$SITL_DIR/scripts/jinja_gen.py" "$jinja" "$SITL_DIR" --output-file "$out"
    done
    # iris_depth_camera airframe: reuse iris airframe (same flight dynamics)
    ATF_DIR="$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes"
    if [ ! -e "$ATF_DIR/1021_gazebo-classic_iris_depth_camera" ]; then
        ln -s 10015_gazebo-classic_iris "$ATF_DIR/1021_gazebo-classic_iris_depth_camera"
    fi
fi

# ---- Install geographiclib datasets（MAVROS 需要） ----
echo "[5/7] 安装 MAVROS 地理数据集..."
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh 2>/dev/null || true

# ---- UAV 模型 ----
echo "[6/7] 设置 Gazebo 模型路径..."
mkdir -p "$HOME/.gazebo/models"
# 设置环境变量（使用唯一标记避免重复追加）
SENTINEL="# PX4 + Gazebo 路径（由 setup_uav.sh 自动添加）"
if ! grep -qF "$SENTINEL" "$HOME/.bashrc" 2>/dev/null; then
    cat >> "$HOME/.bashrc" << 'BASHEOF'

# PX4 + Gazebo 路径（由 setup_uav.sh 自动添加）
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export GAZEBO_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
export GAZEBO_MODEL_PATH="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH"
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default" > /dev/null || true
BASHEOF
fi

# ---- 编译 ----
echo "[7/7] 编译 workspace..."
# 移除之前构建中可能屏蔽的包
rm -f "$WS_DIR/src/forest_avoidance_utils/CATKIN_IGNORE"
cd "$WS_DIR"
if [ -f devel/setup.bash ] && [ -d build ]; then
    echo "  已有构建产物，跳过编译（如需重编请删除 build/ 和 devel/ 后重跑）"
else
    catkin_make -j$(nproc --ignore=2) 2>&1 | tee "$WS_DIR/build.log" | tail -20
fi
# NOTE: sourcing here only affects this subshell; callers must re-source manually
source devel/setup.bash 2>/dev/null || true

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
