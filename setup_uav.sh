#!/usr/bin/env bash
# ============================================================================
# Air-Ground UAV 仿真环境一键安装（含 PX4 SITL）
#
# 安装完成后队友即可启动 PX4 + MAVROS + EGO-Planner UAV 建图。
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR"
AIRGROUND_ENV_FILE="${AIRGROUND_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/airground/env.sh}"
if [ -z "${PX4_DIR:-}" ] && [ -r "$AIRGROUND_ENV_FILE" ]; then
    # Reuse the path selected by an earlier successful setup.  An explicit
    # PX4_DIR in the caller always has priority over this managed file.
    # shellcheck disable=SC1090
    source "$AIRGROUND_ENV_FILE"
fi
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
EGO_PLANNER_URL="https://github.com/ZJU-FAST-Lab/ego-planner.git"
EGO_PLANNER_COMMIT="${EGO_PLANNER_COMMIT:-bfda51284c8c1b476043255a8145ef925a3778a5}"
PX4_TAG="v1.14.3"
PX4_COMMIT="1dacb4cdef2d7145754fc788fa8dc482eed74b40"

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
    git gnome-terminal \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-gazebo-ros-pkgs ros-noetic-husky-simulator \
    ros-noetic-universal-robots ros-noetic-moveit ros-noetic-navigation \
    ros-noetic-laser-filters ros-noetic-velodyne-gazebo-plugins \
    ros-noetic-octomap-ros ros-noetic-octomap-msgs ros-noetic-octomap-server \
    ros-noetic-grid-map ros-noetic-grid-map-visualization \
    ros-noetic-mavros ros-noetic-mavros-extras \
    ros-noetic-robot-localization ros-noetic-move-base ros-noetic-amcl \
    ros-noetic-twist-mux ros-noetic-nodelet \
    ros-noetic-rviz ros-noetic-tf2-ros ros-noetic-rqt ros-noetic-rqt-reconfigure \
    python3-pip python3-catkin-tools python3-scipy python3-rosdep python3-vcstool \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

echo "  安装固定版本 Python 感知依赖..."
python3 -m pip install --user -r "$WS_DIR/requirements-runtime.txt"

# ---- pinned workspace sources ----
echo "[1b/7] 初始化固定版本源码依赖..."
cd "$WS_DIR"
if git -C "$WS_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    git submodule sync --recursive
    git submodule update --init --recursive
fi

# elevation_mapping itself is a project-specific vendored fork.  Only its
# three pinned ANYbotics helper repositories are imported here.
vcs import src --skip-existing < "$WS_DIR/src/elevation_mapping.repos"

REQUIRED_SOURCE_FILES=(
    "src/autonomous_exploration_development_environment/src/local_planner/package.xml"
    "src/direct_lidar_inertial_odometry/package.xml"
    "src/robot_body_filter/package.xml"
)
for required_file in "${REQUIRED_SOURCE_FILES[@]}"; do
    if [ ! -f "$WS_DIR/$required_file" ]; then
        echo "  错误: 缺少源码依赖 $required_file"
        echo "        请使用 git clone --recurse-submodules，或在仓库内执行："
        echo "        git submodule update --init --recursive"
        exit 1
    fi
done

# vcs --skip-existing deliberately preserves local directories, so verify that
# an old checkout did not silently defeat the revisions pinned in the repos file.
PINNED_HELPERS=(
    "message_logger:bd99bd663bc6029b454919ce6d628b25ce2468e7"
    "kindr:32800890d546e306f73c3ee3091fb6903452f925"
    "kindr_ros:8d60e3f8df5ddd8bcc58db3072edbb651f286b32"
)
for helper_entry in "${PINNED_HELPERS[@]}"; do
    helper_name="${helper_entry%%:*}"
    expected_commit="${helper_entry#*:}"
    helper_dir="$WS_DIR/src/$helper_name"
    actual_commit="$(git -C "$helper_dir" rev-parse HEAD 2>/dev/null || true)"
    if [ "$actual_commit" != "$expected_commit" ]; then
        echo "  错误: $helper_name 当前为 ${actual_commit:-非 Git 工作区}"
        echo "        需要固定提交 $expected_commit。"
        echo "        请备份并移走 $helper_dir 后重新运行本脚本。"
        exit 1
    fi
done

# ---- ego-planner ----
echo "[2/7] 克隆 EGO-Planner..."
cd "$WS_DIR/src"
if [ ! -d ego-planner ]; then
    git init ego-planner
    git -C ego-planner remote add origin "$EGO_PLANNER_URL"
    git -C ego-planner fetch --depth 1 origin "$EGO_PLANNER_COMMIT"
    git -C ego-planner checkout --detach FETCH_HEAD
else
    CURRENT_EGO_COMMIT="$(git -C ego-planner rev-parse HEAD 2>/dev/null || true)"
    if [ "$CURRENT_EGO_COMMIT" != "$EGO_PLANNER_COMMIT" ]; then
        echo "  错误: 已有 EGO-Planner 版本为 ${CURRENT_EGO_COMMIT:-unknown}"
        echo "        本提交固定为 $EGO_PLANNER_COMMIT，不能向未知版本应用项目补丁。"
        echo "        请备份并移走 src/ego-planner 后重新运行本脚本。"
        exit 1
    fi
fi

# Prepare the pinned egocentric navigation sources and their local patches.
# The final, complete workspace build remains step 7 below.
SETUP_NAV_SKIP_BUILD=1 bash "$WS_DIR/setup_nav.sh"

# ---- rosdep ----
echo "[3/7] 安装工作空间依赖 (rosdep)..."
cd "$WS_DIR"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y
# 应用自定义修改（地面滤波、A* 池自适应、森林仿真配置）
echo "  应用 Air-Ground 自定义补丁..."
cp "$WS_DIR/patches/grid_map.h" "$WS_DIR/src/ego-planner/src/planner/plan_env/include/plan_env/"
cp "$WS_DIR/patches/grid_map.cpp" "$WS_DIR/src/ego-planner/src/planner/plan_env/src/"
cp "$WS_DIR/patches/planner_manager.cpp" "$WS_DIR/src/ego-planner/src/planner/plan_manage/src/"
cp "$WS_DIR/patches/ego_replan_fsm.cpp" "$WS_DIR/src/ego-planner/src/planner/plan_manage/src/"
cp "$WS_DIR/patches/default.rviz" "$WS_DIR/src/ego-planner/src/planner/plan_manage/launch/"
cp "$WS_DIR/patches/forest_debug.rviz" "$WS_DIR/src/ego-planner/src/planner/plan_manage/launch/"
cp "$WS_DIR/patches/video_demo.rviz" "$WS_DIR/src/ego-planner/src/planner/plan_manage/launch/"
cp "$WS_DIR/patches/minimal.rviz" "$WS_DIR/src/ego-planner/src/planner/plan_manage/launch/"
cp "$WS_DIR/patches/run_forest_sim.launch" "$WS_DIR/src/ego-planner/src/planner/plan_manage/launch/"
cp -r "$WS_DIR/patches/scripts/"* "$WS_DIR/src/ego-planner/src/planner/plan_manage/scripts/" 2>/dev/null || true

# ---- PX4 SITL ----
echo "[4/7] 安装 PX4 SITL（首次约需 15-30 分钟）..."
if [ ! -d "$PX4_DIR" ]; then
    mkdir -p "$(dirname "$PX4_DIR")"
    git clone https://github.com/PX4/PX4-Autopilot.git --branch "$PX4_TAG" --depth 1 "$PX4_DIR"
fi
if [ ! -f "$PX4_DIR/Tools/setup/ubuntu.sh" ]; then
    echo "  错误: PX4_DIR 已存在但不是完整的 PX4 $PX4_TAG 源码目录: $PX4_DIR"
    exit 1
fi

cd "$PX4_DIR"
if ! git -C "$PX4_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "  错误: PX4_DIR 不是 Git 工作区，无法核对固定版本: $PX4_DIR"
    exit 1
fi
CURRENT_PX4_COMMIT="$(git -C "$PX4_DIR" rev-parse HEAD)"
if [ "$CURRENT_PX4_COMMIT" != "$PX4_COMMIT" ]; then
    echo "  错误: $PX4_DIR 当前为 $CURRENT_PX4_COMMIT"
    echo "        本提交固定为 PX4 $PX4_TAG ($PX4_COMMIT)。"
    echo "        为避免覆盖已有 PX4，请指定一个新目录后重试，例如："
    echo "        PX4_DIR=\"$HOME/PX4-Autopilot-$PX4_TAG\" bash \"$WS_DIR/setup_uav.sh\""
    exit 1
fi

# Persist the verified directory independently of ~/.bashrc.  Both this setup
# script and airground_takeoff.sh read the file, while an explicit PX4_DIR still
# overrides it.
mkdir -p "$(dirname "$AIRGROUND_ENV_FILE")"
{
    echo "# Generated by Air-Ground setup_uav.sh after PX4 revision verification."
    printf 'export PX4_DIR=%q\n' "$PX4_DIR"
} > "$AIRGROUND_ENV_FILE"
git submodule update --init --recursive --depth 1

if [ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
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
else
    echo "  PX4 SITL 已构建，跳过 make。"
fi

# 从 Jinja 模板生成所有 SDF 文件（make 不总是生成它们）
SITL_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
find "$SITL_DIR/models" -name "*.sdf.jinja" | while read -r jinja; do
    out="${jinja%.jinja}"
    [ -f "$out" ] || python3 "$SITL_DIR/scripts/jinja_gen.py" "$jinja" "$SITL_DIR" --output-file "$out"
done
# iris_depth_camera airframe: reuse iris airframe (same flight dynamics)
ATF_DIR="$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes"
if [ ! -e "$ATF_DIR/1021_gazebo-classic_iris_depth_camera" ]; then
    ln -s 10015_gazebo-classic_iris "$ATF_DIR/1021_gazebo-classic_iris_depth_camera"
fi

# ---- Install geographiclib datasets（MAVROS 需要） ----
echo "[5/7] 安装 MAVROS 地理数据集..."
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# ---- UAV 模型 ----
echo "[6/7] 设置 Gazebo 模型路径..."
mkdir -p "$HOME/.gazebo/models"
# 设置环境变量（使用唯一标记避免重复追加）。v2 块从受管理配置读取
# 已核验的 PX4_DIR，因此自定义安装目录在新终端中不会退回默认路径。
SENTINEL="# Air-Ground PX4 + Gazebo 路径 v2（由 setup_uav.sh 自动添加）"
if ! grep -qF "$SENTINEL" "$HOME/.bashrc" 2>/dev/null; then
    cat >> "$HOME/.bashrc" << 'BASHEOF'

# Air-Ground PX4 + Gazebo 路径 v2（由 setup_uav.sh 自动添加）
[ ! -r "${XDG_CONFIG_HOME:-$HOME/.config}/airground/env.sh" ] || \
    source "${XDG_CONFIG_HOME:-$HOME/.config}/airground/env.sh"
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export GAZEBO_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default" > /dev/null || true
BASHEOF
fi

# ---- 编译 ----
echo "[7/7] 编译 workspace..."
# 移除之前构建中可能屏蔽的包
rm -f "$WS_DIR/src/forest_avoidance_utils/CATKIN_IGNORE"
cd "$WS_DIR"
catkin config --extend "/opt/ros/$ROS_DISTRO"
mkdir -p "$WS_DIR/logs"
catkin build -j"$(nproc --ignore=2)" 2>&1 | tee "$WS_DIR/logs/setup_build.log" | tail -20
# NOTE: sourcing here only affects this subshell; callers must re-source manually
source devel/setup.bash 2>/dev/null || true

# ---- 输出目录 ----
mkdir -p "$HOME/pointcloud_maps"

echo ""
echo "============================================"
echo "  安装完成！"
echo "============================================"
echo ""
echo "=== 启动空地协同连续排雷演示 ==="
echo "  cd $WS_DIR && bash airground_takeoff.sh"
echo ""
echo "=== PCD → 高程图 ==="
echo "  source devel/setup.bash"
echo "  roslaunch mobile_manipulator pcd_to_elevation.launch \\"
echo "      pcd_file:=~/pointcloud_maps/uav_points_map_latest.pcd"
echo ""
