#!/bin/bash

# ===== Validate ROS environment =====
if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "ROS Noetic not found."
    exit 1
fi

if [ ! -f ~/learning_ws/devel/setup.bash ]; then
    echo "learning workspace not built. Run: catkin build"
    exit 1
fi

source /opt/ros/noetic/setup.bash
source ~/learning_ws/devel/setup.bash

# Source prefix injected into every tmux window so each shell has ROS + workspace
ROS_SOURCE="source /opt/ros/noetic/setup.bash && source ~/learning_ws/devel/setup.bash"

# ===== LLM API key (read from caller's env; fail loudly if absent) =====
# Set LLM_API_KEY (or DEEPSEEK_API_KEY) before running this script.
# Optionally override: LLM_BASE_URL, LLM_MODEL
LLM_API_KEY="${LLM_API_KEY:-${DEEPSEEK_API_KEY}}"
if [ -z "$LLM_API_KEY" ]; then
    echo "[WARN] LLM_API_KEY and DEEPSEEK_API_KEY are both unset — agent will fail to connect."
fi

SESSION=robot_system

# ---------- Colors ----------
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

err() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# ---------- Check Dependencies ----------
command -v tmux >/dev/null 2>&1 || { err "tmux not installed."; exit 1; }
command -v roscore >/dev/null 2>&1 || { err "ROS not sourced."; exit 1; }

# ---------- Kill old session ----------
tmux kill-session -t $SESSION 2>/dev/null

# ---------- Start roscore ----------
log "Starting roscore..."
tmux new-session -d -s $SESSION -n core
tmux send-keys -t $SESSION:core "roscore" C-m

# ---------- Wait for ROS master ----------
log "Waiting for roscore..."
until rostopic list >/dev/null 2>&1
do
    sleep 1
done
log "roscore ready."

# ==========================================================
# Window 1 : Robot Spawn
# ==========================================================
tmux new-window -t $SESSION -n spawn
tmux send-keys -t $SESSION:spawn \
"$ROS_SOURCE && roslaunch mobile_manipulator spawn_robot.launch" C-m

sleep 3

# ==========================================================
# Window 2 : Navigation + SLAM
# ==========================================================
tmux new-window -t $SESSION -n nav
tmux send-keys -t $SESSION:nav \
"$ROS_SOURCE && roslaunch mobile_manipulator slam_toolbox_navigation.launch" C-m

sleep 2

# ==========================================================
# Window 3 : MoveIt RViz
# ==========================================================
tmux new-window -t $SESSION -n rviz
tmux send-keys -t $SESSION:rviz \
"$ROS_SOURCE && roslaunch husky_ur5_moveit_config moveit_rviz.launch" C-m

sleep 2

# ==========================================================
# Window 4 : Move Group
# ==========================================================
tmux new-window -t $SESSION -n movegroup
tmux send-keys -t $SESSION:movegroup \
"$ROS_SOURCE && roslaunch husky_ur5_moveit_config move_group.launch" C-m

sleep 2

# ==========================================================
# Window 5 : Perception (YOLO)
# ==========================================================
tmux new-window -t $SESSION -n yolo
tmux send-keys -t $SESSION:yolo \
"$ROS_SOURCE && while true; do rosrun mobile_manipulator trt_yolo_node; echo 'YOLO crashed, restarting...'; sleep 2; done" C-m

sleep 1

# ==========================================================
# Window 6 : Flask UI
# ==========================================================
tmux new-window -t $SESSION -n flask
FLASK_ENV="export LLM_API_KEY='${LLM_API_KEY}'"
[ -n "$LLM_BASE_URL" ] && FLASK_ENV="$FLASK_ENV; export LLM_BASE_URL='${LLM_BASE_URL}'"
[ -n "$LLM_MODEL"    ] && FLASK_ENV="$FLASK_ENV; export LLM_MODEL='${LLM_MODEL}'"
tmux send-keys -t $SESSION:flask \
"$ROS_SOURCE && $FLASK_ENV && while true; do rosrun mobile_manipulator flask_ui.py; echo 'Flask crashed, restarting...'; sleep 2; done" C-m
# ==========================================================
# Final Status
# ==========================================================
log "System startup complete."
log "Attach using:"
echo "tmux attach -t $SESSION"

log "Windows:"
echo "0 core"
echo "1 spawn"
echo "2 nav"
echo "3 rviz"
echo "4 movegroup"
echo "5 yolo"
echo "6 flask"

# Auto attach
tmux attach -t $SESSION