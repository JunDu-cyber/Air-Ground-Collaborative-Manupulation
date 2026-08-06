#!/usr/bin/env bash
# Stop the processes started by airground_takeoff.sh without deleting outputs.
set -u

echo "[airground-stop] requesting ROS node shutdown ..."
if command -v rosnode >/dev/null 2>&1 && rosnode list >/dev/null 2>&1; then
  rosnode kill -a >/dev/null 2>&1 || true
fi

sleep 2

PATTERN='roslaunch|roscore|rosmaster|gzserver|gzclient|px4|mavros_node|px4_bridge.py|rviz|dlio_odom_node|ugv_target_tour|airground_anchor_latch|mine_seg_localizer_node.py|mine_map_fusion_node.py|mine_map_to_worldtarget.py|mine_survey_waypoints.py|uav_goal_arbiter.py|mine_camera_diagnostics.py|grasp_task.py|gpd_grasp_server.py|landmine_detector.py|mine_align.py'

pkill -TERM -f "$PATTERN" 2>/dev/null || true
sleep 2
pkill -KILL -f "$PATTERN" 2>/dev/null || true

echo "[airground-stop] stopped; generated maps and source files were preserved."
