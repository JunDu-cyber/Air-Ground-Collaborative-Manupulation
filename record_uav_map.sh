#!/usr/bin/env bash
# ============================================================================
# 一键 UAV 点云录制：将 UAV LiDAR 点云保存为 PCD 文件。
#
# 前置条件：UAV 已在 ego_ws 中飞行（PX4 + MAVROS + EGO-Planner 已启动）。
#          确认 LiDAR topic（默认 /uav0/velodyne_points_raw）有数据。
#
# 用法：
#   cd ~/Air-Ground-Collaborative-Manupulation
#   bash record_uav_map.sh
#
# 输出：
#   ~/pointcloud_maps/uav_points_map_latest.pcd       （始终最新）
#   ~/pointcloud_maps/uav_points_map_YYYYMMDD_HHMMSS.pcd （带时间戳）
#
# 停止：Ctrl-C 退出，会自动保存最后一版 PCD。
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== UAV 点云录制 ==="
echo "输出目录: $HOME/pointcloud_maps/"
mkdir -p "$HOME/pointcloud_maps"

roslaunch mobile_manipulator uav_record_pointcloud.launch "$@"
