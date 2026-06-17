#!/usr/bin/env bash
set -e

UGV_WS=${UGV_WS:-"$HOME/Air-Ground-Collaborative-Manupulation"}
SETUP_FILE="$UGV_WS/devel/setup.bash"
RVIZ_BIN="/opt/ros/noetic/lib/rviz/rviz"

if [ ! -f "$SETUP_FILE" ]; then
  echo "[run_ugv_elevation_rviz] missing UGV setup: $SETUP_FILE" >&2
  exit 1
fi

if [ ! -x "$RVIZ_BIN" ]; then
  echo "[run_ugv_elevation_rviz] missing RViz binary: $RVIZ_BIN" >&2
  exit 1
fi

source "$SETUP_FILE"
exec "$RVIZ_BIN" "$@"
