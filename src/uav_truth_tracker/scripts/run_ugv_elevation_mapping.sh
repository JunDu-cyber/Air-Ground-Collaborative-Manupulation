#!/usr/bin/env bash
set -e

UGV_WS=${UGV_WS:-"$HOME/Air-Ground-Collaborative-Manupulation"}
SETUP_FILE="$UGV_WS/devel/setup.bash"
NODE_BIN="$UGV_WS/devel/lib/elevation_mapping/elevation_mapping"

if [ ! -f "$SETUP_FILE" ]; then
  echo "[run_ugv_elevation_mapping] missing UGV setup: $SETUP_FILE" >&2
  exit 1
fi

if [ ! -x "$NODE_BIN" ]; then
  echo "[run_ugv_elevation_mapping] missing elevation_mapping binary: $NODE_BIN" >&2
  exit 1
fi

source "$SETUP_FILE"
exec "$NODE_BIN" "$@"
