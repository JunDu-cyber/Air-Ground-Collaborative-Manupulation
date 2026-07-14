#!/usr/bin/env python3
"""Sample the live global costmap at every mission mine (manual diagnostic)."""

import math

import rospy
from nav_msgs.msg import OccupancyGrid

from mobile_manipulator.msg import MineMission, MineMissionEntry


def main():
    rospy.init_node("check_mine_hazard_runtime", anonymous=True)
    mission = rospy.wait_for_message("/mine_mission/status", MineMission, timeout=10.0)
    costmap = rospy.wait_for_message(
        "/move_base/global_costmap/costmap", OccupancyGrid, timeout=10.0
    )
    info = costmap.info
    print(
        "global_costmap frame={} size={}x{} resolution={:.3f} origin=({:.2f},{:.2f})".format(
            costmap.header.frame_id,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
        )
    )
    for entry in mission.entries:
        x = entry.mine_pose.pose.position.x
        y = entry.mine_pose.pose.position.y
        mx = int(math.floor((x - info.origin.position.x) / info.resolution))
        my = int(math.floor((y - info.origin.position.y) / info.resolution))
        inside = 0 <= mx < info.width and 0 <= my < info.height
        cost = costmap.data[my * info.width + mx] if inside else None
        state = {
            MineMissionEntry.CLEARED: "CLEARED",
            MineMissionEntry.NAVIGATING: "NAVIGATING",
            MineMissionEntry.PENDING: "PENDING",
            MineMissionEntry.WAITING_ARM: "WAITING_ARM",
            MineMissionEntry.MANUAL_REQUIRED: "MANUAL_REQUIRED",
        }.get(entry.state, str(entry.state))
        print(
            "M{:03d} state={:<16} map=({:7.3f},{:7.3f}) cell=({:4d},{:4d}) cost={}".format(
                entry.id, state, x, y, mx, my, cost
            )
        )


if __name__ == "__main__":
    main()
