#!/usr/bin/env python3
"""Standalone single-pick tester for the GPD grasp pipeline (no LLM agent, no YOLO).

Drives one execute_pick() against a target you specify, so you can validate the
GPD path in isolation. Two ways to give the target:

  # A) Explicit coordinates in the ur5_base_link frame:
  rosrun mobile_manipulator test_pick.py 0.70 0.0 0.10 0.06   # X Y Z [DIAMETER]

  # B) Click the object in RViz (uses the "Publish Point" tool -> /clicked_point):
  rosrun mobile_manipulator test_pick.py click [DIAMETER]

What it does:
  1. Brings up MasterControl (MoveIt + TF).
  2. Moves the arm to the forward "ready" scan pose so the wrist camera looks
     at the object.
  3. Builds a PoseStamped at the target and calls execute_pick() with an EMPTY
     target_class -> no YOLO re-detection is performed.

execute_pick() then asks GPD for a 6-DOF grasp on the cloud cropped around that
point and runs the pregrasp -> Cartesian-insert -> close -> retreat sequence.
If GPD returns nothing it falls back to the horizontal heuristic.

Requires running first (separate terminals):
    roslaunch mobile_manipulator spawn_outdoor_city.launch     # Gazebo + robot
    roslaunch husky_ur5_moveit_config move_group.launch        # MoveIt
    roslaunch husky_ur5_moveit_config moveit_rviz.launch       # (optional) RViz
    roslaunch gpd_ros husky_ur5_gpd.launch                     # GPD
"""

import sys
import rospy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2 transform)
from geometry_msgs.msg import PoseStamped, PointStamped
from mobile_manipulator.master_control import MasterControl


def _target_from_click(mc) -> tuple:
    """Wait for an RViz 'Publish Point' click and return it in ur5_base_link."""
    rospy.loginfo("[TEST] Click the object in RViz with the 'Publish Point' tool...")
    pt = rospy.wait_for_message("/clicked_point", PointStamped)
    if pt.header.frame_id and pt.header.frame_id != "ur5_base_link":
        pt.header.stamp = rospy.Time(0)
        pt = mc.tf_buffer.transform(pt, "ur5_base_link", rospy.Duration(3.0))
    return pt.point.x, pt.point.y, pt.point.z


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    rospy.init_node("test_pick", anonymous=True)
    mc = MasterControl()

    if sys.argv[1] == "click":
        diameter = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
        x, y, z = _target_from_click(mc)
    else:
        if len(sys.argv) < 4:
            print(__doc__)
            sys.exit(1)
        x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
        diameter = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0

    rospy.loginfo("[TEST] Moving arm to ready/scan pose so the camera sees the object...")
    mc._go_ready()
    rospy.sleep(1.0)

    target = PoseStamped()
    target.header.frame_id = "ur5_base_link"
    target.header.stamp = rospy.Time(0)
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = z
    target.pose.orientation.w = 1.0

    rospy.loginfo(f"[TEST] execute_pick at ur5_base_link ({x:.3f}, {y:.3f}, {z:.3f}), "
                  f"diameter={diameter*1000:.0f} mm")
    ok = mc.execute_pick(target, target_class="", obj_diameter=diameter)
    rospy.loginfo(f"[TEST] execute_pick returned: {ok}")


if __name__ == "__main__":
    main()
