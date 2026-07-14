cmake_minimum_required(VERSION 3.0.2)
project(mine_grasp_minimal)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  controller_manager_msgs
  cv_bridge
  diagnostic_msgs
  gazebo_grasp_plugin_ros
  gazebo_msgs
  geometry_msgs
  message_filters
  mobile_manipulator
  moveit_msgs
  nav_msgs
  rosgraph_msgs
  rospy
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/wrist_mine_localizer.py
  scripts/mine_grasp_executor.py
  scripts/grasp_test_manager.py
  scripts/controller_hold_test.py
  scripts/transport_pose_probe.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_motion_stability.py)
  catkin_add_nosetests(test/test_wrist_geometry.py)
  catkin_add_nosetests(test/test_action_result_race.py)
  catkin_add_nosetests(test/test_place_release_interlock.py)
  catkin_add_nosetests(test/test_stage_relocalization.py)
endif()
