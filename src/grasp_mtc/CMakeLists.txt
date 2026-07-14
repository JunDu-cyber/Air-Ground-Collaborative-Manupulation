cmake_minimum_required(VERSION 3.0.2)
project(grasp_mtc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  GetGrasps.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/landmine_detector.py
  scripts/analytic_grasp_server.py
  scripts/grasp_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
