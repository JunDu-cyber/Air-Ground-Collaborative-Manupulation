cmake_minimum_required(VERSION 3.0.2)
project(grasp_mtc)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  gazebo_msgs
  geometry_msgs
  message_generation
  moveit_msgs
  rospy
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  visualization_msgs
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
  CATKIN_DEPENDS
    actionlib
    control_msgs
    gazebo_msgs
    geometry_msgs
    message_runtime
    moveit_msgs
    rospy
    sensor_msgs
    shape_msgs
    std_msgs
    std_srvs
    trajectory_msgs
    visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/landmine_detector.py
  scripts/analytic_grasp_server.py
  scripts/grasp_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
