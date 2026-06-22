cmake_minimum_required(VERSION 3.0.2)
project(forest_avoidance_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  cv_bridge
  mavros_msgs
  quadrotor_msgs
  octomap_server
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(depth_preprocess_node src/depth_preprocess_node.cpp)
target_link_libraries(depth_preprocess_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(stuck_recovery_node src/stuck_recovery_node.cpp)
target_link_libraries(stuck_recovery_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/global_path_planner.py
  scripts/goal_elevator.py
  scripts/trajectory_logger.py
  scripts/analyze_trajectory.py
  scripts/octomap_manager.py
  scripts/pointcloud_to_pointcloud2_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
