cmake_minimum_required(VERSION 3.0.2)
project(terrain_cost_adapter)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  pcl_ros
  pcl_conversions
)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs grid_map_core grid_map_ros grid_map_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(gridmap_to_terrainmap src/gridmap_to_terrainmap.cpp)
target_link_libraries(gridmap_to_terrainmap
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
