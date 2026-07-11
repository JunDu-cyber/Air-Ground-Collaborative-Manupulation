cmake_minimum_required(VERSION 3.0.2)
project(terrain_cost_adapter)

add_compile_options(-std=c++17)

# The workspace sets no CMAKE_BUILD_TYPE, so without this the roughness filter's
# per-cell loops compile with no optimisation at all -- worth ~40x here.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  pcl_ros
  pcl_conversions
  filters
  pluginlib
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES terrain_cost_adapter_filters
  CATKIN_DEPENDS roscpp sensor_msgs grid_map_core grid_map_ros grid_map_msgs pcl_ros pcl_conversions
                 filters pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(gridmap_to_terrainmap src/gridmap_to_terrainmap.cpp)
target_link_libraries(gridmap_to_terrainmap
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# grid_map filter-chain plugin, loaded in-process by elevation_mapping.
add_library(terrain_cost_adapter_filters src/roughness_filter.cpp)
target_link_libraries(terrain_cost_adapter_filters ${catkin_LIBRARIES})

install(TARGETS gridmap_to_terrainmap
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS terrain_cost_adapter_filters
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES filter_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
