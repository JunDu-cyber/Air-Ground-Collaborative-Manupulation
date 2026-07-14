cmake_minimum_required(VERSION 3.0.2)
project(gazebo_link_attacher)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros message_generation)
find_package(gazebo REQUIRED)

add_service_files(FILES Attach.srv)
generate_messages()

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros message_runtime
  LIBRARIES gazebo_link_attacher
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(gazebo_link_attacher src/link_attacher_plugin.cpp)
target_link_libraries(gazebo_link_attacher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_link_attacher ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS gazebo_link_attacher
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
