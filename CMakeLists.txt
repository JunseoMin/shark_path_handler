cmake_minimum_required(VERSION 3.0.2)
project(shark_path_handler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  sensor_msgs
  morai_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES shark_path_handler
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf tf2 sensor_msgs tf2_ros morai_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_handling_node src/path_handling_node.cpp)

target_link_libraries(path_handling_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS path_handling_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)