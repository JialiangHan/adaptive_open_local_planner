cmake_minimum_required(VERSION 3.0.2)
project(adaptive_open_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  tf
  # obstacle_detector
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Waypoint.msg
  WaypointArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES adaptive_open_local_planner
 CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
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

## Declare a C++ library
add_library(helpers
  src/planner_helpers.cpp
  src/visualization_helpers.cpp
)

add_executable(${PROJECT_NAME}_node src/adaptive_open_local_planner_node.cpp)
target_link_libraries(${PROJECT_NAME}_node helpers)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

