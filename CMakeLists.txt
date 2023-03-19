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
  base_local_planner
  costmap_2d
  costmap_converter
  cmake_modules
  geometry_msgs
  interactive_markers
  message_generation
  nav_core
  nav_msgs
  mbf_costmap_core
  mbf_msgs
  roscpp
  std_msgs
  pluginlib
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES

# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
 INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
 LIBRARIES teb_local_planner ${EXTERNAL_LIBS}
 CATKIN_DEPENDS
 base_local_planner
 costmap_2d
 costmap_converter
 geometry_msgs
 interactive_markers
 message_runtime
 nav_core
 nav_msgs
 pluginlib
 roscpp
 mbf_costmap_core
 std_msgs
       tf2
       tf2_ros
 visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(helpers
  src/planner_helpers.cpp
  src/visualization_helpers.cpp
)
add_library(adaptive_open_local_planner
  src/adaptive_open_local_planner_ros.cpp
   src/obstacles.cpp
   src/planner_helpers.cpp
   src/visualization_helpers.cpp
)


# Generate messages before compiling the lib
add_dependencies(adaptive_open_local_planner ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(adaptive_open_local_planner
   ${EXTERNAL_LIBS}
   ${catkin_LIBRARIES}
)

## Build additional executables

# add_executable(test_optim_node src/test_optim_node.cpp)

# target_link_libraries(test_optim_node
# adaptive_open_local_planner
#    ${EXTERNAL_LIBS}
#    ${catkin_LIBRARIES}
# )

