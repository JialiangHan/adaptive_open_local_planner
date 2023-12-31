cmake_minimum_required(VERSION 3.0.2)
project(adaptive_open_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_CXX_FLAGS "-lstdc++fs -std=c++17")
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
  cmake_modules
  geometry_msgs
  interactive_markers
  message_generation
  nav_core
  nav_msgs
  roscpp
  std_msgs
  pluginlib
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  ackermann_msgs
)
FIND_PACKAGE(Glog REQUIRED)
FIND_PACKAGE(Gflags)
find_package(Python3 COMPONENTS Interpreter Development NumPy)
find_package(PythonLibs 3.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
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
 geometry_msgs
 interactive_markers
 message_runtime
 nav_core
 nav_msgs
 pluginlib
 roscpp
 std_msgs
       tf2
       tf2_ros
 visualization_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
include
${catkin_INCLUDE_DIRS}
${Python3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(helpers
  src/planner_helpers.cpp
  src/visualization_helpers.cpp
)

target_link_libraries(helpers
       glog::glog 
       ${catkin_LIBRARIES}
)
add_library(adaptive_open_local_planner
  src/adaptive_open_local_planner_ros.cpp
   src/obstacles.cpp
   src/planner_helpers.cpp
   src/visualization_helpers.cpp
   src/parameter_manager.cpp
   src/path_evaluator.cpp
   src/velocity_planner.cpp
   src/pso.cpp
   src/mpc.cpp
)


target_link_libraries(adaptive_open_local_planner
   ${EXTERNAL_LIBS}
   ${catkin_LIBRARIES}
       glog::glog 
       ${PYTHON_LIBRARIES}   
       OsqpEigen::OsqpEigen 
)

add_executable(test_mpc tests/test.cpp   src/obstacles.cpp
src/planner_helpers.cpp
src/visualization_helpers.cpp
src/parameter_manager.cpp
src/path_evaluator.cpp
src/velocity_planner.cpp
src/pso.cpp
src/mpc.cpp)
target_link_libraries(test_mpc
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
  glog::glog 
  ${PYTHON_LIBRARIES}   
  OsqpEigen::OsqpEigen 
   ) 

