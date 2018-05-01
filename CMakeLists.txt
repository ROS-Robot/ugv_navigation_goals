cmake_minimum_required(VERSION 2.8.3)
project(rulah_navigation_goals)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  tf
  roscpp
  grid_map_msgs
  grid_map_ros
  grid_map_core
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rulah_navigation_goals
#  CATKIN_DEPENDS actionlib move_base_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rulah_navigation_goals src/rulah_navigation_goals.cpp src/functions.cpp src/bezier_curve.cpp src/utilities.cpp src/vehicle_dynamics.cpp)
target_link_libraries(rulah_navigation_goals ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
