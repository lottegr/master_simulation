cmake_minimum_required(VERSION 2.8.3)
project(simulation_test)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
)

find_package(gazebo REQUIRED)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros
  DEPENDS GAZEBO
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simudrive src/simudrive.cpp)
target_link_libraries(simudrive ${catkin_LIBRARIES})
add_dependencies(simudrive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
