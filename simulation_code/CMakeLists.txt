cmake_minimum_required(VERSION 2.8.3)
project(simulation_code)

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

add_executable(simudrive_2point src/simudrive_2point.cpp)
target_link_libraries(simudrive_2point ${catkin_LIBRARIES})
add_dependencies(simudrive_2point ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simudrive_goal src/simudrive_goal.cpp)
target_link_libraries(simudrive_goal ${catkin_LIBRARIES})
add_dependencies(simudrive_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})