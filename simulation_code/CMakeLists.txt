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
  message_generation
)

find_package(gazebo REQUIRED)




add_message_files(
  FILES
  Localization.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros message_runtime
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


add_executable(simudrive_feedback src/simudrive_feedback.cpp)
target_link_libraries(simudrive_feedback ${catkin_LIBRARIES})
add_dependencies(simudrive_feedback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(localize src/localize.cpp)
target_link_libraries(localize ${catkin_LIBRARIES})
add_dependencies(localize ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(coll_avoid src/coll_avoid.cpp)
target_link_libraries(coll_avoid ${catkin_LIBRARIES})
add_dependencies(coll_avoid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(environment src/environment.cpp)
target_link_libraries(environment ${catkin_LIBRARIES})
add_dependencies(environment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sensors src/sensors.cpp)
target_link_libraries(sensors ${catkin_LIBRARIES})
add_dependencies(sensors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})