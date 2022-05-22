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
  EncoderData.msg
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

# ----------------------------------------------------------------------------------------------------- new after spezialization project

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

add_executable(exp_environment src/exp_environment.cpp)
target_link_libraries(exp_environment ${catkin_LIBRARIES})
add_dependencies(exp_environment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sensors src/sensors.cpp)
target_link_libraries(sensors ${catkin_LIBRARIES})
add_dependencies(sensors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_from_encoder src/odom_from_encoder.cpp)
target_link_libraries(odom_from_encoder ${catkin_LIBRARIES})
add_dependencies(odom_from_encoder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_switch src/odom_switch.cpp)
target_link_libraries(odom_switch ${catkin_LIBRARIES})
add_dependencies(odom_switch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(drive_straight src/exp_drive_straight.cpp)
target_link_libraries(drive_straight ${catkin_LIBRARIES})
add_dependencies(drive_straight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(u_turn src/exp_u_turn_v0.cpp)
target_link_libraries(u_turn ${catkin_LIBRARIES})
add_dependencies(u_turn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(exit_enter_turn src/exp_exit_enter_turn.cpp)
target_link_libraries(exit_enter_turn ${catkin_LIBRARIES})
add_dependencies(exit_enter_turn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(exp_test src/exp_test.cpp)
target_link_libraries(exp_test ${catkin_LIBRARIES})
add_dependencies(exp_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(subscribers include/simulation_code/functions/subscribers.cpp)
# target_link_libraries(subscribers ${catkin_LIBRARIES} -nostartfiles) 
# add_dependencies(subscribers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
