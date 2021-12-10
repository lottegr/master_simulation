in every command window:
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
<!-- export TURTLEBOT3_MODEL=burger    (default info on navigation 5.1) -->
    <!-- check with: $ echo $TURTLEBOT3_MODEL -->

-----------------------------------------------------------

GAZEBO:
$ roslaunch turtlebot3_gazebo "WORLDNAME".launch

SLAM:
$ roslaunch turtlebot3_slam turtlebot3_slam.launch
    save:
    $ rosrun map_server map_saver -f ~/"NAME"
    $ rosrun map_server map_saver -f ~/gazebo/"NAME"
    <!-- With the above command, "NAME".pgm and "NAME".yaml will be saved in the home folder ~/(/home/${username}). -->

    NAVIGATION: 
    $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/gazebo/"NAME".yaml

VALUES FROM SCAN: 
$ rosrun turtlebot3_gazebo turtlebot3_drive

-----------------------------------------------------------

KEYBOARD:
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

JOYSTICK: 
/home/lotte/catkin_ws/src/robot_gui_bridge/gui/gui.html
$ roslaunch robot_gui_bridge websocket.launch

-----------------------------------------------------------




***** rosrun turtlebot3_gazebo turtlebot3_drive

recordmydesktop --on-the-fly-encoding







# TurtleBot3
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">

[![kinetic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/kinetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/kinetic-devel)
[![melodic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/melodic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/melodic-devel)
[![noetic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/noetic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/noetic-devel)

[![dashing-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/dashing-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/dashing-devel)
[![foxy-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/foxy-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/foxy-devel)
[![galactic-devel Status](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/workflows/galactic-devel/badge.svg)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/galactic-devel)

## ROBOTIS e-Manual for TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)

## Wiki for turtlebot3_simulations Packages
- http://wiki.ros.org/turtlebot3_simulations (metapackage)
- http://wiki.ros.org/turtlebot3_fake
- http://wiki.ros.org/turtlebot3_gazebo

## Open Source related to TurtleBot3
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [turtlebot3_applications_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs)
- [turtlebot3_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications)
- [turtlebot3_autorace](https://github.com/ROBOTIS-GIT/turtlebot3_autorace)
- [turtlebot3_deliver](https://github.com/ROBOTIS-GIT/turtlebot3_deliver)
- [hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)
- [turtlebot3_manipulation](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git)
- [turtlebot3_manipulation_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git)
- [robotis_manipulator](https://github.com/ROBOTIS-GIT/robotis_manipulator)
- [open_manipulator_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_msgs)
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [open_manipulator_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_simulations)
- [open_manipulator_perceptions](https://github.com/ROBOTIS-GIT/open_manipulator_perceptions)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

## Documents and Videos related to TurtleBot3
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)
- [ROBOTIS e-Manual for OpenManipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator/)
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [Website for TurtleBot Series](http://www.turtlebot.com/)
- [e-Book for TurtleBot3](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51/)
- [Videos for TurtleBot3 ](https://www.youtube.com/playlist?list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU)
