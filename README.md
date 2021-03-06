# File structure

|        **Folder**       | **Content**                                                                            |
|:-----------------------:|----------------------------------------------------------------------------------------|
| ROS2-ws                 | Files from the ROS galactic workspace, must be ran in ROS 2                            |
| gazebo                  | Map files and map from sketch generation files                                         |
| navigation              | The ROS navigation stack                                                               |
| rb1_description         | Robot model for Gazebo and RViz                                                        |
| robot_localization      | The ROS package robot_localization, addition of launch file for sensor fusion          |
| robot_localization_demo | A tutorial for using robot_localization, used launch file is inspired from this        |
| robot_navigation_params | Parameters for setting up navigation stack on robot                                    |
| simulation_code         | Mostly all new developed code, like navigation nodes, feedback, odometry generation ++ |
| _turtlebot_xxx_         | TurtleBot3 files, mostly kept as original                                              |
| ydlidar_ros             | Files for launching the LiDAR TG15                                                     |
| _other files_           | Files for making the different plots included in the thesis                            |























<!-- From specialization project: -->

<!-- | Folder                 | Subfolder                | File                         | Changes                                                        |
|------------------------|--------------------------|------------------------------|----------------------------------------------------------------|
| rb1_description        | meshes                   | All                          | New - Files recieved from Adigo                                |
|                        | urdf                     | rb1_description.gazebo.xacro | New - based on "turtlebot3_burger.gazebo.xacro"                |
|                        |                          | rb1_description.urdf.xacro   | Changes - based on "turtlebot3_burger.urdf.xacro"              |
| simulation_code        | All                      | All                          | New - based on "turtlebot3_drive.cpp" and "turtlebot3_drive.h" |
| turtlebot3_navigation  | launch                   | turtlebot3_navigation.launch | Changes: set default map                                       |
|                        | maps                     | temprail.yaml                | New map files generated with SLAM                              |
|                        |                          | temprail.pgm                 |                                                                |
| turtlebot3_simulations | turtlebot3_gazebo/launch | testrail.launch              | New - based on "turtlebot3_empty_world.launch"                 |
|                        | turtlebot3_gazebo/worlds | testrail.world               | New - generated from Gazebo                                    |

If no information is given, files are as original in the Turtlebot3 repositiries.  -->


<!-- # Commands

In every command window:
```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

Defining Turtlebots Burger robot as default:
```
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
``` -->
<!-- check with: $ echo $TURTLEBOT3_MODEL -->

<!-- -----------------------------------------------------------

Run GAZEBO:
```
roslaunch turtlebot3_gazebo testrail.launch
``` -->
<!-- roslaunch turtlebot3_gazebo "WORLDNAME".launch -->

<!-- Run KEYBOARD teleoperation:
```    
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
-----------------------------------------------------------

Run SLAM:
```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Save map file:
```
rosrun map_server map_saver -f ~/"NAME"
``` -->
<!-- $??rosrun map_server map_saver -f ~/gazebo/"NAME" -->
    
<!-- With the above command, "NAME".pgm and "NAME".yaml will be saved in the home folder ~/(/home/${username}).

-----------------------------------------------------------

Run NAVIGATION: 
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/"NAME".yaml
```
Default map-file is set to be the one created for the project -->
<!-- JOYSTICK: 
/home/lotte/catkin_ws/src/robot_gui_bridge/gui/gui.html
    roslaunch robot_gui_bridge websocket.launch -->


<!-- 
Run SIMULATION:
```
rosrun simulation_code simudrive
```
-----------------------------------------------------------

Record screen:
```
recordmydesktop --on-the-fly-encoding
```






# TurtleBot3
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="200">

Information about Turtlebot3 can be found in the README.md file in turtlebot3_simulations -->