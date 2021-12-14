# Commands

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
```
<!-- check with: $ echo $TURTLEBOT3_MODEL -->

-----------------------------------------------------------

Run GAZEBO:
```
roslaunch turtlebot3_gazebo "WORLDNAME".launch
```

Run KEYBOARD teleoperation:
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
```
<!-- $ rosrun map_server map_saver -f ~/gazebo/"NAME" -->
    
With the above command, "NAME".pgm and "NAME".yaml will be saved in the home folder ~/(/home/${username}).

-----------------------------------------------------------

Run NAVIGATION: 
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/"NAME".yaml
```

<!-- JOYSTICK: 
/home/lotte/catkin_ws/src/robot_gui_bridge/gui/gui.html
    roslaunch robot_gui_bridge websocket.launch -->


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

Information about Turtlebot3 can be found in the README.md file in turtlebot3_simulations