/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef SIMUDRIVE_H_
#define SIMUDRIVE_H_

#include <ros/ros.h>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>


#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>








#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define lin_vel   0.3
#define ang_vel   1.5

#define dist_rows_y   0.4
#define dist_rows_x   0.3

#define forward   0 
#define left      1
#define backward  2
#define right     3

#define get_placement 0 
#define betong_cross  1 
#define betong_turn   2
#define rail_f        3
#define rail_b        4
#define rail_end_f    5
#define rail_end_b    6





class SimulationDrive
{
 public:
  SimulationDrive();
  ~SimulationDrive();
  bool init();
  bool controlLoop();
  bool simulationLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time
  ros::Time begin;

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  ros::Publisher goal_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;

  ros::Subscriber pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber move_base_status_sub_;

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> status_client_;
  

  // Variables
  double forward_dist_;
  double side_dist_;

  double scan_data_[4] = {10,10,10,10};

  double pose_rot;
  double pose_pos_x;
  double pose_pos_y;
  double pose_odom_rot;
  double pose_odom_pos_x;
  double pose_odom_pos_y;
  double pose_goal_rot;
  double pose_goal_x;
  double pose_goal_y;

  bool enter_f = true;
  bool enter_b = false;
  bool rail = false;
  bool first = true;
  int turns;
  int round = 1;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void updateNavigationGoal(double pos_x, double pos_y, double rot_z);
  
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  
  void makeUturn(int round);
};
#endif // SIMUDRIVE_H_
