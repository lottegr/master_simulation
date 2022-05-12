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
#ifndef func_H
#define func_H

#include <ros/ros.h>
#include <math.h>

#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <simulation_code/Localization.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <fstream>



#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// simulations
// #define lin_vel   0.5
// #define ang_vel   1.5

// real
#define lin_vel   0.2
#define ang_vel   0.5

#define dist_rows_y   1.5
#define dist_rows_x   4

#define forward   0 
#define left      1
#define backward  2
#define right     3

#define get_placement   0 
#define concrete_cross  1 
#define concrete_turn   2
#define rail_f          3
#define rail_b          4
#define rail_end_f      5
#define rail_end_b      6



class FeedbackFunctions
{
 public:
  FeedbackFunctions();
  ~FeedbackFunctions();
  bool init();
  bool controlLoop();
  bool simulationLoop();

  // Functions
  double rotate(double sensor, double target, bool over180);
  double driveStraight(int dir, double sensor_lin, double target_lin, double sensor_ang, double target_ang, bool target_pos, int direction);
  void write_to_file(std::vector<double> v, std::string name);

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time
  ros::Time begin;

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber odom_sub_;
  // ros::Subscriber pose_sub_;

  // ROS Action Clients
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> status_client_;

  // Variables
  double forward_dist_;
  double side_dist_;

  double pose_rot;
  double pose_pos_x;
  double pose_pos_y;
  double pose_odom_rot;
  double pose_odom_pos_x;
  double pose_odom_pos_y;
  double twist_odom_lin;
  double twist_odom_ang;

  // Functions
  void updateCommandVelocity(double linear, double angular);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  // void poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);



  ros::Publisher steer_pub_;
  void updateSteerAngle(double angle);

  double prev = 0;



};

#endif