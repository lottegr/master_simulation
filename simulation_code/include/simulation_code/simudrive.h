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

#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

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





class SimulationDrive
{
 public:
  SimulationDrive();
  ~SimulationDrive();
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
  ros::Publisher goal_pub_;
  ros::Publisher init_pose_pub_;
  ros::Publisher env_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;

  ros::Subscriber pose_sub_;
  ros::Subscriber move_base_status_sub_;
  ros::Subscriber localization_sub_;
  ros::Subscriber environment_sub_;
  ros::Subscriber obstacle_sub_;
  ros::Subscriber cmd_vel_sub_;


  // ROS Action Clients
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> status_client_;

  // Variables
  double forward_dist_;
  double side_dist_;

  double scan_data_[4] = {10,10,10,10};

  double init_x;
  double init_y;
  double init_z;
  double pose_rot;
  double pose_pos_x;
  double pose_pos_y;
  double pose_odom_rot;
  double pose_odom_pos_x;
  double pose_odom_pos_y;
  double twist_odom_lin;
  double twist_odom_ang;
  double pose_goal_rot;
  double pose_goal_x;
  double pose_goal_y;
  double row_;
  std::string section_;
  std::string env_;
  bool obst_;
  double cmd_lin_;
  double cmd_ang_;

  bool drive_f = true;
  bool drive_b = false;
  bool first = true;
  int turns;
  int round = 1;
  int turn_step = 1;
  int i = 0;

  // Functions
  void updateCommandVelocity(double linear, double angular);
  void updateInitialPose(double pos_x, double pos_y, double rot_z);
  void updateNavigationGoal(double pos_x, double pos_y, double rot_z);
  void updateEnvironment(std::string environment);
  
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void localizationCallBack(const simulation_code::LocalizationConstPtr &msg);
  void environmentCallBack(const std_msgs::StringConstPtr &msg);
  void obstacleCallBack(const std_msgs::Bool::ConstPtr &msg);
  void commandVelocityCallBack(const geometry_msgs::TwistConstPtr &msg);

  void makeUturn(int round);

  // std::ofstream outFile;
  // std::vector<double> y1;
  // std::vector<double> y2;
  // std::vector<double> y3;
  // std::vector<double> y4;
  // std::vector<double> y5;
  // std::vector<double> l1;
  // std::vector<double> l3;
  // std::vector<double> l5;

  // std::vector<double> y1u;
  // std::vector<double> y2u;
  // std::vector<double> y3u;
  // std::vector<double> y4u;
  // std::vector<double> y5u;
  // std::vector<double> l1u;
  // std::vector<double> l3u;
  // std::vector<double> l5u;

  // std::vector<double> px;
  // std::vector<double> py;
  // std::vector<double> pa;


  // std::vector<double> twist;


  double output_ang_prev = 0;



  double prev = 0;




  ros::Publisher steer_pub_;
  void updateSteerAngle(double angle);

  std::vector<double> x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12;
  std::vector<double> y0, y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12;
  std::vector<double> z0, z1, z2, z3, z4, z5, z6, z7, z8, z9, z10, z11, z12;

  std::vector<std::vector<double>> x_vecs = {x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12};
  std::vector<std::vector<double>> y_vecs = {y0, y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12};
  std::vector<std::vector<double>> z_vecs = {z0, z1, z2, z3, z4, z5, z6, z7, z8, z9, z10, z11, z12};

  std::vector<std::string> x_names = {"x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12"};
  std::vector<std::string> y_names = {"y0", "y1", "y2", "y3", "y4", "y5", "y6", "y7", "y8", "y9", "y10", "y11", "y12"};
  std::vector<std::string> z_names = {"z0", "z1", "z2", "z3", "z4", "z5", "z6", "z7", "z8", "z9", "z10", "z11", "z12"};

};
#endif // SIMUDRIVE_H_
