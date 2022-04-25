#include "simulation_code/functions/publishers.h"
// #include "simulation_code/simudrive.h"
#include "simulation_code/rows_and_sections.h"
#include "pid/pid.h"
#include "pid/pid.cpp"
#include <vector>
#include <fstream>
#include <iostream>
// #include "matplotlib-cpp/matplotlibcpp.h"

Publishers::Publishers()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  init();
}

Publishers::~Publishers()
{
  // updateCommandVelocity(0.0, 0.0);
//   ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool Publishers::init()
{
  // initialize variables
  forward_dist_ = 0.5;
  side_dist_    = 1;

  // initialize publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  env_pub_ = nh_.advertise<std_msgs::String>("environment", 10);
  loc_pub_ = nh_.advertise<simulation_code::Localization>("/localization", 10);

}



// publishers

void Publishers::updateCommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}


void Publishers::updateInitialPose(double pos_x, double pos_y, double rot_z)
{
  geometry_msgs::PoseWithCovarianceStamped init_pose;

  init_pose.pose.pose.position.x = pos_x;
  init_pose.pose.pose.position.y = pos_y;

  init_pose.pose.pose.orientation.z = sin(rot_z * DEG2RAD * 0.5);
  init_pose.pose.pose.orientation.w = cos(rot_z * DEG2RAD * 0.5);

  init_pose.header.stamp = ros::Time::now();
  init_pose.header.frame_id = "map";

  init_pose_pub_.publish(init_pose);
}


void Publishers::updateNavigationGoal(double pos_x, double pos_y, double rot_z)
{
  status_client_ ac("move_base",true);
  ac.waitForServer();  
  
  move_base_msgs::MoveBaseGoal posegoal;

  posegoal.target_pose.pose.position.x = pos_x;
  posegoal.target_pose.pose.position.y = pos_y;
  posegoal.target_pose.pose.orientation.z = sin(rot_z * DEG2RAD * 0.5);
  posegoal.target_pose.pose.orientation.w = cos(rot_z * DEG2RAD * 0.5);

  posegoal.target_pose.header.stamp = ros::Time::now();
  posegoal.target_pose.header.frame_id = "map";

  ROS_INFO("Sending goal");
  ac.sendGoal(posegoal);
  ac.waitForResult();
}

void Publishers::updateEnvironment(std::string environment)
{
  std_msgs::String env;
  env.data = environment;

  env_pub_.publish(env);
}

void Publishers::updateLocalization(double row_, const char* section_)
{
  simulation_code::Localization loc;

  loc.row = row_;
  loc.section = section_;

  loc_pub_.publish(loc);
}