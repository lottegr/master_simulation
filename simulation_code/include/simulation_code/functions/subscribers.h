#ifndef SUBSCRIBE_H_
#define SUBSCRIBE_H_

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
#define ang_vel   0.4

#define dist_rows_y   1.5
#define dist_rows_x   4

#define forward   0 
#define left      1
#define backward  2
#define right     3



class Subscribers
{
 public:
  Subscribers();
  ~Subscribers();
  bool init();
  bool controlLoop();
  bool simulationLoop();

  // Functions
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void localizationCallBack(const simulation_code::LocalizationConstPtr &msg);
  void environmentCallBack(const std_msgs::StringConstPtr &msg);
  void obstacleCallBack(const std_msgs::Bool::ConstPtr &msg);



 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time
  ros::Time begin;

  // ROS Topic Publishers

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber localization_sub_;
  ros::Subscriber environment_sub_;
  ros::Subscriber obstacle_sub_;

  // ROS Action Clients
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
  double twist_odom_lin;
  double twist_odom_ang;
  double pose_goal_rot;
  double pose_goal_x;
  double pose_goal_y;
  double row_;
  std::string section_;
  std::string env_;
  bool obst_;

};
#endif // SUBSCRIBE_H_
