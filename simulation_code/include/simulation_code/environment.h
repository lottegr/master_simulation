#ifndef SIMUDRIVE_H_
#define SIMUDRIVE_H_

#include <ros/ros.h>
#include <math.h>

#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <simulation_code/Localization.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <fstream>



#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define lin_vel   0.5
#define ang_vel   1.5

#define dist_rows_y   1.5
#define dist_rows_x   4

#define forward   0 
#define left      1
#define backward  2
#define right     3




class Environment
{
 public:
  Environment();
  ~Environment();
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
  ros::Publisher env_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;

  ros::Subscriber pose_sub_;
  ros::Subscriber move_base_status_sub_;
  ros::Subscriber localization_sub_;

  // ROS Action Clients
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> status_client_;

  // Variables
  double forward_dist_ = 0.7;
  double backward_dist_ = 1.2;
  double side_dist_ = 1;

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

  bool drive_f = true;
  bool drive_b = false;
  bool first = true;
  int turns;
  int round = 1;
  int turn_step = 1;

  // Functions
  void updateCommandVelocity(double linear, double angular);
  void updateInitialPose(double pos_x, double pos_y, double rot_z);
  void updateNavigationGoal(double pos_x, double pos_y, double rot_z);
  void updateEnvironment(std::string environment);
  
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void localizationCallBack(const simulation_code::LocalizationConstPtr &msg);

};
#endif // SIMUDRIVE_H_
