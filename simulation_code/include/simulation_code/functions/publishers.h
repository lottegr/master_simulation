#ifndef PUBLISH_H_
#define PUBLISH_H_

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

#define forwards   0 
#define left      1
#define backwards  2
#define right     3



class Publishers
{
 public:
  Publishers();
  ~Publishers();
  bool init();
  bool controlLoop();
  bool simulationLoop();

  // Functions
  void updateCommandVelocity(double linear, double angular);
  void updateInitialPose(double pos_x, double pos_y, double rot_z);
  void updateNavigationGoal(double pos_x, double pos_y, double rot_z);
  void updateEnvironment(std::string environment);
  void updateLocalization(double row_, const char* section_);

//   obst
//   cmdvel? (collavoid)      - sub lol


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
  ros::Publisher loc_pub_;

  // ROS Topic Subscribers


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
#endif // PUBLISH_H_
