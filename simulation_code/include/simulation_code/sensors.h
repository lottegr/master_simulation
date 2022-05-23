#ifndef SIMUDRIVE_H_
#define SIMUDRIVE_H_

#include <ros/ros.h>
#include <math.h>

#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

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

#define forwards   0 
#define left      1
#define backwards  2
#define right     3




class Sensors
{
 public:
  Sensors();
  ~Sensors();
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
  // ros::Publisher env_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber localization_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_filter_sub_;

  // Variables
  double scan_data_[4] = {10,10,10,10};

  double pose_odom_rot;
  double pose_odom_pos_x;
  double pose_odom_pos_y;
  double pose_amcl_rot;
  double pose_amcl_pos_x;
  double pose_amcl_pos_y;
  double pose_imu_rot;
  double pose_imu_pos_xdotdot;
  double pose_imu_pos_ydotdot; 
  double pose_imu_pos_xdot;
  double pose_imu_pos_ydot;
  double pose_imu_pos_x;
  double pose_imu_pos_y;
  double pose_odom_f_rot;
  double pose_odom_f_pos_x;
  double pose_odom_f_pos_y;

  double pose_pos_x;
  double pose_pos_y;
  double pose_rot_z;
  double prev_pos_x = 0;
  double prev_pos_y = 0;
  double prev_rot_z = 0;

  double prev_odom_rot = 0;
  double prev_odom_pos_x = 0;
  double prev_odom_pos_y = 0;
  double prev_amcl_rot = 0;
  double prev_amcl_pos_x = 0;
  double prev_amcl_pos_y = 0;
  double prev_imu_rot = 0;

  // double twist_odom_lin;
  // double twist_odom_ang;


  // Functions
  // void updateEnvironment(std::string environment);
  
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void localizationCallBack(const simulation_code::LocalizationConstPtr &msg);
  void imuCallBack(const sensor_msgs::Imu::ConstPtr &msg);
  void odomFilterCallBack(const nav_msgs::Odometry::ConstPtr &msg);

  void write_to_file(std::vector<double> v, std::string name);

  std::vector<double> odom_x;
  std::vector<double> odom_y;
  std::vector<double> odom_rot;
  std::vector<double> amcl_x;
  std::vector<double> amcl_y;
  std::vector<double> amcl_rot;
  std::vector<double> imu_x;
  std::vector<double> imu_y;
  std::vector<double> imu_rot;
  std::vector<double> odom_f_x;
  std::vector<double> odom_f_y;
  std::vector<double> odom_f_rot;

  std::vector<double> pose_x;
  std::vector<double> pose_y;
  std::vector<double> pose_rot;

};
#endif // SIMUDRIVE_H_




