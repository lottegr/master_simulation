#include <ros/ros.h>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <simulation_code/Localization.h>

#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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
#define concrete_cross  1 
#define concrete_turn   2
#define rail_f        3
#define rail_b        4
#define rail_end_f    5
#define rail_end_b    6


class Localization
{
 public:
  Localization();
  ~Localization();
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
  ros::Publisher init_pose_pub_;
  ros::Publisher localization_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber move_base_status_sub_;

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
  double pose_goal_rot;
  double pose_goal_x;
  double pose_goal_y;

  bool drive_f = true;
  bool drive_b = false;
  bool first = true;
  int turns;
  int round = 1;

  // Functions
  // void updateCommandVelocity(double linear, double angular);
//   void updateIntialPose(double pos_x, double pos_y, double rot_z);
//   void updateNavigationGoal(double pos_x, double pos_y, double rot_z);
  void updateLocalization(double row_, const char* section_);
  
  // void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

};