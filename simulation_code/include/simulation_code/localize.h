#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <simulation_code/Localization.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)


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
  ros::Publisher loc_pub_;

  // ROS Topic Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;

  // Variables
  double pose_rot;
  double pose_pos_x;
  double pose_pos_y;
  double pose_odom_rot;
  double pose_odom_pos_x;
  double pose_odom_pos_y;
  
  // Functions
  // void updateLocalization(double row_, const char* section_);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  // void poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

};