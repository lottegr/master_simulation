#include <ros/ros.h>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>


#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)


class OdomSwitch
{
 public:
  OdomSwitch();
  ~OdomSwitch();
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
  ros::Publisher odom_pub_;

  // ROS Topic Subscribers
  ros::Subscriber odom_ground_sub_;
  ros::Subscriber odom_rail_sub_;
  ros::Subscriber environment_sub_;

  // Variables
  std_msgs::Header header_g;
  std::string child_frame_id_g;
  geometry_msgs::PoseWithCovariance pose_g;
  geometry_msgs::TwistWithCovariance twist_g;

  std_msgs::Header header_r;
  std::string child_frame_id_r;
  geometry_msgs::PoseWithCovariance pose_r;
  geometry_msgs::TwistWithCovariance twist_r;

  std_msgs::Header header;
  std::string child_frame_id;
  geometry_msgs::PoseWithCovariance pose;
  geometry_msgs::TwistWithCovariance twist;

  std::string env_;
  
  // Functions
  void updateOdom(std_msgs::Header header, 
                  std::string child_frame_id,
                  geometry_msgs::PoseWithCovariance pose,
                  geometry_msgs::TwistWithCovariance twist);
  
  void odomGroundCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void odomRailCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void environmentCallBack(const std_msgs::StringConstPtr &msg);
  
};