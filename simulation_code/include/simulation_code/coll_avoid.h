#include <ros/ros.h>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <simulation_code/Localization.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)


class CollisionAvoid
{
 public:
  CollisionAvoid();
  ~CollisionAvoid();
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
  ros::Publisher obst_pub_;

  // ROS Topic Subscribers
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber localization_sub_;
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber environment_sub_;

  // Variables
  double pose_rot;
  double pose_pos_x;
  double pose_pos_y;
  double pose_odom_rot;
  double pose_odom_pos_x;
  double pose_odom_pos_y;
  double scan_data[28] = {10,10,10,10,10,10,10,
                          10,10,10,10,10,10,10,
                          10,10,10,10,10,10,10,
                          10,10,10,10,10,10,10};
  double scan_data_[4] = {10,10,10,10};

  double row_;
  std::string section_;
  std::string env_;
  double cmd_lin_;
  double cmd_ang_;
  bool stopped = false;
  
  // Functions
  void updateCommandVelocity(double linear, double angular);
  void updateObstacleBool(bool obst);


//   void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
//   void poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void commandVelocityCallBack(const geometry_msgs::TwistConstPtr &msg);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void localizationCallBack(const simulation_code::LocalizationConstPtr &msg);
  void environmentCallBack(const std_msgs::StringConstPtr &msg);

};