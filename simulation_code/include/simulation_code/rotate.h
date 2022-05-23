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

#define lin_vel   0.5
#define ang_vel   1.5

#define dist_rows_y   1.5
#define dist_rows_x   4

#define forwards   0 
#define left      1
#define backwards  2
#define right     3

#define get_placement   0 
#define concrete_cross  1 
#define concrete_turn   2
#define rail_f          3
#define rail_b          4
#define rail_end_f      5
#define rail_end_b      6





class Rotate
{
 public:
  Rotate();
  ~Rotate();
  bool init();
  bool controlLoop();
  bool simulationLoop();
  void rotation(int row, double sensor, double target);


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
  void environmentCallBack(const std_msgs::StringConstPtr &msg);
  void obstacleCallBack(const std_msgs::Bool::ConstPtr &msg);

  void cross(int direction, int row);
  void makeUturn(int round);
  // void uturn();
  void write_to_file(std::vector<double> v, std::string name);


  // std::ofstream outFile;
  std::vector<double> y1;
  std::vector<double> y2;
  std::vector<double> y3;
  std::vector<double> y4;
  std::vector<double> y5;
  std::vector<double> l1;
  std::vector<double> l3;
  std::vector<double> l5;

  std::vector<double> y1u;
  std::vector<double> y2u;
  std::vector<double> y3u;
  std::vector<double> y4u;
  std::vector<double> y5u;
  std::vector<double> l1u;
  std::vector<double> l3u;
  std::vector<double> l5u;

  std::vector<double> px;
  std::vector<double> py;
  std::vector<double> pa;


  std::vector<double> twist;

};
