#include "simulation_code/functions/subscribers.h"
#include "simulation_code/simudrive.h"
#include "simulation_code/rows_and_sections.h"
#include <vector>
#include <fstream>
#include <iostream>
// #include "matplotlib-cpp/matplotlibcpp.h"

Subscribers::Subscribers()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Running subs node");
  init();
//   ROS_ASSERT(ret);
}

Subscribers::~Subscribers()
{
  // updateCommandVelocity(0.0, 0.0);
//   ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool Subscribers::init()
{
  // initialize variables
  forward_dist_ = 0.5;
  side_dist_    = 1;

  // initialize subscribers
//   laser_scan_sub_  = nh_.subscribe("scan", 10, &Subscribers::laserScanMsgCallBack, this);
//   odom_sub_ = nh_.subscribe("odom", 10, &Subscribers::odomMsgCallBack, this);
//   pose_sub_ = nh_.subscribe("amcl_pose", 10, &Subscribers::poseMsgCallBack, this);
//   localization_sub_ = nh_.subscribe("localization", 10, &Subscribers::localizationCallBack, this);
//   environment_sub_ = nh_.subscribe("environment", 10, &Subscribers::environmentCallBack, this);
//   obstacle_sub_ = nh_.subscribe("obstacle", 10, &Subscribers::obstacleCallBack, this);

  return true;
}



// subscribers 

void Subscribers::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  // pose
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

  pose_odom_rot = atan2(siny, cosy) * RAD2DEG;
  pose_odom_pos_x = msg->pose.pose.position.x;
  pose_odom_pos_y = msg->pose.pose.position.y;

  // twist
  twist_odom_lin = msg->twist.twist.linear.x;
  twist_odom_ang = msg->twist.twist.angular.z;
}

void Subscribers::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[4] = {0, 90, 180, 270};

  for (int num = 0; num < 4; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Subscribers::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

  pose_rot = atan2(siny, cosy) * RAD2DEG;
  pose_pos_x = msg->pose.pose.position.x;
  pose_pos_y = msg->pose.pose.position.y;
}

void Subscribers::localizationCallBack(const simulation_code::Localization::ConstPtr &msg)
{
  row_ = msg->row;
  section_ = msg->section;
}

void Subscribers::environmentCallBack(const std_msgs::String::ConstPtr &msg)
{
  env_ = msg->data;
}

void Subscribers::obstacleCallBack(const std_msgs::Bool::ConstPtr &msg)
{
  obst_ = msg->data;
}

