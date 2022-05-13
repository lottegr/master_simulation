#include "simulation_code/coll_avoid.h"


CollisionAvoid::CollisionAvoid()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Running collision avoidance node");
  auto ret = init();
  ROS_ASSERT(ret);
}

CollisionAvoid::~CollisionAvoid()
{
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool CollisionAvoid::init()
{
  // initialize publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  obst_pub_ = nh_.advertise<std_msgs::Bool>("obstacle", 10);

  // initialize subscribers
  // odom_sub_ = nh_.subscribe("odom", 10, &CollisionAvoid::odomMsgCallBack, this);
  // pose_sub_ = nh_.subscribe("/amcl_pose", 10, &CollisionAvoid::poseMsgCallBack, this);
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &CollisionAvoid::commandVelocityCallBack, this);
  laser_scan_sub_  = nh_.subscribe("scan", 10, &CollisionAvoid::laserScanMsgCallBack, this);
  localization_sub_ = nh_.subscribe("localization", 10, &CollisionAvoid::localizationCallBack, this);
  environment_sub_ = nh_.subscribe("environment", 10, &CollisionAvoid::environmentCallBack, this);

  return true;
}



// subscribers 

// void CollisionAvoid::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
// {
//   double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
// 	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

// 	pose_odom_rot = atan2(siny, cosy) * RAD2DEG;
//   pose_odom_pos_x = msg->pose.pose.position.x;
//   pose_odom_pos_y = msg->pose.pose.position.y;
// }


// void CollisionAvoid::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
// {
//   double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
// 	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

// 	pose_rot = atan2(siny, cosy) * RAD2DEG;
//   pose_pos_x = msg->pose.pose.position.x;
//   pose_pos_y = msg->pose.pose.position.y;
// }


void CollisionAvoid::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
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
  ROS_INFO_STREAM(scan_data_[0]);
  ROS_INFO_STREAM(scan_data_[1]);
  ROS_INFO_STREAM(scan_data_[2]);
  ROS_INFO_STREAM(scan_data_[3]);
}


void CollisionAvoid::localizationCallBack(const simulation_code::Localization::ConstPtr &msg)
{
  row_ = msg->row;
  section_ = msg->section;
}


void CollisionAvoid::environmentCallBack(const std_msgs::String::ConstPtr &msg)
{
  env_ = msg->data;
}


void CollisionAvoid::commandVelocityCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd_lin_ = msg->linear.x;
  cmd_ang_ = msg->angular.z;
}







// publishers

void CollisionAvoid::updateCommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}


void CollisionAvoid::updateObstacleBool(bool obst)
{
  std_msgs::Bool bool_data;

  bool_data.data = obst;

  obst_pub_.publish(bool_data);
}







/*******************************************************************************
* CollisionAvoid function 
*******************************************************************************/

bool CollisionAvoid::simulationLoop()
{

  if ((scan_data_[0] < 0.5 && cmd_lin_ >= 0) || (scan_data_[2] < 1 && cmd_lin_ <= 0)) 
  {
  
    if (env_ != "end_f" && env_ != "end_b")
    {
      updateObstacleBool(true);

      ROS_WARN("Obstacle in path, waiting for 3 seconds.");
      // ROS_INFO_STREAM("Press c to continue.");
      ros::Duration(3).sleep();

      // std::string c;
      // std::cin >> c;

      // if (c == "c")
      // {
      //   updateObstacleBool(false);
      // }
    }
  }  
  else
  {
    updateObstacleBool(false);
    
    stopped = false;

  } 


  return true;
}







/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "localize");
  CollisionAvoid collavoid;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    collavoid.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}