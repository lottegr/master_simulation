#include "simulation_code/environment.h"


Environment::Environment()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Running environmental finding node");
  auto ret = init();
  ROS_ASSERT(ret);
}

Environment::~Environment()
{
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool Environment::init()
{
  // initialize publishers
  env_pub_ = nh_.advertise<std_msgs::String>("environment_exp", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Environment::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odometry/filtered_map", 10, &Environment::odomMsgCallBack, this);
  pose_sub_ = nh_.subscribe("/amcl_pose", 10, &Environment::poseMsgCallBack, this);
  

  return true;
}



// subscribers 

void Environment::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[4] = {1011, 1515, 5, 506};

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


void Environment::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	pose_odom_rot = atan2(siny, cosy) * RAD2DEG;
  pose_odom_pos_x = msg->pose.pose.position.x;
  pose_odom_pos_y = msg->pose.pose.position.y;
}


void Environment::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	pose_rot = atan2(siny, cosy) * RAD2DEG;
  pose_pos_x = msg->pose.pose.position.x;
  pose_pos_y = msg->pose.pose.position.y;
}



// publishers

void Environment::updateEnvironment(std::string environment)
{
  std_msgs::String env;
  env.data = environment;

  env_pub_.publish(env);
}





/*******************************************************************************
* Environment function 
*******************************************************************************/

bool Environment::simulationLoop()
{
  if (scan_data_[forward] > forward_dist_ && scan_data_[backward] > forward_dist_)  // not (obstacle ahead)
  {
    if (abs(pose_odom_pos_x) < 1 && pose_odom_pos_y < 0.4)                          // if (on rail)
    {
      if (drive_f && !drive_b)
      {
      updateEnvironment("rail_f");
      }
      else if (!drive_f && drive_b)
      {
      updateEnvironment("rail_b");
      }
    }
    else                                                                            // not (on rail)
    {
      if (drive_f && !drive_b) 
      {
      updateEnvironment("concrete_turn");
      drive_f = true;
      }
      else if (!drive_f && drive_b) 
      {
      updateEnvironment("concrete_cross");
      drive_b = true;
      }
    }
  }
  else                                                                              // if (obstacle ahead)                                                                                
  {
    if (abs(pose_odom_pos_x) < 1 && pose_odom_pos_y < 0.4)                          // if (on rail)
    {
      if (scan_data_[forward] < forward_dist_) 
      {
      updateEnvironment("end_f");
      drive_f = false;
      drive_b = true;
      }
      else if (scan_data_[backward] < forward_dist_)
      {
      updateEnvironment("end_b");
      drive_f = true;
      drive_b = false;
      }

    }
}



  return true;
}







/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "environment");
  Environment environment;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    environment.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}














