#include "simulation_code/rows_and_sections.h"
#include "simulation_code/odom_switch.h"


OdomSwitch::OdomSwitch()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Running odomSwitch node");
  auto ret = init();
  ROS_ASSERT(ret);
}

OdomSwitch::~OdomSwitch()
{
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool OdomSwitch::init()
{
  // initialize publishers
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

  // initialize subscribers
  odom_ground_sub_ = nh_.subscribe("odom_ground", 10, &OdomSwitch::odomGroundCallBack, this);
  odom_rail_sub_ = nh_.subscribe("odom_rail", 10, &OdomSwitch::odomRailCallBack, this);
  environment_sub_ = nh_.subscribe("environment", 10, &OdomSwitch::environmentCallBack, this);

    
  return true;
}



// subscribers 

void OdomSwitch::odomGroundCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  header_g = msg->header;
  child_frame_id_g = msg->child_frame_id;
  pose_g = msg->pose;
  twist_g = msg->twist;
}

void OdomSwitch::odomRailCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  header_r = msg->header;
  child_frame_id_r = msg->child_frame_id;
  pose_r = msg->pose;
  twist_r = msg->twist;
}

void OdomSwitch::environmentCallBack(const std_msgs::String::ConstPtr &msg)
{
  env_ = msg->data;
}



// publishers

void OdomSwitch::updateOdom(std_msgs::Header header, 
                            std::string child_frame_id,
                            geometry_msgs::PoseWithCovariance pose,
                            geometry_msgs::TwistWithCovariance twist)
{
  nav_msgs::Odometry odom;

  odom.pose = pose;

  odom_pub_.publish(odom);
}







/*******************************************************************************
* OdomSwitch function 
*******************************************************************************/

bool OdomSwitch::simulationLoop()
{
  static uint8_t find_env = 0;
  static uint8_t find_section = 0;

  static uint8_t row_ = 0;
  static const char* section_ = " ";


    switch(find_env)
    {
      case 0:
        if (env_.substr(0,4) == "rail")
        {
          find_env = 1;
          ROS_INFO_STREAM("rail");
        }
        else
        {
          find_env = 2;
          ROS_INFO_STREAM("ground");
        }
        break;

      case 1:
        updateOdom(header_r, child_frame_id_r, pose_r, twist_r);
        find_env = 0;
        break;

      case 2:
        updateOdom(header_g, child_frame_id_g, pose_g, twist_g);
        find_env = 0;
        break;
    }

    

  return true;
}







/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "odomSwitch");
  OdomSwitch odomSwitch;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    odomSwitch.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}