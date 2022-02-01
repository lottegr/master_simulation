#include "simulation_code/rows_and_sections.h"
#include "simulation_code/localize.h"


Localization::Localization()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Running simudrive node");
  auto ret = init();
  ROS_ASSERT(ret);
}

Localization::~Localization()
{
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool Localization::init()
{
  // initialize publishers
  localization_pub_ = nh_.advertise<simulation_code::Localization>("/localization", 10);

  // initialize subscribers
  odom_sub_ = nh_.subscribe("odom", 10, &Localization::odomMsgCallBack, this);
  pose_sub_ = nh_.subscribe("/amcl_pose", 10, &Localization::poseMsgCallBack, this);
    
  return true;
}



// subscribers 

void Localization::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	pose_odom_rot = atan2(siny, cosy) * RAD2DEG;
  pose_odom_pos_x = msg->pose.pose.position.x;
  pose_odom_pos_y = msg->pose.pose.position.y;
}


void Localization::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	pose_rot = atan2(siny, cosy) * RAD2DEG;
  pose_pos_x = msg->pose.pose.position.x;
  pose_pos_y = msg->pose.pose.position.y;
}



// publishers

void Localization::updateLocalization(double row_, const char* section_)
{
  simulation_code::Localization loc;

  loc.row = row_;
  loc.section = section_;

  localization_pub_.publish(loc);
}







/*******************************************************************************
* Localization function 
*******************************************************************************/

bool Localization::simulationLoop()
{
  static uint8_t find_row = 0;
  static uint8_t find_section = 0;

  static uint8_t row_ = 0;
  static const char* section_ = " ";


    switch(find_row)
    {
      case 0:
        // ROS_INFO_STREAM(pose_pos_y);
        if (abs(pose_pos_y) < error_)
        {
          find_row = 1;
          // ROS_INFO_STREAM("row1");
        }
        else if (abs(pose_pos_y - row2) < error_)
        {
          find_row = 2;
          // ROS_INFO_STREAM("row2");
        }
        else if (abs(pose_pos_y - row3) < error_)
        {
          find_row = 3;
          // ROS_INFO_STREAM("row3");
        }
        else if (abs(pose_pos_y - row4) < error_)
        {
          find_row = 4;
          // ROS_INFO_STREAM("row4");
        }
        break;

      case 1:
        row_ = 1;
        find_row = 0;
        break;

      case 2:
        row_ = 2;
        find_row = 0;
        break;

      case 3:
        row_ = 3;
        find_row = 0;
        break;

      case 4:
        row_ = 4;
        find_row = 0;
        break;
    }

    switch(find_section)
    {
      
      case 0:
        if (pose_pos_x < secA)
        {
          find_section = 1;
          // ROS_INFO_STREAM("secA");
        }
        else if (pose_pos_x < secB && pose_pos_x > secA)
        {
          find_section = 2;
          // ROS_INFO_STREAM("secB");
        }
        else if (pose_pos_x < secC && pose_pos_x > secB)
        {
          find_section = 3;
          // ROS_INFO_STREAM("secC");
        }
        else if (pose_pos_x > secD && pose_pos_x < secE)
        {
          find_section = 4;
          // ROS_INFO_STREAM("secD");
        }
        else if (pose_pos_x > secE && pose_pos_x < secF)
        {
          find_section = 5;
          // ROS_INFO_STREAM("secE");
        }
        else if (pose_pos_x > secF)
        {
          find_section = 6;
          // ROS_INFO_STREAM("secF");
        }
        else
        {
          find_section = 7;
        }
        break;
 
      case 1:
        section_ = "A";
        find_section = 0;
        break;

      case 2:
        section_ = "B";
        find_section = 0;
        break;

      case 3:
        section_ = "C";
        find_section = 0;
        break;

      case 4:
        section_ = "D";
        find_section = 0;
        break;

      case 5:
        section_ = "E";
        find_section = 0;
        break;

      case 6:
        section_ = "F";
        find_section = 0;
        break;

      case 7:
        section_ = " ";
        find_section = 0;
        break;
    }

    updateLocalization(row_, section_);

  return true;
}







/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "localize");
  Localization localize;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    localize.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}