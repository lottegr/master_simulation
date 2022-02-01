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
  // updateCommandVelocity(0.0, 0.0);
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool Localization::init()
{
  // initialize variables
  forward_dist_ = 0.3;
  side_dist_    = 0.3;

  // initialize publishers
  // cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  // init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
  localization_pub_ = nh_.advertise<simulation_code::Localization>("/localization", 10);

  // initialize subscribers
  // laser_scan_sub_  = nh_.subscribe("scan", 10, &Localization::laserScanMsgCallBack, this);
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

// void Localization::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
// {
//   uint16_t scan_angle[4] = {0, 90, 180, 270};

//   for (int num = 0; num < 4; num++)
//   {
//     if (std::isinf(msg->ranges.at(scan_angle[num])))
//     {
//       scan_data_[num] = msg->range_max;
//     }
//     else
//     {
//       scan_data_[num] = msg->ranges.at(scan_angle[num]);
//     }
//   }
// }

void Localization::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	pose_rot = atan2(siny, cosy) * RAD2DEG;
  pose_pos_x = msg->pose.pose.position.x;
  pose_pos_y = msg->pose.pose.position.y;
}




// publishers

// void Localization::updateCommandVelocity(double linear, double angular)
// {
//   geometry_msgs::Twist cmd_vel;

//   cmd_vel.linear.x  = linear;
//   cmd_vel.angular.z = angular;

//   cmd_vel_pub_.publish(cmd_vel);
// }


// void Localization::updateIntialPose(double pos_x, double pos_y, double rot_z)
// {
//   geometry_msgs::PoseWithCovarianceStamped init_pose;

//   init_pose.pose.pose.position.x = pos_x;
//   init_pose.pose.pose.position.y = pos_y;

//   init_pose.pose.pose.orientation.z = sin(rot_z * DEG2RAD * 0.5);
//   init_pose.pose.pose.orientation.w = cos(rot_z * DEG2RAD * 0.5);

//   init_pose.header.stamp = ros::Time::now();
//   init_pose.header.frame_id = "map";

//   init_pose_pub_.publish(init_pose);
// }


// void Localization::updateNavigationGoal(double pos_x, double pos_y, double rot_z)
// {
//   status_client_ ac("move_base",true);
//   ac.waitForServer();  
  
//   move_base_msgs::MoveBaseGoal posegoal;

//   posegoal.target_pose.pose.position.x = pos_x;
//   posegoal.target_pose.pose.position.y = pos_y;
//   posegoal.target_pose.pose.orientation.z = sin(rot_z * DEG2RAD * 0.5);
//   posegoal.target_pose.pose.orientation.w = cos(rot_z * DEG2RAD * 0.5);

//   posegoal.target_pose.header.stamp = ros::Time::now();
//   posegoal.target_pose.header.frame_id = "map";

//   ROS_INFO("Sending goal");
//   ac.sendGoal(posegoal);
//   ac.waitForResult();
// }


void Localization::updateLocalization(double row_, const char* section_)
{
  simulation_code::Localization loc;

  loc.row = row_;
  loc.section = section_;

  localization_pub_.publish(loc);
}







/*******************************************************************************
* Simulating driving function 
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
          ROS_INFO_STREAM("row1");
        }
        else if (abs(pose_pos_y - row2) < error_)
        {
          find_row = 2;
          ROS_INFO_STREAM("row2");
        }
        else if (abs(pose_pos_y - row3) < error_)
        {
          find_row = 3;
          ROS_INFO_STREAM("row3");
        }
        else if (abs(pose_pos_y - row4) < error_)
        {
          find_row = 4;
          ROS_INFO_STREAM("row4");
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
          ROS_INFO_STREAM("secA");
        }
        else if (pose_pos_x < secB && pose_pos_x > secA)
        {
          find_section = 2;
          ROS_INFO_STREAM("secB");
        }
        else if (pose_pos_x < secC && pose_pos_x > secB)
        {
          find_section = 3;
          ROS_INFO_STREAM("secC");
        }
        else if (pose_pos_x > secD && pose_pos_x < secE)
        {
          find_section = 4;
          ROS_INFO_STREAM("secD");
        }
        else if (pose_pos_x > secE && pose_pos_x < secF)
        {
          find_section = 5;
          ROS_INFO_STREAM("secE");
        }
        else if (pose_pos_x > secF)
        {
          find_section = 6;
          ROS_INFO_STREAM("secF");
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