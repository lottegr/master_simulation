#include "simulation_code/functions/feedback.h"
#include "simulation_code/functions/feedback.cpp"
#include "simulation_code/simudrive.h"
#include "MiniPID/MiniPID.h"
// #include "MiniPID/MiniPID.cpp"
#include "pid/pid.h"
#include "pid/pid.cpp"
#include <vector>
#include <fstream>
#include <iostream>

SimulationDrive::SimulationDrive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Running simudrive node");
  auto ret = init();
  ROS_ASSERT(ret);
}

SimulationDrive::~SimulationDrive()
{
  // updateCommandVelocity(0.0, 0.0);
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool SimulationDrive::init()
{
  // initialize variables
  forward_dist_ = 0.5;
  side_dist_    = 1;

  // initialize publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  // env_pub_ = nh_.advertise<std_msgs::String>("environment", 10);
  steer_pub_ = nh_.advertise<std_msgs::Float64>("steer", 10);


  // initialize subscribers
  // laser_scan_sub_  = nh_.subscribe("scan", 10, &SimulationDrive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odometry/filtered_map", 10, &SimulationDrive::odomMsgCallBack, this);
  // odom_sub_ = nh_.subscribe("odom", 10, &SimulationDrive::odomMsgCallBack, this);
  
  
  // pose_sub_ = nh_.subscribe("amcl_pose", 10, &SimulationDrive::poseMsgCallBack, this);
  localization_sub_ = nh_.subscribe("localization", 10, &SimulationDrive::localizationCallBack, this);
  environment_sub_ = nh_.subscribe("environment", 10, &SimulationDrive::environmentCallBack, this);
  obstacle_sub_ = nh_.subscribe("obstacle", 10, &SimulationDrive::obstacleCallBack, this);
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &SimulationDrive::commandVelocityCallBack, this);


  // ROS_INFO_STREAM("x: ");
  // std::cin >> init_x;
  // ROS_INFO_STREAM("y: "); 
  // std::cin >> init_y;
  // ROS_INFO_STREAM("z: ");
  // std::cin >> init_z;

  // updateInitialPose(init_x, init_y, init_z);

  return true;
}



// subscribers 

void SimulationDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
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

// void SimulationDrive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
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

// void SimulationDrive::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
// {
//   double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
// 	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

// 	pose_rot = atan2(siny, cosy) * RAD2DEG;
//   pose_pos_x = msg->pose.pose.position.x;
//   pose_pos_y = msg->pose.pose.position.y;
// }

void SimulationDrive::localizationCallBack(const simulation_code::Localization::ConstPtr &msg)
{
  row_ = msg->row;
  section_ = msg->section;
}

void SimulationDrive::environmentCallBack(const std_msgs::String::ConstPtr &msg)
{
  env_ = msg->data;
}

void SimulationDrive::obstacleCallBack(const std_msgs::Bool::ConstPtr &msg)
{
  obst_ = msg->data;
}

void SimulationDrive::commandVelocityCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd_lin_ = msg->linear.x;
  cmd_ang_ = msg->angular.z;
}




// publishers

void SimulationDrive::updateCommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}


void SimulationDrive::updateInitialPose(double pos_x, double pos_y, double rot_z)
{
  geometry_msgs::PoseWithCovarianceStamped init_pose;

  init_pose.pose.pose.position.x = pos_x;
  init_pose.pose.pose.position.y = pos_y;

  init_pose.pose.pose.orientation.z = sin(rot_z * DEG2RAD * 0.5);
  init_pose.pose.pose.orientation.w = cos(rot_z * DEG2RAD * 0.5);

  init_pose.header.stamp = ros::Time::now();
  init_pose.header.frame_id = "map";

  init_pose_pub_.publish(init_pose);
}


void SimulationDrive::updateNavigationGoal(double pos_x, double pos_y, double rot_z)
{
  status_client_ ac("move_base",true);
  ac.waitForServer();  
  
  move_base_msgs::MoveBaseGoal posegoal;

  posegoal.target_pose.pose.position.x = pos_x;
  posegoal.target_pose.pose.position.y = pos_y;
  posegoal.target_pose.pose.orientation.z = sin(rot_z * DEG2RAD * 0.5);
  posegoal.target_pose.pose.orientation.w = cos(rot_z * DEG2RAD * 0.5);

  posegoal.target_pose.header.stamp = ros::Time::now();
  posegoal.target_pose.header.frame_id = "map";

  ROS_INFO("Sending goal");
  ac.sendGoal(posegoal);
  ac.waitForResult();
}

void SimulationDrive::updateEnvironment(std::string environment)
{
  std_msgs::String env;
  env.data = environment;

  env_pub_.publish(env);
}

void SimulationDrive::updateSteerAngle(double angle)
{
  std_msgs::Float64 ang;

  ang.data = angle;

  steer_pub_.publish(ang);
}





void SimulationDrive::write_to_file(std::vector<double> v, std::string name)
{

  name = name + ".txt";
  
  std::ofstream outFile;
  outFile.open(name);
  if (!outFile.is_open()){
    ROS_INFO_STREAM("failed to open file");
  }
  else{
    for (const auto &e : v) outFile << e << "\n";
  }
  outFile.close();


  return;
}







/*******************************************************************************
* Simulating driving function 
*******************************************************************************/

bool SimulationDrive::simulationLoop()
{
  if (!obst_)
  {
    // forward
    FeedbackFunctions feedback;
    // feedback.driveStraight(1,0,0,pose_odom_pos_y,0,false,1);

    // backward
    // driveStraight(0,0,0,pose_odom_pos_y,0,false,-1);


  //-------------------- pid tuning -----------------------------
    
    // angle goal
    // double out = feedback.rotate(pose_odom_rot, 0, false);
    
    // position goal
    // double out = feedback.driveStraight(0,pose_odom_pos_x,0,pose_odom_pos_y,2);
  




    // line follow
    if (i == 0)   // drive straight
    {
      if ( (abs(pose_odom_pos_x - 0) > 0.05) || (cmd_lin_ > 0.01) )
      {
        feedback.driveStraight(0,pose_odom_pos_x,0,pose_odom_pos_y,2);
      }
      else
      {
        updateCommandVelocity(0,0);
        ros::Duration(1).sleep();
        i += 1;
      }
    }       
    else if (i == 1)            // rotate
    {
      if ( (abs(pose_odom_rot - (-90)) > 0.1) || (cmd_ang_ > 0.005) )
      {
        feedback.rotate(pose_odom_rot, -90, false);
      }
      else
      {
        updateCommandVelocity(0,0);
        ros::Duration(2).sleep();
        i += 1;
      }
    }
    else if (i == 2)            // rotate
    {
      if ( (abs(pose_odom_rot - (-90)) > 0.1) || (cmd_ang_ > 0.005) )
      {
        feedback.rotate(pose_odom_rot, -90, false);
      }
      else
      {
        updateCommandVelocity(0,0);
        ros::Duration(1).sleep();
        i += 1;
      }
    }
    else if (i == 3)
    {
      double out = feedback.driveStraight(1,0,0,pose_odom_rot,-90,false,1);

      y1.push_back(pose_odom_pos_x);
      y1u.push_back(out);
      // y2u.push_back(cmd_ang_);
      
      feedback.write_to_file(y1, "sensor");
      feedback.write_to_file(y1u, "input");
      // feedback.write_to_file(y2u, "input_cmd");
    }



    
  }
  else
  {
    updateCommandVelocity(0,0);
  }
  return true;
}





/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simudrive");
  SimulationDrive simudrive;

  ros::Rate loop_rate(125);

  

  while (ros::ok())
  {
    simudrive.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}