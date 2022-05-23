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

void SimulationDrive::tfListener()
{
  tf::StampedTransform transform;

  try
  {
    tf_p_listener.lookupTransform("/map", "/p", ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) 
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
    
  pose_p_pos_x = transform.getOrigin().x();
  pose_p_pos_y = transform.getOrigin().y();

  return; 
}





/*******************************************************************************
* Simulating driving function 
*******************************************************************************/

bool SimulationDrive::simulationLoop()
{
  tfListener();
  if (!obst_)
  {

    FeedbackFunctions feedback;

    std::vector<double> sensor_goal = {pose_odom_pos_y,
                                        pose_odom_rot,
                                        pose_odom_pos_x,
                                        pose_odom_rot,
                                        pose_odom_pos_y,
                                        pose_odom_rot,
                                        pose_odom_pos_x,
                                        pose_odom_rot,
                                        pose_odom_pos_y,
                                        pose_odom_rot,
                                        pose_odom_pos_x,
                                        pose_odom_rot,
                                        pose_odom_pos_x};
    std::vector<double> sensor_line = {pose_p_pos_x,0,
                                        pose_p_pos_y,0,
                                        pose_p_pos_x,0,
                                        pose_p_pos_y,0,
                                        pose_p_pos_x,0,
                                        pose_p_pos_y,0,
                                        pose_p_pos_x};
    std::vector<double> target_goal = {3.5, 
                                        0, 
                                        2, 
                                        90, 
                                       4.5,
                                        0,
                                       1.25,
                                        90,
                                       2.5,
                                        0,
                                        0,
                                       -90,
                                        0};
    std::vector<double> target_line = {0, 0, 
                                       3.5, 0, 
                                       2, 0,
                                       4.5, 0,
                                       1.25, 0,
                                       2.5, 0,
                                       0};
    std::vector<double> dirs = {0, 0,
                                1, 0,
                                0, 0,
                                0, 0,
                                1, 0,
                                0, 0,
                                3};

    ROS_INFO_STREAM("Step: " << i << ", goal: " << target_goal[i] << ", measurement: " << sensor_goal[i]);

    if (i%2 == 0)   // drive straight
    {
      if (i==12)
      {
        // if (env_.substr(0,4) != "rail")
        // {
        //   double out = feedback.driveStraight(1,0,0,sensor_goal[i],target_goal[i],false,1);
        //   u_vecs[i].push_back(cmd_ang_);
        // }
        // else
        {
          updateCommandVelocity(lin_vel,0);
        }
      }
      else 
      {
        if ( abs(sensor_goal[i] - target_goal[i]) > 0.05 || cmd_lin_ > 0.005 )
        {
          feedback.driveStraight(dirs[i],sensor_goal[i],target_goal[i],0,0);
          u_vecs[i].push_back(cmd_lin_);
        }
        else
        {
            updateCommandVelocity(0,0);
            ros::Duration(2).sleep();
            i += 1;
        }
      }
      
    }       
    else            // rotate
    {
      if ( abs(sensor_goal[i] - target_goal[i]) > 0.1 || cmd_ang_ > 0.005)
      {
        feedback.rotate(sensor_goal[i],target_goal[i],false);
      }
      else
      {
          updateCommandVelocity(0,0);
          ros::Duration(2).sleep();
          i += 1;
      }
      u_vecs[i].push_back(cmd_ang_);
    }
  
    

    x_vecs[i].push_back(pose_odom_pos_x);
    y_vecs[i].push_back(pose_odom_pos_y);
    z_vecs[i].push_back(pose_odom_rot);

    feedback.write_to_file(x_vecs[i], x_names[i]); 
    feedback.write_to_file(y_vecs[i], y_names[i]); 
    feedback.write_to_file(z_vecs[i], z_names[i]); 
    feedback.write_to_file(u_vecs[i], u_names[i]); 
    

      // y1.push_back(pose_odom_pos_x);
      // y1u.push_back(out);

      // feedback.write_to_file(y1, name_s);
      // feedback.write_to_file(y1u, "input");
    

    
  }
  else
  {
    ROS_WARN("Obstacle in path, waiting for 3 seconds.");
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