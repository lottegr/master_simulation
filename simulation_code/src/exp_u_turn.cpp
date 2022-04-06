#include "simulation_code/simudrive.h"
#include "simulation_code/exp_rows_and_sections.h"
#include "pid/pid.h"
#include "pid/pid.cpp"
#include <vector>
#include <fstream>
#include <iostream>
// #include "matplotlib-cpp/matplotlibcpp.h"

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

  // initialize subscribers
  // laser_scan_sub_  = nh_.subscribe("scan", 10, &SimulationDrive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &SimulationDrive::odomMsgCallBack, this);
  // pose_sub_ = nh_.subscribe("amcl_pose", 10, &SimulationDrive::poseMsgCallBack, this);
  localization_sub_ = nh_.subscribe("localization", 10, &SimulationDrive::localizationCallBack, this);
  environment_sub_ = nh_.subscribe("environment", 10, &SimulationDrive::environmentCallBack, this);
  obstacle_sub_ = nh_.subscribe("obstacle", 10, &SimulationDrive::obstacleCallBack, this);

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



double SimulationDrive::rotate(double sensor, double target, bool over180)
{
  PID pid_ang = PID(0.1,ang_vel,-ang_vel, 0.01, 0.005, 0.05);

  if (over180){
    if (pose_odom_rot > 0){
      sensor = sensor;
    } else {
      sensor = 360 + sensor;
    }
  }

  double output_ang = pid_ang.calculate(target, sensor);
  updateCommandVelocity(0, output_ang); 

  return output_ang;
}


double SimulationDrive::driveStraight(int dir, double sensor_lin, double target_lin, double sensor_ang, double target_ang, bool target_pos=true, int direction=1)
{
  if (target_pos)
  {
    PID pid_lin = PID(0.1,lin_vel,-lin_vel, 1, 0.05, 0.5);
    PID pid_ang = PID(0.1,ang_vel,-ang_vel, 1, 0.05, 0);         // droppe D-term ??
    
    double output_lin = pid_lin.calculate(target_lin,sensor_lin);
    double output_ang = pid_ang.calculate(target_ang,sensor_ang);
    if (dir == 1){
      updateCommandVelocity(output_lin, output_ang); 
    } else if (dir == 0) {
      updateCommandVelocity(-output_lin, -output_ang); 
    } else if (dir == 2) {
      updateCommandVelocity(output_lin, -output_ang);
    }

    return output_lin, output_ang;
  }

  else
  {
    PID pid = PID(0.1,ang_vel,-ang_vel, 1, 0.05, 0);
    double output = pid.calculate(target_ang,sensor_ang);

    if (dir == 1)
    {
      updateCommandVelocity(direction*lin_vel, direction*output);
    }
    else 
    {
      updateCommandVelocity(direction*lin_vel, -direction*output);
    }
  }

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
    
    // double initial_y = 2;
    std::vector<double> sensor_goal = {pose_odom_pos_x,
                                        pose_odom_rot,
                                        pose_odom_pos_y,
                                        pose_odom_rot,
                                        pose_odom_pos_x};
    std::vector<double> sensor_line = {pose_odom_pos_y,0,
                                        pose_odom_pos_x,0,
                                        pose_odom_pos_y};
    std::vector<double> target_goal = {move_x_1, 
                                        90, 
                                        row2, 
                                        180, 
                                        0};
    std::vector<double> target_line = {0, 0, 
                                        move_x_1, 0, 
                                        row2};
    std::vector<double> dirs = {1, 0,
                                2, 0,
                                0};

   
    ROS_INFO_STREAM(i);
    
    if (i%2 == 0)   // drive straight
    {
        if ( abs(sensor_goal[i] - target_goal[i]) > 0.05 )
        {
            driveStraight(dirs[i],sensor_goal[i],target_goal[i],sensor_line[i],target_line[i]);
        }
        else
        {
            updateCommandVelocity(0,0);
            ros::Duration(2).sleep();
            i += 1;
        }
    }       
    else            // rotate
    {
        if ( abs(sensor_goal[i] - target_goal[i]) > 0.1 )
        {
            if (i != 3)
            {
                rotate(sensor_goal[i],target_goal[i],false);
            }
            else
            {
                rotate(sensor_goal[i],target_goal[i],true);
            }
        }
        else
        {
            updateCommandVelocity(0,0);
            ros::Duration(2).sleep();
            i += 1;
        }
    }
  
    



    // driveStraight(round%2,0,0,pose_odom_pos_y,(round-1)*dist_rows_y,false,1);

    // rotate(pose_odom_rot,90,false);

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