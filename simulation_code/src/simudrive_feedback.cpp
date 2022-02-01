#include "simulation_code/simudrive.h"
#include "MiniPID/MiniPID.h"
#include "MiniPID/MiniPID.cpp"
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
  updateCommandVelocity(0.0, 0.0);
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool SimulationDrive::init()
{
  // initialize variables
  forward_dist_ = 0.3;
  side_dist_    = 0.3;

  // initialize publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  env_pub_ = nh_.advertise<std_msgs::String>("environment", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &SimulationDrive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &SimulationDrive::odomMsgCallBack, this);
  pose_sub_ = nh_.subscribe("amcl_pose", 10, &SimulationDrive::poseMsgCallBack, this);
  localization_sub_ = nh_.subscribe("localization", 10, &SimulationDrive::localizationCallBack, this);
  
  // std::ofstream outFile;
  // outFile.open("src/plot_test/src/y1.txt");

  

  // y1;  


  return true;
}



// subscribers 

void SimulationDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	pose_odom_rot = atan2(siny, cosy) * RAD2DEG;
  pose_odom_pos_x = msg->pose.pose.position.x;
  pose_odom_pos_y = msg->pose.pose.position.y;
}

void SimulationDrive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
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

void SimulationDrive::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	pose_rot = atan2(siny, cosy) * RAD2DEG;
  pose_pos_x = msg->pose.pose.position.x;
  pose_pos_y = msg->pose.pose.position.y;
}

void SimulationDrive::localizationCallBack(const simulation_code::Localization::ConstPtr &msg)
{
  row_ = msg->row;
  section_ = msg->section;
}





// publishers

void SimulationDrive::updateCommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}


void SimulationDrive::updateIntialPose(double pos_x, double pos_y, double rot_z)
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





// u turn function

void SimulationDrive::makeUturn(int row)
{
  ros::Time now = ros::Time::now();

  MiniPID pid = MiniPID(1,0,0);

  double target1 = 0;
  double target2 = 90;
  double target3 = row*0.4;
  double target4;
  double target5;
  
  if (row%2 != 0) {
    target4 = 180;
    target5 = 0.4;
  } else {
    target4 = 0;
    target5 = 0.4;
  }

  double sensor_x = pose_odom_pos_x;
  double sensor_y = pose_odom_pos_y;
  double sensor_rot = pose_odom_rot;  



  if (turn_step == 1)
  {
    pid.setOutputLimits(-.3,.3);
    double sensor = sensor_x;
    double target = target1;

    if ( abs(sensor - target) > 0.01 )
    {
      double output = pid.getOutput(sensor,target);
      updateCommandVelocity(output,0); 
      y1.push_back(sensor);
      y1u.push_back(output);
    }
    else
    {
      updateCommandVelocity(0,0);
      write_to_file(y1,"y1");
      write_to_file(y1u,"y1u");
      ros::Duration(2).sleep();
      turn_step += 1;
    }
  } 
  
  else if (turn_step == 2)
  {
    pid.setOutputLimits(-1,1);
    double sensor = sensor_rot;
    double target = target2;

    if ( abs(sensor - target) > 0.1 )
    {
      double output = pid.getOutput(sensor,target);
      updateCommandVelocity(0,output); 
      y2.push_back(sensor);
      y2u.push_back(output);
    }
    else
    {
      updateCommandVelocity(0,0);
      write_to_file(y2,"y2");
      write_to_file(y2u,"y2u");
      ros::Duration(2).sleep();
      turn_step += 1;
    }
  }
 
  else if (turn_step == 3)
  {
    pid.setOutputLimits(-.3,.3);
    double sensor = sensor_y;
    double target = target3;

    if ( abs(sensor - target) > 0.01 )
    {
      double output = pid.getOutput(sensor,target);
      updateCommandVelocity(output,0); 
      y3.push_back(sensor);
      y3u.push_back(output);
    }
    else
    {
      updateCommandVelocity(0,0); 
      write_to_file(y3,"y3");
      write_to_file(y3u,"y3u");
      ros::Duration(2).sleep();
      turn_step += 1;
    }
  }

  else if (turn_step == 4)
  {
    pid.setOutputLimits(-1,1);
    double sensor;
    double target = target4;

    if (pose_odom_rot > 0)
    {
      sensor = sensor_rot;
    }
    else
    {
      sensor = 360 + sensor_rot;
    }

    if ( abs(sensor - target) > 0.1 )
    {
      double output = pid.getOutput(sensor,target);
      updateCommandVelocity(0,output); 
      y4.push_back(sensor);
      y4u.push_back(output);
    }
    else
    {
      updateCommandVelocity(0,0);
      write_to_file(y4,"y4");
      write_to_file(y4u,"y4u");
      ros::Duration(2).sleep();
      turn_step += 1;
    }
  }

  else if (turn_step == 5)
  {
    pid.setOutputLimits(-.3,.3);
    double sensor = sensor_x;
    double target = target5;

    if ( abs(sensor - target) > 0.01 )
    {
      double output = pid.getOutput(sensor,target);
      updateCommandVelocity(output,0); 
      y5.push_back(sensor);
      y5u.push_back(output);
    }
    else if (environment == "rail_f")
    {
      write_to_file(y5,"y5");
      write_to_file(y5u,"y5u");
    }
    
    else
    {
      updateCommandVelocity(0,0);
      ros::Duration(2).sleep();
    }
  } 
}











void SimulationDrive::write_to_file(std::vector<double> v, std::string name)
{

  name = name + ".txt";
  ROS_INFO_STREAM(name);


  std::ofstream outFile(name);
  if (!outFile.is_open()){
    ROS_INFO_STREAM("fuck");
  }
  else{
    for (const auto &e : v) outFile << e << "\n";
  }

  return;
}1


















/*******************************************************************************
* Simulating driving function 
*******************************************************************************/

bool SimulationDrive::simulationLoop()
{
  static uint8_t rb_status = 0;
  
  switch(rb_status)
  {
    case get_placement:
      if (scan_data_[forward] > forward_dist_ && scan_data_[backward] > forward_dist_)
      {
        if (scan_data_[left] < side_dist_ && scan_data_[right] < side_dist_)
        {
          
          if (drive_f && !drive_b)
          {
            rb_status = rail_f;
            updateEnvironment("rail_f");
          }
          else if (!drive_f && drive_b)
          {
            rb_status = rail_b;
            updateEnvironment("rail_b");
          }
        
        }
        else
        {
          if (drive_f && !drive_b) 
          {
            rb_status = concrete_turn;
            updateEnvironment("concrete_turn");
            drive_f = true;
          }
          else if (!drive_f && drive_b) 
          {
            rb_status = concrete_cross;
            updateEnvironment("concrete_cross");
            drive_b = true;
          }
        }
      }

      else {
        if (scan_data_[left] < side_dist_ && scan_data_[right] < side_dist_)
        {
          if (scan_data_[forward] < forward_dist_) 
          {
            rb_status = rail_end_f;
            updateEnvironment("end_f");
            drive_f = false;
            drive_b = true;
          }
          else if (scan_data_[backward] < forward_dist_)
          {
            rb_status = rail_end_b;
            updateEnvironment("end_b");
            drive_f = true;
            drive_b = false;
          }

        }
        else
        {
          ROS_WARN("where are you now?!");
        }
      }
        
      break;

// ----------------------------------------------------------------

    case concrete_turn:
      if (!first) 
      {
        if (round < 4)
        {
          begin = ros::Time::now();

          makeUturn(round);
          
          rb_status = get_placement;
          break;

        } 
        else
        {
          updateNavigationGoal(0, 0, 0);
          ROS_INFO_STREAM("Simulation ended");
        }
      }
      else
      {
        updateIntialPose(0,0,0);
        ros::Duration(2).sleep();
        updateCommandVelocity(lin_vel, 0.0);
        rb_status = get_placement;
      }
      break;


    case concrete_cross:
      updateCommandVelocity(-lin_vel, 0.0);
      rb_status = get_placement;
      break;

    case rail_f:
      updateCommandVelocity(lin_vel, 0.0);
      first = false;
      turns = 0;

      // ROS_INFO_STREAM(y1);

      rb_status = get_placement;
      break;

    case rail_b:
      updateCommandVelocity(-lin_vel, 0.0);
      rb_status = get_placement;
      break;

    case rail_end_f:
      updateCommandVelocity(-lin_vel, 0.0);
      rb_status = get_placement;
      break;

    case rail_end_b:
      updateCommandVelocity(lin_vel, 0.0);
      rb_status = get_placement;
      break;


    default:
      rb_status = get_placement;
      break;
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