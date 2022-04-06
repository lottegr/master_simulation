#include "simulation_code/rotate.h"
#include "MiniPID/MiniPID.h"
#include "MiniPID/MiniPID.cpp"
#include <vector>
#include <fstream>
#include <iostream>
// #include "matplotlib-cpp/matplotlibcpp.h"

Rotate::Rotate()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Running simudrive node");
  auto ret = init();
  ROS_ASSERT(ret);
}

Rotate::~Rotate()
{
  // updateCommandVelocity(0.0, 0.0);
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool Rotate::init()
{
  // initialize variables
  forward_dist_ = 0.5;
  side_dist_    = 1;

  // initialize publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  // env_pub_ = nh_.advertise<std_msgs::String>("environment", 10);

  // initialize subscribers
  // laser_scan_sub_  = nh_.subscribe("scan", 10, &Rotate::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Rotate::odomMsgCallBack, this);
  // pose_sub_ = nh_.subscribe("amcl_pose", 10, &Rotate::poseMsgCallBack, this);
  localization_sub_ = nh_.subscribe("localization", 10, &Rotate::localizationCallBack, this);
  environment_sub_ = nh_.subscribe("environment", 10, &Rotate::environmentCallBack, this);
  obstacle_sub_ = nh_.subscribe("obstacle", 10, &Rotate::obstacleCallBack, this);

//   ROS_INFO_STREAM("x: ");
//   std::cin >> init_x;
//   ROS_INFO_STREAM("y: "); 
//   std::cin >> init_y;
//   ROS_INFO_STREAM("z: ");
//   std::cin >> init_z;

//   updateInitialPose(init_x, init_y, init_z);

  return true;
}



// subscribers 

void Rotate::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
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

// void Rotate::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
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

// void Rotate::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
// {
//   double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
// 	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

// 	pose_rot = atan2(siny, cosy) * RAD2DEG;
//   pose_pos_x = msg->pose.pose.position.x;
//   pose_pos_y = msg->pose.pose.position.y;
// }

void Rotate::localizationCallBack(const simulation_code::Localization::ConstPtr &msg)
{
  row_ = msg->row;
  section_ = msg->section;
}

void Rotate::environmentCallBack(const std_msgs::String::ConstPtr &msg)
{
  env_ = msg->data;
}

void Rotate::obstacleCallBack(const std_msgs::Bool::ConstPtr &msg)
{
  obst_ = msg->data;
}




// publishers

void Rotate::updateCommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}


void Rotate::updateInitialPose(double pos_x, double pos_y, double rot_z)
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


void Rotate::updateNavigationGoal(double pos_x, double pos_y, double rot_z)
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

void Rotate::updateEnvironment(std::string environment)
{
  std_msgs::String env;
  env.data = environment;

  env_pub_.publish(env);
}







// u turn function

void Rotate::rotation(int row, double sensor, double target)
{
    MiniPID pid_ang = MiniPID(0.1,0.5,1);
    pid_ang.setOutputLimits(-ang_vel,ang_vel);

    if ( abs(sensor - target) > 0.1 )
    {
    double output_ang = pid_ang.getOutput(sensor,target);
    // double output_lin = pid_lin.getOutput(sensor_, target_);
    updateCommandVelocity(0, output_ang); 
    y2.push_back(sensor);
    y2u.push_back(output_ang);
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







void Rotate::write_to_file(std::vector<double> v, std::string name)
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

bool Rotate::simulationLoop()
{
  static uint8_t rb_status = 0;
  
  

  if (!obst_)
  {
    switch(rb_status)
    {
      case get_placement:
        if (env_ == "rail_f")
        {
          // ROS_INFO_STREAM("railf");
          rb_status = rail_f;
        }
        else if (env_ == "rail_b")
        {
          // ROS_INFO_STREAM("railb");
          rb_status = rail_b;
        }
        else if (env_ == "concrete_turn") 
        {
          // ROS_INFO_STREAM("turn");
          rb_status = concrete_turn;
        }
        else if (env_ == "concrete_cross") 
        {
          // ROS_INFO_STREAM("cross");
          rb_status = concrete_cross;
        }
        else if (env_ == "end_f") 
        {
          rb_status = rail_end_f;
        }
        else if (env_ == "end_b")
        {
          rb_status = rail_end_b;
        }

        break;

  // ---------------------------------------------------------------- 

      case concrete_turn:
        if (!first) 
        {
          if (round < 5)
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
          cross(1,round);
          rb_status = get_placement;
        }
        break;


      case concrete_cross:
        cross(-1,round);
        rb_status = get_placement;
        break;

      case rail_f:
        updateCommandVelocity(lin_vel, 0.0);
        first = false;
        turn_step = 1;
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
  ros::init(argc, argv, "rotate");
  Rotate rotate;

  ros::Rate loop_rate(125);

  

  while (ros::ok())
  {
    rotate.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}