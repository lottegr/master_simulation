#include "simulation_code/simudrive.h"
#include "simulation_code/rows_and_sections.h"
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







// u turn function

void SimulationDrive::makeUturn(int row)
{
  // variables
  double target1 = 0;
  double target2 = 90;
  double target3 = row*dist_rows_y;
  double target4;
  double target5;
  bool over180;
  
  if (row%2 != 0) {
    target4 = 180;
    target5 = dist_rows_x;
  } else {
    target4 = 0;
    target5 = -dist_rows_x;
  }

  double sensor_x = pose_odom_pos_x;
  double sensor_y = pose_odom_pos_y;
  double sensor_rot = pose_odom_rot;  




  // u-turn loop
  if (turn_step == 1)
  {
    double sensor = sensor_x;
    double target = target1;
    double sensor_ = pose_odom_pos_y;
    double target_ = (row-1)*dist_rows_y;
    

    if ( abs(sensor - target) > 0.05 )
    {
      double output_lin, output_ang = driveStraight(row%2, sensor, target, sensor_, target_);
      // y1.push_back(sensor);
      // y1u.push_back(output_lin);
      // l1.push_back(sensor_);
      // l1u.push_back(output_ang);
    }
    else
    {
      updateCommandVelocity(0,0);
      // write_to_file(y1,"y1");
      // write_to_file(y1u,"y1u");
      // write_to_file(l1,"l1");
      // write_to_file(l1u,"l1u");
      ros::Duration(2).sleep();
      turn_step += 1;
    }

  } 
  

  else if (turn_step == 2)
  {
    if (row == 4){
      round += 1;
    }
    else {
      double sensor = sensor_rot;
      double target = target2;

      if (row%2 == 0){
        over180 = true;
      } else {
        over180 = false;
      }


      if ( abs(sensor - target) > 0.1 )
      {
        double output_ang = rotate(sensor,target,over180);
        // y2.push_back(sensor);
        // y2u.push_back(output_ang);
      }
      else
      {
        updateCommandVelocity(0,0);
        // write_to_file(y2,"y2");
        // write_to_file(y2u,"y2u");
        ros::Duration(2).sleep();
        turn_step += 1;
      }
    }
  }
 


  else if (turn_step == 3)
  {
    double sensor = sensor_y;
    double target = target3;
    double sensor_ = pose_odom_pos_x;
    double target_ = 0;

    if ( abs(sensor - target) > 0.02 )
    {
      double output_lin, output_ang = driveStraight(2, sensor, target, sensor_, target_);
      // updateCommandVelocity(output_lin, -output_ang); 
      // y3.push_back(sensor);
      // y3u.push_back(output_lin);
      // l3.push_back(sensor_);
      // l3u.push_back(output_ang);
    }
    else
    {
      updateCommandVelocity(0,0); 
      // write_to_file(y3,"y3");
      // write_to_file(y3u,"y3u");
      // write_to_file(l3,"l3");
      // write_to_file(l3u,"l3u");
      ros::Duration(2).sleep();
      turn_step += 1;
    }
  }





  else if (turn_step == 4)
  {
    double sensor = sensor_rot;
    double target = target4;

    if (row%2 == 1){
      over180 = true;
    } else {
      over180 = false;
    }

    if ( abs(sensor - target) > 0.1 )
    {
      double output_ang = rotate(sensor,target, over180);
      updateCommandVelocity(0, output_ang); 
      // y4.push_back(sensor);
      // y4u.push_back(output_ang);
    }
    else
    {
      updateCommandVelocity(0,0);
      // write_to_file(y4,"y4");
      // write_to_file(y4u,"y4u");
      ros::Duration(2).sleep();
      turn_step += 1;
      round += 1;
    }
  }





  else if (turn_step == 5)
  {
    double sensor = sensor_x;
    double target = target5;       
    double sensor_ = pose_odom_pos_y;
    double target_ = (row-1)*dist_rows_y;

    if ( abs(sensor - target) > 0.05 )
    {
      double output_lin, output_ang = driveStraight(row%2, sensor, target, sensor_, target_);               // bool false??
      // y5.push_back(sensor);
      // y5u.push_back(output_lin);
      // l5.push_back(sensor_);
      // l5u.push_back(output_ang);
    }
    // else if (environment == "rail_f")
    // {
    //   write_to_file(y5,"y5");
    //   write_to_file(y5u,"y5u");
    // }
    
    else
    {
      updateCommandVelocity(0,0);
      ros::Duration(2).sleep();
    }
  } 


  // ROS_INFO_STREAM(round);


  // plotting position

  // px.push_back(pose_odom_pos_x);
  // py.push_back(pose_odom_pos_y);
  // pa.push_back(pose_odom_rot);



  // write_to_file(px,"px");
  // write_to_file(py,"py");
  // write_to_file(pa,"pa");
  
  // write_to_file(y5,"y5");
  // write_to_file(y5u,"y5u");
  // write_to_file(l5,"l5");
  // write_to_file(l5u,"l5u");

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
    tf_p_listener.lookupTransform("/map", "/p" ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) 
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  
  pose_p_pos_x = -transform.getOrigin().x();
  pose_p_pos_y = transform.getOrigin().y();
  pose_p_rot;

  // ROS_INFO_STREAM(pose_p_pos_x);
  // ROS_INFO_STREAM("--------------------------------------------------------------------------------------------------");

  return; 
}









/*******************************************************************************
* Simulating driving function 
*******************************************************************************/

bool SimulationDrive::simulationLoop()
{
  static uint8_t rb_status = 0;
  tfListener();
  
  ROS_INFO_STREAM(pose_odom_pos_x << " --x-- " << pose_p_pos_x);
  ROS_INFO_STREAM(pose_odom_pos_y << " --y-- " << pose_p_pos_y);


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
          // cross(1,round);
          driveStraight(round%2,0,0,pose_odom_pos_y,(round-1)*dist_rows_y,false,1);
          rb_status = get_placement;
        }
        break;


      case concrete_cross:
        // cross(-1,round);
        driveStraight(round%2,0,0,pose_odom_pos_y,(round-1)*dist_rows_y,false,-1);
        rb_status = get_placement;
        break;

      case rail_f:    // ROS_INFO_STREAM(sensor_ << "   " << output);
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