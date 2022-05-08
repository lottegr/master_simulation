#include "simulation_code/functions/feedback.h"
#include "pid/pid.h"
#include "pid/pid.cpp"
#include <vector>
#include <fstream>
#include <iostream>
// #include "matplotlib-cpp/matplotlibcpp.h"

FeedbackFunctions::FeedbackFunctions()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  auto ret = init();
  ROS_ASSERT(ret);
}

FeedbackFunctions::~FeedbackFunctions()
{
  // updateCommandVelocity(0.0, 0.0);
//   ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool FeedbackFunctions::init()
{
  // initialize variables
  forward_dist_ = 0.5;
  side_dist_    = 1;

  // initialize publishers
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // initialize subscribers
  odom_sub_ = nh_.subscribe("odom", 10, &FeedbackFunctions::odomMsgCallBack, this);
  // pose_sub_ = nh_.subscribe("amcl_pose", 10, &FeedbackFunctions::poseMsgCallBack, this);

  return true;
}



// subscribers 

void FeedbackFunctions::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
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

// void FeedbackFunctions::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
// {
//   double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
// 	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

// 	pose_rot = atan2(siny, cosy) * RAD2DEG;
//   pose_pos_x = msg->pose.pose.position.x;
//   pose_pos_y = msg->pose.pose.position.y;
// }





// publishers

void FeedbackFunctions::updateCommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}





double FeedbackFunctions::rotate(double sensor, double target, bool over180)
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



double FeedbackFunctions::driveStraight(int dir, double sensor_lin, double target_lin, double sensor_ang, double target_ang, bool target_pos=true, int direction=1)
{
  if (target_pos)
  {
    PID pid_lin = PID(0.1,lin_vel,-lin_vel, 1, 0.05, 0.5);
    PID pid_ang = PID(0.1,ang_vel,-ang_vel, 1, 0.05, 0);         // droppe D-term ??
    
    double output_lin = pid_lin.calculate(target_lin,sensor_lin);
    // double output_ang = pid_ang.calculate(target_ang,sensor_ang);
    double output_ang = 0;
    if (dir == 1){
      updateCommandVelocity(output_lin, output_ang); 
    } else if (dir == 0) {
      updateCommandVelocity(-output_lin, -output_ang); 
    } else if (dir == 2) {
      updateCommandVelocity(output_lin, -output_ang);
    } else if (dir == 3) {
      updateCommandVelocity(-output_lin, output_ang);
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

