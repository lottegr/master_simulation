#include "simulation_test/simudrive.h"

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
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool SimulationDrive::init()
{
  

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.6;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  goal_pub_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);



  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &SimulationDrive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &SimulationDrive::odomMsgCallBack, this);

  pose_sub_ = nh_.subscribe("/amcl_pose", 10, &SimulationDrive::poseMsgCallBack, this);
  


  return true;
}




// subscribers 

void SimulationDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void SimulationDrive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};
  // uint16_t scan_angle[3] = {0, 90, 360-90};

  for (int num = 0; num < 3; num++)
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

  // ROS_INFO_STREAM("front: " << scan_data_[0] << ", left: " << scan_data_[1] << ", right: " << scan_data_[2]);
}

void SimulationDrive::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	pose_ = atan2(siny, cosy) * (180/M_PI);
  ROS_INFO_STREAM(pose_);
}


// publishers

void SimulationDrive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}


void SimulationDrive::updateNavigationGoal(double pos_x, double pos_y, double rot_z)
{
  move_base_msgs::MoveBaseActionGoal posegoal;

  posegoal.goal.target_pose.pose.position.x = pos_x;
  posegoal.goal.target_pose.pose.position.y = pos_y;
  

  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  posegoal.goal.target_pose.pose.orientation.z = sin(rot_z * 0.5);
  posegoal.goal.target_pose.pose.orientation.w = cos(rot_z * 0.5);

  posegoal.goal.target_pose.header.stamp = ros::Time::now();
  posegoal.goal.target_pose.header.frame_id = "map";

  goal_pub_.publish(posegoal);
}






/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool SimulationDrive::controlLoop()
{
  static uint8_t turtlebot3_state_num = 0;

  switch(turtlebot3_state_num)
  {
    case GET_TB3_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist_)
      {
        if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        else
        {
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {
        prev_tb3_pose_ = tb3_pose_;
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      break;

    case TB3_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }

  return true;
}



/*******************************************************************************
* Simulating driving function 
*******************************************************************************/

bool SimulationDrive::simulationLoop()
{

  if (scan_data_[0] < 0.3){
    end_of_rail = true;
  } else {
    end_of_rail = false;
  }

  if (scan_data_[90] < 0.3){
    on_rail = true;
  } else {
    on_rail = false;
  }


  

  // updateNavigationGoal(0,0.5,90);
  // ROS_INFO_STREAM("test");


  // drive first rail
  
  if (end_of_rail) {
    updatecommandVelocity(0,0);
    // ROS_INFO_STREAM("vegg!");
  } else {
    updatecommandVelocity(0.2, 0);
  }
  
    
    // updatecommandVelocity(-0.2,0);
    // ROS_INFO_STREAM("yo");
    //   if (scan_data_[90] > 1){
    //     updatecommandVelocity(0,0);
    //   }

    
  // }


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
    
    // simudrive.controlLoop();
    simudrive.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();

    // 
    // ros::spinOnce();
    // loop_rate.sleep();
  }

  return 0;
}