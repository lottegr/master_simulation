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
  forward_dist_ = 0.3;
  side_dist_    = 0.2;


  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  goal_pub_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);


  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &SimulationDrive::laserScanMsgCallBack, this);
  // odom_sub_ = nh_.subscribe("odom", 10, &SimulationDrive::odomMsgCallBack, this);

  pose_sub_ = nh_.subscribe("/amcl_pose", 10, &SimulationDrive::poseMsgCallBack, this);
    

  return true;
}




// subscribers 

// void SimulationDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
// {
//   double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
// 	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

// 	tb3_pose_ = atan2(siny, cosy);
// }

void SimulationDrive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // uint16_t scan_angle[3] = {0, 30, 330};
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

  // ROS_INFO_STREAM("front: " << scan_data_[0] << ", left: " << scan_data_[1] << ", right: " << scan_data_[2]);
}

void SimulationDrive::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	pose_rot = atan2(siny, cosy) * RAD2DEG;
  pose_pos_x = msg->pose.pose.position.x;
  pose_pos_y = msg->pose.pose.position.y;

  // ROS_INFO_STREAM("x: " << pose_pos_x << ", y: " << pose_pos_y << ", z: " << pose_rot);
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
  status_client_ ac("move_base",true);
  ac.waitForServer();  
  
  move_base_msgs::MoveBaseGoal posegoal;

  posegoal.target_pose.pose.position.x = pos_x;
  posegoal.target_pose.pose.position.y = pos_y;
  
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  posegoal.target_pose.pose.orientation.z = sin(rot_z * DEG2RAD * 0.5);
  posegoal.target_pose.pose.orientation.w = cos(rot_z * DEG2RAD * 0.5);

  posegoal.target_pose.header.stamp = ros::Time::now();
  posegoal.target_pose.header.frame_id = "map";

  ROS_INFO("Sending goal");
  ac.sendGoal(posegoal);
  ac.waitForResult();
}








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
          
          if (enter_f && !enter_b)
          {
            rb_status = rail_f;
            ROS_INFO_STREAM("rail_f");
          }
          else if (!enter_f && enter_b)
          {
            rb_status = rail_b;
            ROS_INFO_STREAM("rail_b");
          }
        
        }
        else
        {
          if (enter_f && !enter_b) 
          {
            rb_status = betong_turn;
            ROS_INFO_STREAM("betong_turn");
            enter_f = true;
          }
          else if (!enter_f && enter_b) 
          {
            rb_status = betong_cross;
            ROS_INFO_STREAM("betong_cross");
            enter_b = true;
          }
        }
      }

      else {
        if (scan_data_[left] < side_dist_ && scan_data_[right] < side_dist_)
        {
          if (scan_data_[forward] < forward_dist_) 
          {
            rb_status = rail_end_f;
            ROS_INFO_STREAM("end_f");
            enter_f = false;
            enter_b = true;
          }
          else if (scan_data_[backward] < forward_dist_)
          {
            rb_status = rail_end_b;
            ROS_INFO_STREAM("end_b");
            enter_f = true;
            enter_b = false;
          }
  

        }
        else
        {
          ROS_WARN("where tf are u now?!");
        }
      }
        
      break;

// ----------------------------------------------------------------

    case betong_turn:
      if (!first) {
        pose_goal_x = pose_pos_x + 0.3;
        pose_goal_y = pose_pos_y;
        if (round%2 == 1)
        {
          pose_goal_rot = pose_rot + 90;
        }
        else 
        {
          pose_goal_rot = pose_rot - 90;
        }
        
        while (turns < 2) 
        {
          updateNavigationGoal(0, pose_goal_y, pose_goal_rot);
          turns += 1;
          round += 1;
          updateNavigationGoal(0, pose_goal_y + 0.4, pose_goal_rot + 90);
          turns += 1;
        }

        updatecommandVelocity(lin_vel,0.0);
        rb_status = get_placement;
      }
      else
      {
        updatecommandVelocity(lin_vel, 0.0);
        rb_status = get_placement;
      }
      break;


    case betong_cross:
      updatecommandVelocity(-lin_vel, 0.0);
      rb_status = get_placement;
      break;

    case rail_f:
      updatecommandVelocity(lin_vel, 0.0);
      first = false;
      turns = 0;
      rb_status = get_placement;
      break;

    case rail_b:
      updatecommandVelocity(-lin_vel, 0.0);
      rb_status = get_placement;
      break;

    case rail_end_f:
      updatecommandVelocity(-lin_vel, 0.0);
      rb_status = get_placement;
      break;

    case rail_end_b:
      updatecommandVelocity(lin_vel, 0.0);
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