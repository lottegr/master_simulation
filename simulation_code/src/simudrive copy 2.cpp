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
  escape_range_ = 30.0 * DEG2RAD;
  forward_dist_ = 0.3;
  side_dist_    = 0.2;

  // tb3_pose_ = 0.0;
  // prev_tb3_pose_ = 0.0;

  // end_of_rail = false;
  // on_rail = false;
  // forward = true;

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




// void SimulationDrive::statusMsgClient(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
// {
  // int move_base_status = msg->status_list[0].status;

  // ROS_INFO_STREAM(move_base_status);
// }




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
* Simulating driving function 
*******************************************************************************/

bool SimulationDrive::simulationLoop()
{
  static uint8_t rb_status = 0;
  
  switch(rb_status)
  {
    case get_placement:
      if (scan_data_[0] > forward_dist_ && scan_data_[2] > forward_dist_)
      {
        if (scan_data_[1] < side_dist_ && scan_data_[3] < side_dist_)
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
            ROS_INFO_STREAM("betong_f");
            enter_f = true;
          }
          else if (!enter_f && enter_b) 
          {
            rb_status = betong_cross;
            ROS_INFO_STREAM("betong_b");
            enter_b = true;
          }
        }
      }

      else {
        if (scan_data_[1] < side_dist_ && scan_data_[3] < side_dist_)
        {
          if (scan_data_[0] < forward_dist_) 
          {
            rb_status = rail_end_f;
            ROS_INFO_STREAM("end_f");
            enter_f = false;
            enter_b = true;
          }
          else if (scan_data_[2] < forward_dist_)
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
        pose_goal_x = pose_pos_x + 0.2;
        pose_goal_y = pose_pos_y;
        pose_goal_rot = pose_rot + 90;

        status_client_ ac("move_base",true);
        ac.waitForServer();

        move_base_msgs::MoveBaseGoal goal;

        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 1.0;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        ac.waitForResult();
      
      // }
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


              // prev_tb3_pose_ = tb3_pose_;

  // updateNavigationGoal(0,1,90);

  // ROS_INFO_STREAM("yo");

  return true;
}







/*******************************************************************************
* Main function
            ROS_INFO_STREAM("betong_f ");
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simudrive");
  SimulationDrive simudrive;

  ros::Rate loop_rate(25);
  // simudrive.laserScanMsgCallBack();

  while (ros::ok())
  {
    simudrive.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}