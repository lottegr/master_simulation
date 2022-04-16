#include "simulation_code/sensors.h"


Sensors::Sensors()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Running sensors node");
  auto ret = init();
  ROS_ASSERT(ret);
}

Sensors::~Sensors()
{
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool Sensors::init()
{
  // initialize publishers
  // env_pub_ = nh_.advertise<std_msgs::String>("environment", 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Sensors::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Sensors::odomMsgCallBack, this);
  pose_sub_ = nh_.subscribe("/amcl_pose", 10, &Sensors::poseMsgCallBack, this);
  imu_sub_ = nh_.subscribe("/imu", 10, &Sensors::imuCallBack, this);
  odom_filter_sub_ = nh_.subscribe("odometry/filtered_odom", 10, &Sensors::odomFilterCallBack, this);
  

  return true;
}



// subscribers 

void Sensors::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
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


void Sensors::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

    pose_odom_rot = atan2(siny, cosy) * RAD2DEG;
    pose_odom_pos_x = msg->pose.pose.position.x;
    pose_odom_pos_y = msg->pose.pose.position.y;
}


void Sensors::poseMsgCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

    pose_amcl_rot = atan2(siny, cosy) * RAD2DEG;
    pose_amcl_pos_x = msg->pose.pose.position.x;
    pose_amcl_pos_y = msg->pose.pose.position.y;
}


void Sensors::imuCallBack(const sensor_msgs::Imu::ConstPtr &msg)
{
    double siny = 2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
    double cosy = 1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);  

    pose_imu_rot = atan2(siny, cosy) * RAD2DEG;
    pose_imu_pos_xdotdot = msg->linear_acceleration.x;
    pose_imu_pos_ydotdot = msg->linear_acceleration.y;
}


void Sensors::odomFilterCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

    pose_odom_f_rot = atan2(siny, cosy) * RAD2DEG;
    pose_odom_f_pos_x = msg->pose.pose.position.x;
    pose_odom_f_pos_y = msg->pose.pose.position.y;
}


// publishers

// void Sensors::updateEnvironment(std::string environment)
// {
//   std_msgs::String env;
//   env.data = environment;

//   env_pub_.publish(env);
// }



void Sensors::write_to_file(std::vector<double> v, std::string name)
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
* Sensors function 
*******************************************************************************/

bool Sensors::simulationLoop()
{
  pose_pos_x = 0;
  pose_pos_y = 0;
  pose_rot_z = 0;
  double new_meas = 0;
  double new_meas_rot = 0;

  if (pose_amcl_pos_x != prev_amcl_pos_x)
  {
    new_meas += 1;
    new_meas_rot += 1;
    pose_pos_x += pose_amcl_pos_x; 
    pose_pos_y += pose_amcl_pos_y; 
    pose_rot_z += pose_amcl_rot; 
  }
  
  if (pose_odom_pos_x != prev_odom_pos_x)
  {
    new_meas += 1;
    new_meas_rot += 1;
    pose_pos_x += pose_odom_pos_x; 
    pose_pos_y += pose_odom_pos_y; 
    pose_rot_z += pose_odom_rot; 
  }

  if (pose_imu_rot != prev_imu_rot)
  {
    new_meas_rot += 1;
    pose_rot_z += pose_imu_rot;
  }

// ---------------------------------------------------------

  if (new_meas == 0)
  {
    pose_pos_x = prev_pos_x;
    pose_pos_y = prev_pos_y;
    pose_rot_z = prev_rot_z;
  }
  else
  {
    pose_pos_x /= new_meas;
    pose_pos_y /= new_meas;
    pose_rot_z /= new_meas_rot;
  }
  

  prev_amcl_pos_x = pose_amcl_pos_x;
  prev_odom_pos_x = pose_odom_pos_x;
  prev_imu_rot = pose_imu_rot;
  prev_pos_x = pose_pos_x;
  prev_pos_y = pose_pos_y;
  prev_rot_z = pose_rot_z;
  


  pose_x.push_back(pose_pos_x);
  pose_y.push_back(pose_pos_y);
  pose_rot.push_back(pose_rot_z);







  odom_x.push_back(pose_odom_pos_x);
  odom_y.push_back(pose_odom_pos_y);
  odom_rot.push_back(pose_odom_rot);
  amcl_x.push_back(pose_amcl_pos_x);
  amcl_y.push_back(pose_amcl_pos_y);
  amcl_rot.push_back(pose_amcl_rot);
  imu_rot.push_back(pose_imu_rot);
  odom_f_x.push_back(pose_odom_f_pos_x);
  odom_f_y.push_back(pose_odom_f_pos_y);
  odom_f_rot.push_back(pose_odom_f_rot);

  write_to_file(odom_x,"odom_x");
  write_to_file(odom_y,"odom_y");
  write_to_file(odom_rot,"odom_rot");
  write_to_file(amcl_x,"amcl_x");
  write_to_file(amcl_y,"amcl_y");
  write_to_file(amcl_rot,"amcl_rot");
  write_to_file(imu_rot,"imu_rot");
  write_to_file(odom_f_x,"odom_f_x");
  write_to_file(odom_f_y,"odom_f_y");
  write_to_file(odom_f_rot,"odom_f_rot");



// ------------------------
  // double dt = 0.1;

  // pose_imu_pos_xdot += pose_imu_pos_xdotdot*dt;
  // pose_imu_pos_x += pose_imu_pos_xdot*dt;

  // imu_x.push_back(pose_imu_pos_x);
  // write_to_file(imu_x,"imu_x");

  // pose_imu_pos_ydot += pose_imu_pos_ydotdot*dt;
  // pose_imu_pos_y += pose_imu_pos_ydot*dt;

  // imu_y.push_back(pose_imu_pos_y);
  // write_to_file(imu_y,"imu_y");







  // write_to_file(pose_x,"pose_x");
  // write_to_file(pose_y,"pose_y");
  // write_to_file(pose_rot,"pose_rot");

  return true;
}







/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sensors");
  Sensors sensors;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sensors.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}














