#include "simulation_code/rows_and_sections.h"
#include "simulation_code/odom_switch.h"


OdomSwitch::OdomSwitch()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Running odomSwitch node");
  auto ret = init();
  ROS_ASSERT(ret);
}

OdomSwitch::~OdomSwitch()
{
  ros::shutdown();
}


/*******************************************************************************
* Init function
*******************************************************************************/
bool OdomSwitch::init()
{
  // initialize publishers
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom_", 10);

  // initialize subscribers
  odom_ground_sub_ = nh_.subscribe("odom", 10, &OdomSwitch::odomGroundCallBack, this);
    
  return true;
}



// subscribers 

void OdomSwitch::odomGroundCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  header_g = msg->header;
  child_frame_id_g = msg->child_frame_id;
  pose_g = msg->pose;
  twist_g = msg->twist;
}



// publishers

void OdomSwitch::updateOdom(std_msgs::Header header, 
                            std::string child_frame_id,
                            geometry_msgs::PoseWithCovariance pose,
                            geometry_msgs::TwistWithCovariance twist, float x, float y, float z, float w)
{
  nav_msgs::Odometry odom;

  odom.header = header_g;
  odom.header.frame_id = "map";
  odom.child_frame_id = child_frame_id_g;


  double siny = 2.0 * (w * z );
	double cosy = 1.0 - 2.0 * (z * z);  
  double sinyy = 2.0 * (pose.pose.orientation.w * pose.pose.orientation.z);
	double cosyy = 1.0 - 2.0 * (pose.pose.orientation.z * pose.pose.orientation.z);  

	// pose_odom_rot = atan2(siny, cosy) * RAD2DEG;

  double rotmap = atan2(siny, cosy);
  double rotodom = atan2(sinyy, sinyy);

  // double xmap = 
  // double ymap = 
  double xodom = pose.pose.position.x;
  double yodom = pose.pose.position.y;


  odom.pose.pose.position.x = x + sqrt(xodom*xodom + yodom*yodom)*cos(rotodom);
  odom.pose.pose.position.y = y + + sqrt(xodom*xodom + yodom*yodom)*sin(rotodom);;
  odom.pose.pose.orientation.z = pose.pose.orientation.z + z;
  odom.pose.pose.orientation.w = pose.pose.orientation.w + w;

  odom_pub_.publish(odom);
}


// void tfToPose(tf::Transform &trans, geometry_msgs::PoseStamped &msg)
//  {
//    tf::quaternionTFToMsg(trans.getRotation(), msg.pose.orientation);
//    msg.pose.position.x = trans.getOrigin().x();
//    msg.pose.position.y = trans.getOrigin().y();
//    msg.pose.position.z = trans.getOrigin().z();
//  }




void OdomSwitch::tfListener()
{
  tf::StampedTransform transform;

  try
  {
    tf_p_listener.lookupTransform("/map", "/odom", ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) 
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
    

  pose_pos_x = transform.getOrigin().x();
  pose_pos_y = transform.getOrigin().y();
  pose_rot_z = transform.getRotation().getZ();
  pose_rot_w = transform.getRotation().getW();

  return; 
}



/*******************************************************************************
* OdomSwitch function 
*******************************************************************************/

bool OdomSwitch::simulationLoop()
{
  tfListener();

  updateOdom(header_g, child_frame_id_g, pose_g, twist_g, pose_pos_x, pose_pos_y, pose_rot_z, pose_rot_w);



    

  return true;
}







/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "odomSwitch");
  OdomSwitch odomSwitch;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    odomSwitch.simulationLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}