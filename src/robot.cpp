#include "../include/robot/robot.h"




void fp::Robot::get_goal(ros::NodeHandle m_nh, std::array<std::array<double, 2>, 4> &m_aruco_loc)
{
  XmlRpc::XmlRpcValue pos_list1;
  XmlRpc::XmlRpcValue pos_list2;
  XmlRpc::XmlRpcValue pos_list3;
  XmlRpc::XmlRpcValue pos_list4;
  //this can be done using 1 array (will try after testing)
  m_nh.getParam("/aruco_lookup_locations/target_1", pos_list1);
  m_nh.getParam("/aruco_lookup_locations/target_2", pos_list2);
  m_nh.getParam("/aruco_lookup_locations/target_3", pos_list3);
  m_nh.getParam("/aruco_lookup_locations/target_4", pos_list4);
  
  ROS_ASSERT(pos_list1.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pos_list2.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pos_list3.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pos_list1.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int32_t i = 0; i < pos_list1.size(); ++i)
  {
    ROS_ASSERT(pos_list1[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    m_aruco_loc.at(0).at(i) = static_cast<double>(pos_list1[i]);
  }

  for (int32_t i = 0; i < pos_list2.size(); ++i)
  {
    ROS_ASSERT(pos_list2[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    m_aruco_loc.at(1).at(i) = static_cast<double>(pos_list2[i]);
  } 

  for (int32_t i = 0; i < pos_list3.size(); ++i)
  {
    ROS_ASSERT(pos_list3[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    m_aruco_loc.at(2).at(i) = static_cast<double>(pos_list3[i]);
  } 

  for (int32_t i = 0; i < pos_list4.size(); ++i)
  {
    ROS_ASSERT(pos_list4[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    m_aruco_loc.at(3).at(i) = static_cast<double>(pos_list4[i]);
  }  

}

void fp::Robot::fiducial_callback(ros::NodeHandle m_nh, const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
  ROS_INFO("Inside callback");
  int32_t aruco_id; //marker index
 if (!msg->transforms.empty())
 {  saw_marker = true;
    ROS_INFO_STREAM("Seen marker:  ["<< msg->transforms[0].fiducial_id<< "]");
    aruco_id = msg->transforms[0].fiducial_id;
    static tf2_ros::TransformBroadcaster brc;
    geometry_msgs::TransformStamped transformStamped;
     
    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";
    //creation of marker_frame with relative to camera frame at given distance
    //which is exactly what we are seeing in the camera frame
    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
    
    ros::Publisher pub_id = m_nh.advertise<std_msgs::Int32>("explorer_tf/camera_rgb_optical_frame/marker_frame/id", 10);
    pub_id.publish(aruco_id);
    ROS_INFO("Broadcasting marker location");
    brc.sendTransform(transformStamped);
    
 }
}

// int32_t fp::Robot::id_callback(const std_msgs::String::ConstPtr& msg)
// {
//    return msg->data;
// }

void fp::Robot::listen(tf2_ros::Buffer& tfBuffer)
{
  ros::NodeHandle m_nh;
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;
    int32_t aruco_id;
    // ros::Subscriber sub = m_nh.subscribe("explorer_tf/camera_rgb_optical_frame/marker_frame/id", 10, id_callback);
    marker_loc.at(aruco_id).at(0) = aruco_id;
    marker_loc.at(aruco_id).at(1) = trans_x;
    marker_loc.at(aruco_id).at(2) = trans_y;

    ROS_INFO_STREAM("pos_ marker loc new at listener is "<<aruco_id);


    ROS_INFO_STREAM("Position in map frame: ["
      << trans_x << ","
      << trans_y << ","
      << trans_z << "]"
    );

    ROS_INFO_STREAM("Marker is at location:  ["<< marker_loc.at(0).at(0)<< "]"<<"Marker is at location:  ["
    << marker_loc.at(1).at(0)<< "]"<<"Marker is at location:  ["<< marker_loc.at(2).at(0)<< "]");
    
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
}
}

void fp::Robot::explore(ros::NodeHandle m_nh, std::array<std::array<double, 2>, 4> &m_aruco_loc)
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  bool explorer_goal_sent = false;
  fp::Robot::get_goal(m_nh, m_aruco_loc);
  MoveBaseClient explorer_client("/explorer/move_base", true);
  
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }
  move_base_msgs::MoveBaseGoal explorer_goal;

  int i = 0;
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = m_aruco_loc.at(i).at(0);//
  explorer_goal.target_pose.pose.position.y = m_aruco_loc.at(i).at(1);//
  explorer_goal.target_pose.pose.orientation.w = 1;
  i++;

  explorer_client.waitForResult();
  ROS_INFO("Sending goal");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(100);

  //buidling a publisher to do some tricks
  ros::Publisher m_velocity_publisher;
  ros::Subscriber fid_reader;
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = 0.4;
  //writing this to move the bot with a constant angular velocity
  m_velocity_publisher = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 100);
}




