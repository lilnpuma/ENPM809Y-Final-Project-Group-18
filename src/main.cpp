#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include "../include/robot/robot.h"
#include "../include/bot_controller.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// ros::Publisher m_velocity_publisher;

// void m_initialize_publishers(ros::NodeHandle c) 
//   {
//     // ROS_INFO("Initializing Publishers");
//     m_velocity_publisher = c.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 100);
//     //add more subscribers here as needed
//   }

// void m_move(double linear, double angular) 
//   {
//       geometry_msgs::Twist msg;
//       msg.linear.x = linear;
//       msg.angular.z = angular;
//       m_velocity_publisher.publish(msg);
  // }

void get_goal(ros::NodeHandle n, std::vector<std::array<double, 2>> aruco_loc)
{
  XmlRpc::XmlRpcValue pos_list1;
  XmlRpc::XmlRpcValue pos_list2;
  XmlRpc::XmlRpcValue pos_list3;
  XmlRpc::XmlRpcValue pos_list4;
  //this can be done using 1 array (will try after testing)
  n.getParam("/aruco_lookup_locations/target_1", pos_list1);
  n.getParam("/aruco_lookup_locations/target_2", pos_list2);
  n.getParam("/aruco_lookup_locations/target_3", pos_list3);
  n.getParam("/aruco_lookup_locations/target_4", pos_list4);
  
  ROS_ASSERT(pos_list1.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pos_list2.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pos_list3.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pos_list1.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int32_t i = 0; i < pos_list1.size(); ++i)
  {
    ROS_ASSERT(pos_list1[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    aruco_loc.at(0).at(i) = static_cast<double>(pos_list1[i]);
  }

  for (int32_t i = 0; i < pos_list2.size(); ++i)
  {
    ROS_ASSERT(pos_list2[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    aruco_loc.at(1).at(i) = static_cast<double>(pos_list2[i]);
  } 

  for (int32_t i = 0; i < pos_list3.size(); ++i)
  {
    ROS_ASSERT(pos_list3[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    aruco_loc.at(2).at(i) = static_cast<double>(pos_list3[i]);
  } 

  for (int32_t i = 0; i < pos_list1.size(); ++i)
  {
    ROS_ASSERT(pos_list4[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    aruco_loc.at(3).at(i) = static_cast<double>(pos_list4[i]);
  }  
} 

void broadcast() {
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";

  transformStamped.transform.translation.x = 0.5;
  transformStamped.transform.translation.y = 0.5;
  transformStamped.transform.translation.z = 0.2;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;
  ROS_INFO("Broadcasting");
  br.sendTransform(transformStamped);
}

void listen(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "my_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;

    ROS_INFO_STREAM("Position in map frame: ["
      << trans_x << ","
      << trans_y << ","
      << trans_z << "]"
    );
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;
 
  

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  //writing this to retrieve position data from parameter server
  // XmlRpc::XmlRpcValue pos_list1;
  // nh.getParam("/aruco_lookup_locations/target_1", pos_list1);
  // ROS_ASSERT(pos_list1.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // for (int32_t i = 0; i < pos_list1.size(); ++i)
  // {
  //   ROS_ASSERT(pos_list1[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  //   aruco_loc.at(i) = static_cast<double>(pos_list1[i]);
  // } 

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;

  //Build goal for explorer
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = aruco_loc.at(0);//
  explorer_goal.target_pose.pose.position.y = aruco_loc.at(1);//
  explorer_goal.target_pose.pose.orientation.w = 0.0;

  //Build goal for follower
  // follower_goal.target_pose.header.frame_id = "map";
  // follower_goal.target_pose.header.stamp = ros::Time::now();
  // follower_goal.target_pose.pose.position.x = -0.289296;//
  // follower_goal.target_pose.pose.position.y = -1.282680;//
  // follower_goal.target_pose.pose.orientation.w = 1.0;


  // explorer_client.waitForResult();

  // ROS_INFO("Sending goal");
  // follower_client.sendGoal(follower_goal);
  // explorer_client.waitForResult();


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);
 
 

  // pub = rospy.publisher("")

  while (ros::ok()) {
    if (!explorer_goal_sent)     {
      ROS_INFO("Sending goal for explorer");
      explorer_client.sendGoal(explorer_goal);//this should be sent only once
      explorer_goal_sent = true;
    }
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      
      // m_move(0, 0.5);
      ROS_INFO("Hooray, robot reached goal");
    }
    // if (!follower_goal_sent) {
    //   ROS_INFO("Sending goal for explorer");
    //   follower_client.sendGoal(follower_goal);//this should be sent only once
    //   follower_goal_sent = true;
    // }
    // if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //   ROS_INFO("Hooray, robot reached goal");
    // }
    broadcast();
    listen(tfBuffer);
    //ros::spinOnce(); //uncomment this if you have subscribers in your code
    loop_rate.sleep();
  }


}