#include "../include/robot/robot.h"
#include <ros/ros.h>

fp::Robot::Robot(ros::NodeHandle* nh) :
    m_nh{ *nh }
    


void fp::Robot::move(std::array<double,2> goal)
{
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    MoveBaseClient base_client("/" + name + "/move_base", true);
    // wait for the action server to come up
  while (!base_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for robot");
  }
  move_base_msgs::MoveBaseGoal goal;
  

}

void fp::Robot::get_goal(std::vector<std::array<double, 2>> aruco_loc)
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

 void fp::Robot::search_aruco()
 {
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0.2;
    m_velocity_publisher = m_nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 100);
    m_velocity_publisher.publish(msg);
    //check these commands with the main.cpp
 }

 



