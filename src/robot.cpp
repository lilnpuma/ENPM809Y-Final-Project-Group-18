#include "../include/robot/robot.h"


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

<<<<<<< HEAD
void fp::Robot::search_aruco()
{
    
}
=======
}

void fp::Robot::get_goal(ros::NodeHandle n, std::vector<std::array<double, 2>> aruco_loc)
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
>>>>>>> 0d0d023e00db8878f46aa1e4e1857f5adc63a340
