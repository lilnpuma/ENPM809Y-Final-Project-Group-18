#include "../include/robot/robot.h"
void fp::Robot::get_goal()
{
    
}

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