#include "../include/robot/robot.h"


int main(int argc, char** argv)
{
ros::init(argc, argv, "simple_navigation_goals");
ROS_INFO("initialized");
fp::Robot explorer("explorer");
ROS_INFO("created explorer object");
fp::Robot follower("follower");
ROS_INFO("created follower object");
std::array<std::array<double, 2>, 5> explorer_goals;
std::array<std::array<double, 2>, 5> follower_goals;
explorer_goals = explorer.get_goal();
ROS_INFO("got goals");
explorer.move(explorer_goals);
follower_goals = explorer.get_goal("explorer");
follower.move(follower_goals);
ros::shutdown();
}
