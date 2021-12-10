/**
 * @file robot.h
 * @author Manu Pillai (manump@umd.edu)
 * @brief This file contains a class to represent a robot in an environment
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef ROBOT_H
#define ROBOT_H
#include <array>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <cstring>
namespace fp {
    /**
     * @brief Class to represent a robot in an environment 
     * 
     */

    class Robot
    {
    public:
    void get_goal();
    void move(std::array<double,2> goal);
    void search_aruco(); 
    void talker();
    void listener();
    void go_home();

    private:
    std::string name;
    std::array<double,2>goal{};
    std::array<std::array<double, 2>, 4> aruco_loc;


    };
}
#endif