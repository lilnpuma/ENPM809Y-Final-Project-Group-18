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
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
namespace fp {
    /**
     * @brief Class to represent a robot in an environment 
     * 
     */

    class Robot
    {
    public:
    /**
     * @brief Subscriber to get positions of aruco markers for the follower.
     * 
     * @param tfBuffer  
     */
    void listen(tf2_ros::Buffer& tfBuffer);

    /**
     * @brief Get aruco information from the camera and publish the location
     * 
     * @param msg 
     */
    void fiducial_callback(ros::NodeHandle m_nh, const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
    
    /**
     * @brief Get the goal object from parameter server and store it in aruco_loc
     * 
     * @param m_nh 
     * @param aruco_loc 
     */
    void get_goal(ros::NodeHandle m_nh, std::array<std::array<double, 2>, 4> &m_aruco_loc);

    int32_t id_callback(const std_msgs::String::ConstPtr& msg);

    void explore();
    void follow();

    private:
    ros::NodeHandle m_nh;
    bool saw_marker{false};
    std::array<std::array<double, 2>, 4> m_aruco_loc;
    std::array<std::array<double, 3>, 4> marker_loc; 


    };
}
#endif