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
    // main() will need to instantiate a ROS nodehandle, then pass it to the constructor
    /**
     * @brief Construct a new Robot object with nodehandle passed within the constructor
     * 
     * @param nodehandle 
     */
    Robot(ros::NodeHandle* nodehandle);
    
    /**
     * @brief method to directly publish velocity to robot
     * 
     * @param msg 
     */
    void publish_velocities(const geometry_msgs::Twist& msg);
    void rotate(double angle_to_rotate, bool direction, double final_angle);
    void stop();
    double compute_expected_final_yaw(bool direction, double angle_to_rotate);
    
    
    
    
    
    
    /**
     * @brief Get the goal from parameter server
     * 
     */
    void get_goal(std::vector<std::array<double, 2>> aruco_loc);
    /**
     * @brief move the robot to the goal location
     * 
     */
    void move(std::array<double,2> goal);
    /**
     * @brief turn the explorer around to find the aruco (fiducial callback)
     * 
     */
    void fiducial_id_reader(); 
    /**
     * @brief Creating all the Publishers
     * 
     */
    void talker();
    /**
     * @brief Creating all the subscribers
     * 
     */
    void listener();
    /**
     * @brief hardcoded method to send the robot to home
     * 
     */
    void go_home();

    /**
     * @brief for rotating the explorer for one round
     * 
     * @param angle_to_rotate 
     * @param direction 
     * @param final_angle 
     */
    void rotate(double angle_to_rotate, bool direction, double final_angle);

    private:
    ros::NodeHandle m_nh;
    std::string name;
    std::array<double,2>goal{};
    static std::array<std::array<double, 2>, 4> aruco_loc;
    ros::Publisher m_velocity_publisher;
    


    };
}
#endif