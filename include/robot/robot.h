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
#include<array>

namespace fp {
    /**
     * @brief Class to represent a robot in an environment 
     * 
     */

    class Robot
    {
    public:
    void get_goal();
    void move();
    void search_aruco(); 
    void talker();
    void listener();
    void go_home();

    private:
    std::array<double,2>goal {};
    std::array<std::array<double, 2>> aruco_loc;


    };
}
#endif