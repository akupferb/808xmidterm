/**
 *  @file       Robot.hpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Header file for consructing a Robot class. 
 *
 */

#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_
#include <vector>
#include <RobotPosition.hpp>
#include <RobotPath.hpp>
#include <Point.hpp>

/**
 *  @brief      Class for creating a Robot object containing useful information about the system
 */

class Robot {
private:
    int linkCount;
    std::vector<double> linkLengths;
    Point initialEEPosition;
    RobotPosition currentRobotPosition, goalPosition;
    std::vector<RobotPath> path;
public:

/**
 *  @brief      Constructor for class Robot 
 *  @param	Point of the robot's end effector position
 *  @return	Instance of robot
 */
    explicit Robot(Point startingEEPosition);

/**
 *  @brief      Computing the forward kinematics for Robot
 *  @param	Vector of Robot's joint angles as double
 *  @return	Vector of Point objects depicting Robot's joint positions
 */
    std::vector<Point> computeFK(std::vector<double> jointAngles);

/**
 *  @brief      Computing the inverse kinematics for Robot
 *  @param	Point of Robot's end effector target position
 *  @return	Vector of vector of RobotPosition object containing Points of joint positions
 */
    std::vector<std::vector<RobotPosition>> computeIK(Point endEffectorPosition);
};
#endif // INCLUDE_ROBOT_HPP_
