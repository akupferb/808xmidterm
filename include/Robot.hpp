/*
 *  Distributed under the Boost Software License.
 *  Version 2.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       Robot.hpp
 *  @author     Lydia Zoghbi, Ari Kupferberg
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/15/2019
 *  @version    2.0
 *
 *  @brief      Header file for constructing a Robot class. 
 *
 */

#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_

#include <vector>
#include <boost/numeric/ublas/matrix.hpp>
#include <RobotPosition.hpp>
#include <RobotPath.hpp>
#include <Point.hpp>

/**
 *  @brief      Class for creating a Robot object containing useful information about the system
 */

class Robot {
 private:
   int numberLinks = 6;
   std::vector<double> linkLengths;
   Point initialEEPosition;
   RobotPosition currentRobotPosition, goalPosition;
   std::vector<RobotPath> path;
   std::vector<std::vector<double>> dhParams;
  /**
   *  @brief      Compute the intermediate transformation between two DH frames
   *  @param	DH Table as matrix double
   *  @return	'A' transformation matrix
   */
   matrix<double> computeATransform(std::vector<std::vector<double>> dhParams);
  /**
   *  @brief      Computing the forward kinematics for the Robot
   *  @param	Vector of Robot's joint angles as double
   *  @return	Vector of Point objects depicting Robot's joint positions
   */
   std::vector<Point> computeFK(std::vector<double> jointAngles);
  /**
   *  @brief      Compute the intermediate transformation between two DH frames
   *  @param	DH Table as matrix double
   *  @return	'A' transformation matrix
   */
   matrix<double> computeJacobian(robotPosition, matrix<double>);
  /**
   *  @brief      Computing the forward kinematics for the Robot
   *  @param	Vector of Robot's joint angles as double
   *  @return	Vector of Point objects depicting Robot's joint positions
   */
   std::vector<Point> computeFK(std::vector<double> jointAngles);

 public:
  /**
   *  @brief      Constructor for class Robot 
   *  @param	Point of the robot's end effector position
   *  @return	Instance of robot
   */
   explicit Robot(Point);

  /**
   *  @brief      Computing the inverse kinematics for Robot
   *  @param	Point of Robot's end effector target position
   *  @return	Vector of vector of RobotPosition object containing Points of joint positions
   */
   std::vector<std::vector<RobotPosition>> computeIK(Point);
};


#endif // INCLUDE_ROBOT_HPP_
