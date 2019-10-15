/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       RobotPosition.hpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Header file for consructing a RobotPosition class 
 *
 */

#ifndef INCLUDE_ROBOTPOSITION_HPP_
#define INCLUDE_ROBOTPOSITION_HPP_
#include <vector>
#include <Environment.hpp>

/**
 *  @brief      Class for creating robot positions
 */

class RobotPosition {
 private:
   std::vector<Point> jointPositions;
   std::vector<double> jointAngles;

 public:
  /**
   *  @brief      Constructor for creating RobotPosition
   *  @param	Vector of Points for Robot's joints positions and vector of joint angles as double
   *  @return	Instance of RobotPosition
   */
   RobotPosition(std::vector<Point> newJointPositions, std::vector<double> newJointAngles);

  /**
   *  @brief      Retrieve joints from RobotPosition class
   *  @param	None
   *  @return	Vector of Points of Robot's joints positions
   */
   std::vector<Point> getJoints();

  /**
   *  @brief      Retrieve angles from RobotPosition class
   *  @param	None
   *  @return	Vector of Robot's joints angles as double
   */
   std::vector<double> getAngles();

  /**
   *  @brief      Check for collision with obstacles
   *  @param	Environment containing obstacles' location and radius
   *  @return	Boolean as true for collision
   */
   bool checkCollision(Environment environment);
};


#endif // INCLUDE_ROBOTPOSITION_HPP_
