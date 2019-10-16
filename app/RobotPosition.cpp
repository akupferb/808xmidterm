/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       RobotPosition.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Definitions for RobotPosition.hpp
 *
 */

#include "RobotPosition.hpp"

RobotPosition::RobotPosition(std::vector<Point> newJointPositions, std::vector<double> newJointAngles) {
  jointPositions = newJointPositions;
  jointAngles = newJointAngles;
}

bool RobotPosition::checkCollision(Environment environment) {
  return false;
}

std::vector<Point> RobotPosition::getJoints() {return jointPositions;}
std::vector<double> RobotPosition::getAngles() {return jointAngles;}
