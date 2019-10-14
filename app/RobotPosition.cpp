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

std::vector<Point> RobotPosition::getJoints() {return jointPositions;}
std::vector<double> RobotPosition::getAngles() {return jointAngles;}

bool RobotPosition::checkCollision(Environment environment) {
return false;
}
