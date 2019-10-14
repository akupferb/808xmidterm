/**
 *  @file       RobotPath.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Definitions for RobotPath.hpp
 *
 */

#include "RobotPath.hpp"

RobotPath::RobotPath(std::vector<RobotPosition> newRobotPositions) {
    robotPositions = newRobotPositions;
}
    std::vector<RobotPosition> RobotPath::getPositions() {return robotPositions;}

    bool RobotPath::existsCollision() {
return false;
}
