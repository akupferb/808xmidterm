/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
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

bool RobotPath::existsCollision(Environment environment) {
  std::vector<Obstacle> obstacles = environment.getObstacles();
  for (auto position : robotPositions) {
    if (!(position.checkCollision(environment))) {
      return true;
    }
  }
  return false;
}

std::vector<RobotPosition> RobotPath::getPositions() {return robotPositions;}
