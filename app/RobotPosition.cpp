/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       RobotPosition.cpp
 *  @author     Lydia Zoghbi, Ari Kupferberg
 *  @copyright  Copyright ARL 2019
 *  @date       10/16/2019
 *  @version    1.0
 *
 *  @brief      Definitions for RobotPosition.hpp
 *
 */

#include "RobotPosition.hpp"
#include <math.h>

RobotPosition::RobotPosition(std::vector<Point> newJointPositions, std::vector<double> newJointAngles) {
  jointPositions = newJointPositions;
  jointAngles = newJointAngles;
}

bool RobotPosition::checkCollision(Environment environment) {
  std::vector<Obstacle> obstacles = environment.getObstacles();
  for (auto obstacle : obstacles) {
    for (auto jointPoint : jointPositions) {
      // distance//collision formula is:
      // sqrt(deltaX^2 + deltaY^2 + deltaZ^2) < radiusDistance
      // square roots are expensive, so it accepted practice to test the squares instead, as:
      // sqrt(deltaX^2 + deltaY^2 + deltaZ^2) < radiusDistance
      if ((pow((obstacle.getCentroid().getX() - jointPoint.getX()), 2) +
           pow((obstacle.getCentroid().getY() - jointPoint.getY()), 2) +
           pow((obstacle.getCentroid().getZ() - jointPoint.getZ()), 2))
           < pow(obstacle.getRadius(), 2)) {
        return false;
      }
    }
  }
  return true;
}

std::vector<Point> RobotPosition::getJoints() {return jointPositions;}
std::vector<double> RobotPosition::getAngles() {return jointAngles;}
