/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       Robot.cpp
 *  @author     Lydia Zoghbi, Ari Kupferberg
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/16/2019
 *  @version    1.0
 *
 *  @brief      Definitions for Robot.hpp to be completed
 *
 */

#include <cmath>
#include <vector>
#include "Robot.hpp"

explicit Robot::Robot(Point startingEEPosition) {
  initialEEPosition = startingEEPosition;
}

matrix<double> Robot::computeATransform(vector<double> dhRow) {
  double a = dhRow[0];
  double alpha = dhRow[1];
  double d = dhRow[2];
  double theta = dhRow[3];m 
  matrix<double> aTransform (4, 4);
  aTransform = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
				sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
					0,          sin(alpha),            cos(alpha),           d;
					0,               0,                     0,               0];
  return aTransform;
}

std::vector<Point> Robot::computeFK(std::vector<double> jointAngles) {
  dhparams[end][1:6] = jointAngles; // Might need to transpose... indexing correct?
  
// Initialize Matrices

  matrix<double> aTransform (4, 4),  tTransform (4, 4) previousTransform (4, 4);
  std::vector<matrix<double> (4, 4)> tTransforms; // Check syntaxing ...
  identity_matrix<double> identity (4, 4);
  previousTransform = identity;
  
// Loop through A matrices to get T matrices

  for (auto& row : dhParams) { // Or something like this...
    aTransform = computeATransform(dhparams[row]);
	tTransform = prod(previousTransform, aTransform);
	previousTransform = tTransform;
	tTransforms.push_back(tTransform);
  }

  std::vector<Point> points;
// Pull points from T matrices:
  for (auto& m : tTranforms) {
     Point point(m (1, 4), m (2, 4), m (3, 4));
	 points.push_back(point);
  }
  return points;	
}

matrix<double> Robot::computeJacobian(robotPosition, matrix<double>) {}
std::vector<Point> Robot::computeFK(std::vector<double> jointAngles) {}

std::vector<std::vector<RobotPosition>> Robot::computeIK(Point endEffectorPosition) {}
