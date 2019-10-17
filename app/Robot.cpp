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
#include <boost/numeric/ublas/matrix.hpp>
#include "Robot.hpp"
#include <RobotPosition.hpp>
#include <RobotPath.hpp>
#include <Point.hpp>

Robot::Robot(const Point& startingPos): initialEEPosition(startingPos) {
}

boost::numeric::ublas::matrix<double> Robot::computeATransform(std::vector<double> dhRow) {
  double a = dhRow[0];
  double alpha = dhRow[1];
  double d = dhRow[2];
  double theta = dhRow[3]; 
  boost::numeric::ublas::matrix<double> aTransform (4, 4);
  
  // Initialize 'A' matrix variables
  std::vector<double> aTerms = {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta), sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta), 0, sin(alpha), cos(alpha), d, 0, 0, 0, 1};
  
  int i,j = 0;
  for (auto& term : aTerms) {
    aTransform (i, j) = term;
    j++;
    if (j == 6) {
      i++;
      j = 0;
    }
  }

  return aTransform;
}

std::vector<Point> Robot::computeFk(std::vector<double> jointAngles) {
  int i = 0;
  for (auto& element : jointAngles) {
    dhParams[6][i] = element;
    i++;
  }
  //dhparams[6][1:6] = jointAngles; // Might need to transpose... indexing correct?
  
// Initialize Matrices

  boost::numeric::ublas::matrix<double> aTransform (4, 4),  tTransform (4, 4), previousTransform (4, 4);
  std::vector<boost::numeric::ublas::matrix<double>> tTransforms; // Check syntaxing ...
  boost::numeric::ublas::identity_matrix<double> identity (4, 4);
  previousTransform = identity;
  
// Loop through A matrices to get T matrices

  for (auto& row : dhParams) { // Or something like this...
    aTransform = computeATransform(row);
	 tTransform = prod(previousTransform, aTransform);
	 previousTransform = tTransform;
	 tTransforms.push_back(tTransform);
  }

  std::vector<Point> points;
// Pull points from T matrices:
  for (auto& p : tTransforms) {
    Point point(p (1, 4), p (2, 4), p (3, 4));
	 points.push_back(point);
  }
  return points;	
}

boost::numeric::ublas::matrix<double> Robot::computeJacobian(RobotPosition robotPosition, boost::numeric::ublas::matrix<boost::numeric::ublas::matrix<double>> tTransforms) {
  std::vector<double> angles = robotPosition.getAngles();
  std::vector<Point> joints = robotPosition.getJoints();
  return tTransforms (1, 1);
}

//std::vector<std::vector<RobotPosition>> Robot::computeIK(Point endEffectorPosition) {}
