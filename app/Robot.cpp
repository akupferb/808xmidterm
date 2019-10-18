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
#include <boost/math/constants/constants.hpp>
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
  std::vector<double> aTerms = {cos(theta), -sin(theta)*cos(alpha), 
     sin(theta)*sin(alpha), a*cos(theta), sin(theta), cos(theta)*cos(alpha),
     -cos(theta)*sin(alpha), a*sin(theta), 0, sin(alpha), cos(alpha), d, 0, 
     0, 0, 1};
  
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
  
  // Initialize Matrices
  boost::numeric::ublas::matrix<double> aTransform (4, 4),  tTransform (4, 4), previousTransform (4, 4);
  boost::numeric::ublas::identity_matrix<double> identity (4, 4);
  previousTransform = identity;
  
  // Loop through A matrices to get T matrices
  std::vector<boost::numeric::ublas::matrix<double>> tTransforms;
  for (auto& row : dhParams) {
    aTransform = computeATransform(row);
	 tTransform = prod(previousTransform, aTransform);
	 previousTransform = tTransform;
	 tTransforms.push_back(tTransform);
  }

  // Pull points from T matrices:
  std::vector<Point> points;
  for (auto& p : tTransformations) {
    Point point(p (0, 3), p (1, 3), p (2, 3));
	 points.push_back(point);
  }
  return points;	
}


boost::numeric::ublas::matrix<double> Robot::computeJacobian(RobotPosition robotPosition) {

  std::vector<Point> joints = robotPosition.getJoints(); //Get all the joint positions in Point
  boost::numeric::ublas::vector<double> zAxes (3);
  zAxes.insert_element(0,0);
  zAxes.insert_element(0,1);
  zAxes.insert_element(1,2); 

  boost::numeric::ublas::matrix<double> jacobian; 
  boost::numeric::ublas::vector<double> jacobianPopulating (3);

  int index = 0;
  for( auto& joint : joints) {
    Point robotEEPosition = joints[6];
    Point robotJointPosition = joint;
    double differenceInX = robotEEPosition.getX() - robotJointPosition.getX();
    double differenceInY = robotEEPosition.getY() - robotJointPosition.getY();
    double differenceInZ = robotEEPosition.getZ() - robotJointPosition.getZ();
    boost::numeric::ublas::vector<double> differenceVector (3);
    differenceVector.insert_element(differenceInX, 0);
    differenceVector.insert_element(differenceInY, 1);
    differenceVector.insert_element(differenceInZ, 2);

    boost::numeric::ublas::vector<double> dummyVector = prod(tTransformations[index],zAxes);
    index++;
// Add the columns for the Jacobian


  }
  return jacobian;
}










//std::vector<std::vector<RobotPosition>> Robot::computeIK(Point endEffectorPosition) {}
