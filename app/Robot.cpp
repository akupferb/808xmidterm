/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       Robot.cpp
 *  @author     Lydia Zoghbi, Ari Kupferberg, Ryan Bates
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/19/2019
 *  @version    1.0
 *
 *  @brief      Definitions for Robot.hpp to be completed
 *
 */

#include <cmath>
#include <vector>
#include <boost/numeric/ublas/matrix.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include "Robot.hpp"
#include <RobotPosition.hpp>
#include <RobotPath.hpp>
#include <Point.hpp>
#include <iostream>
#include <math.h>


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

boost::numeric::ublas::vector<double> Robot::crossProduct(boost::numeric::ublas::vector<double> vector1, boost::numeric::ublas::vector<double> vector2) {
  boost::numeric::ublas::vector<double> resultVector(3);
  resultVector(0) = vector1(1)*vector2(2)-vector1(2)*vector2(1);
  resultVector(1) = vector1(2)*vector2(0)-vector1(0)*vector2(2);
  resultVector(2) = vector1(0)*vector2(1)-vector1(1)*vector2(0);

  return resultVector;
}


boost::numeric::ublas::matrix<double> Robot::computeJacobian(RobotPosition robotPosition) {

  // Get all the joint positions in Point
  std::vector<Point> joints = robotPosition.getJoints(); 
  boost::numeric::ublas::vector<double> zAxes (3);
  zAxes(0) = 0;
  zAxes(1) = 0;
  zAxes(2) = 1;

  boost::numeric::ublas::matrix<double> jacobian (6,6); 

  int index = 0;
  for( auto& joint : joints) {
    Point robotEEPosition = joints[6];
    Point robotJointPosition = joint;
    double differenceInX = robotEEPosition.getX() - robotJointPosition.getX();
    double differenceInY = robotEEPosition.getY() - robotJointPosition.getY();
    double differenceInZ = robotEEPosition.getZ() - robotJointPosition.getZ();
    boost::numeric::ublas::vector<double> differenceVector (3);
	differenceVector(0) = differenceInX;
	differenceVector(1) = differenceInY;
	differenceVector(2) = differenceInZ;

    boost::numeric::ublas::vector<double> transformedZAxes = prod(tTransformations[index],zAxes);
    
    boost::numeric::ublas::vector<double> crossProductVector = crossProduct(transformedZAxes, differenceVector); 
    
     
	// Populate the Jacobian's columns iteratively
    std::vector<double> iterator = {0, 1, 2};  
    for (auto& element : iterator){
		jacobian(element, index) = crossProductVector(element);
		jacobian(element, index+3) = transformedZAxes(element);
	};

    index++;
    
  };
  return jacobian;
}




// https://gist.github.com/javidcf/25066cf85e71105d57b6 used as reference
// We may want to add some error and bounds checking here
boost::numeric::ublas::matrix<double> Robot::penroseInverseMatrix(boost::numeric::ublas::matrix<double> mat) {
  boost::numeric::ublas::matrix<double> result(mat.size1(), mat.size2());

  Eigen::MatrixXd convertedInput(mat.size1(), mat.size2());

  // Convert from boost matrix to eigen matrix
  // we permit ourselves to use int for loops 
  // because the indexing is useful for referencing between datatypes

  for (unsigned int rowIndex = 0; rowIndex < mat.size1(); rowIndex++) {
    for (unsigned int columnIndex = 0; columnIndex < mat.size2(); columnIndex++) {
      convertedInput(rowIndex, columnIndex) = mat(rowIndex, columnIndex);
    }
  }


  // Now for the actual math

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(convertedInput, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // we can safely use adjoint instead of transpose because this is a real matrix
  Eigen::MatrixXd sigma = svd.singularValues().asDiagonal();

  double tolerance = 0.001;
  // take the reciprocal of each element, if possible, 
  // as per https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
  // we only hit the diagonal, since this is a diagonal matrix.
  // We allow this loop type because the indexing is useful.
  for (int i = 0; i < sigma.rows(); i++) {
    if (fabs(sigma(i, i)) < tolerance) {
      sigma(i, i) = 0;
    } else {
      sigma(i, i) = 1 / sigma(i, i);
    }
  }

  Eigen::MatrixXd sigmaTransposed = sigma.transpose();
  Eigen::MatrixXd eigenResult = svd.matrixV() * sigmaTransposed * svd.matrixU().adjoint();



  // Convert eigen matrix to boost matrix
  // we permit ourselves to use int for loops 
  // because the indexing is useful for referencing between datatypes

  
  
  for (unsigned int rowIndex = 0; rowIndex < mat.size1(); rowIndex++) {
    for (unsigned int columnIndex = 0; columnIndex < mat.size2(); columnIndex++) {
      result(rowIndex, columnIndex) = static_cast<double>(eigenResult(rowIndex, columnIndex));
      // Legacy for debugging purposes
      // std::cout << rowIndex << ", " << columnIndex << " is " << result(rowIndex, columnIndex) << std::endl;
    }
  }
  


  return result;
}


