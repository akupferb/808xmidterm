/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       Robot.cpp
 *  @author     Lydia Zoghbi, Ari Kupferberg, Ryan Bates
 *  @copyright  Copyright ARL 2019
 *  @date       10/19/2019
 *  @version    1.0
 *
 *  @brief      Definitions for Robot.hpp 
 *
 */
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <math.h>
#include <cmath>
#include <vector>
#include <iostream>
#include "Robot.hpp"
#include <RobotPosition.hpp>
#include <RobotPath.hpp>
#include <Point.hpp>
#include <boost/numeric/ublas/matrix.hpp>

Robot::Robot(const Point& startingPos): initialEEPosition(startingPos) {
}

boost::numeric::ublas::matrix<double> Robot::computeATransform(std::vector<double> dhRow) {

  // Initializing parameters
  double a = dhRow[0];
  double alpha = dhRow[1];
  double d = dhRow[2];
  double theta = dhRow[3];
  boost::numeric::ublas::matrix<double> aTransform(4, 4);

  // Initialize 'A' matrix variables
  std::vector<double> aTerms = {cos(theta), -sin(theta)*cos(alpha),
     sin(theta)*sin(alpha), a*cos(theta), sin(theta), cos(theta)*cos(alpha),
     -cos(theta)*sin(alpha), a*sin(theta), 0, sin(alpha), cos(alpha), d, 0,
     0, 0, 1};

  int i = 0;
  int j = 0;

  // Populate the A transformation matrix
  for (auto& term: aTerms) {
    aTransform(i, j) = term;
    j++;
    if (j == 4) {
      i++;
      j = 0;
    }
  }
  return aTransform;
}

std::vector<Point> Robot::computeFk(std::vector<double> jointAngles) {
  int i = 0;
  for (auto& element : jointAngles) {
    dhParams[i][3] = element + dhParams[i][3];
    i++;
  }

  // Initialize Matrices
  boost::numeric::ublas::matrix<double> aTransform(4, 4);
  boost::numeric::ublas::matrix<double> tTransform(4, 4);
  boost::numeric::ublas::matrix<double> previousTransform(4, 4);
  boost::numeric::ublas::identity_matrix<double> identity(4, 4);
  previousTransform = identity;

  // Loop through A matrices to get T matrices
  std::vector<boost::numeric::ublas::matrix<double>> tTransforms;
  for (auto& row : dhParams) {
     aTransform = computeATransform(row);
	 tTransform = prod(previousTransform, aTransform);
	 previousTransform = tTransform;
	 tTransforms.push_back(tTransform);
  }

  // Pull points from T matrices
  Point initialPoint(0.0, 0.0, 0.0);
  std::vector<Point> points = {initialPoint};
  for (auto& p : tTransforms) {
     Point point(p(0, 3), p(1, 3), p(2, 3));
     points.push_back(point);
  }
  return points;
}

boost::numeric::ublas::vector<double> Robot::crossProduct(boost::numeric::ublas::vector<double> vector1, boost::numeric::ublas::vector<double> vector2) {

  // Cross product function because there was no easy way to get it done through Boost
  boost::numeric::ublas::vector<double> resultVector(3);
  resultVector(0) = vector1(1)*vector2(2)-vector1(2)*vector2(1);
  resultVector(1) = vector1(2)*vector2(0)-vector1(0)*vector2(2);
  resultVector(2) = vector1(0)*vector2(1)-vector1(1)*vector2(0);
  return resultVector;
}

std::vector<boost::numeric::ublas::matrix<double>> Robot::computeTransformationMatrices(std::vector<double> jointAngles) {

 int i = 0;
  for (auto& element : jointAngles) {
    dhParams[i][3] = element + dhParams[i][3];
    i++;
  }

  // Initialize Matrices
  boost::numeric::ublas::matrix<double> aTransform(4, 4);
  boost::numeric::ublas::matrix<double> tTransform(4, 4);
  boost::numeric::ublas::matrix<double> previousTransform(4, 4);
  boost::numeric::ublas::identity_matrix<double> identity(4, 4);
  previousTransform = identity;

  // Loop through A matrices to get T matrices
  std::vector<boost::numeric::ublas::matrix<double>> tTransforms;
  tTransforms.push_back(identity);
  for (auto& row : dhParams) {
     aTransform = computeATransform(row);
	 tTransform = prod(previousTransform, aTransform);
	 previousTransform = tTransform;
	 tTransforms.push_back(tTransform);
  }
 return tTransforms;
}

boost::numeric::ublas::matrix<double> Robot::computeJacobian(RobotPosition robotPosition, std::vector<boost::numeric::ublas::matrix<double>> tTransforms) {

  // Get all the joint positions in Point
  std::vector<Point> joints = robotPosition.getJoints();
  boost::numeric::ublas::vector<double> zAxes(3);
  zAxes(0) = 0;
  zAxes(1) = 0;
  zAxes(2) = 1;

  // Initialize some parameteres
  boost::numeric::ublas::matrix<double> jacobian(6,6);
  std::vector<double> iterator = {0, 1, 2};
  boost::numeric::ublas::matrix<double> trans(3,3);

  int index = 0;
  Point robotEEPosition = joints[6];
  joints.pop_back();

  // Loop through the joints to compute the Jacobian
  for (auto& joint : joints) {
    Point robotJointPosition = joint;
    double differenceInX = robotEEPosition.getX() - robotJointPosition.getX();
    double differenceInY = robotEEPosition.getY() - robotJointPosition.getY();
    double differenceInZ = robotEEPosition.getZ() - robotJointPosition.getZ();
    boost::numeric::ublas::vector<double> differenceVector(3);
	differenceVector(0) = differenceInX;
	differenceVector(1) = differenceInY;
	differenceVector(2) = differenceInZ;

    trans(0,0) = tTransforms[index](0,0); trans(0,1) = tTransforms[index](0,1);
    trans(0,2) = tTransforms[index](0,2); trans(1,0) = tTransforms[index](1,0);
    trans(1,1) = tTransforms[index](1,1); trans(1,2) = tTransforms[index](1,2);
    trans(2,0) = tTransforms[index](2,0); trans(2,1) = tTransforms[index](2,1);
    trans(2,2) = tTransforms[index](2,2);
    boost::numeric::ublas::vector<double> transformedZAxes = prod(trans,zAxes);
    boost::numeric::ublas::vector<double> crossProductVector = crossProduct(transformedZAxes, differenceVector);

	// Populate the Jacobian's columns iteratively
    for (auto& element : iterator) {
		jacobian(element, index) = crossProductVector(element);
		jacobian(element+3, index) = transformedZAxes(element);
	}
    index++;
  }
  return jacobian;
}

// https://gist.github.com/javidcf/25066cf85e71105d57b6 used as reference
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

  // We can safely use adjoint instead of transpose because this is a real matrix
  Eigen::MatrixXd sigma = svd.singularValues().asDiagonal();

  double tolerance = 0.001;
  // Take the reciprocal of each element, if possible,
  // as per https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
  // We only hit the diagonal, since this is a diagonal matrix.
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
  // We permit ourselves to use int for loops
  // because the indexing is useful for referencing between datatypes
  for (unsigned int rowIndex = 0; rowIndex < mat.size1(); rowIndex++) {
    for (unsigned int columnIndex = 0; columnIndex < mat.size2(); columnIndex++) {
      result(rowIndex, columnIndex) = static_cast<double>(eigenResult(rowIndex, columnIndex));
    }
  }
  return result;
}

std::vector<RobotPosition> Robot::computeIK(Point targetPoint, Environment environment) {

    // Initialize the angles vector theta
	std::vector<RobotPosition> allRobotPositions;
    boost::numeric::ublas::vector<double> velocity(6);
	boost::numeric::ublas::vector<double> theta(6);
	theta(0) = initialJointAngles[0]; theta(1) = initialJointAngles[1]; theta(2) = initialJointAngles[2];
	theta(3) = initialJointAngles[3]; theta(4) = initialJointAngles[4]; theta(5) = initialJointAngles[5];

	std::vector<double> jointAngles = initialJointAngles;

    std::vector<Point> robotJointPosition = computeFk(jointAngles);
	Point eePosition = robotJointPosition.back();
	PathPlanner pathPlanner;
	std::vector<Point> getPath = pathPlanner.findStraightPath(eePosition, targetPoint);

	double eeX; double eeY; double eeZ;
	double targetX; double targetY; double targetZ;
	double norm;
	int counter = 0;

	for(auto& pathPoints:getPath) {

		// Extact each point from a discretized path to find joint angles
		robotJointPosition = computeFk(jointAngles);
		eePosition = robotJointPosition.back();
		eeX = eePosition.getX(); eeY = eePosition.getY(); eeZ = eePosition.getZ();
		targetX = pathPoints.getX(); targetY = pathPoints.getY(); targetZ = pathPoints.getZ();
    	velocity(0) = targetX - eeX; velocity(1) = targetY - eeY; velocity(2) = targetZ - eeZ;
 		velocity(3) = 0.0; velocity(4) = 0.0; velocity(5) = 0.0;
		norm = norm_2(velocity);
	 	counter++;

		// Loop through each point to compute the inverse kinematics using the pseudo-inverse Jacobian
		while (norm > 0.01) {

    		robotJointPosition = computeFk(jointAngles);
			RobotPosition robotPosition(robotJointPosition, jointAngles);
		    std::vector<boost::numeric::ublas::matrix<double>> tTrans = computeTransformationMatrices(jointAngles);
			boost::numeric::ublas::matrix<double> jacobian = computeJacobian(robotPosition, tTrans);
			boost::numeric::ublas::matrix<double> pseudoInverse = penroseInverseMatrix(jacobian);
			Point eePosition = robotJointPosition.back();

			eeX = eePosition.getX(); eeY = eePosition.getY(); eeZ = eePosition.getZ();
			targetX = pathPoints.getX(); targetY = pathPoints.getY(); targetZ = pathPoints.getZ();
    		velocity(0) = targetX - eeX; velocity(1) = targetY - eeY; velocity(2) = targetZ - eeZ;
			velocity(3) = 0.0; velocity(4) = 0.0; velocity(5) = 0.0;
			norm = norm_2(velocity);

			boost::numeric::ublas::vector<double> deltaTheta = prod(pseudoInverse, velocity);
			theta = theta + 0.0000001*deltaTheta;
			jointAngles[0] = theta(0); jointAngles[1] = theta(1); jointAngles[2] = theta(2);
			jointAngles[3] = theta(3); jointAngles[4] = theta(4); jointAngles[5] = theta(5);

		}

		RobotPosition robotPosition(robotJointPosition, jointAngles);

		// Check for any collision
		if (robotPosition.checkCollision(environment)) {
			allRobotPositions.push_back(robotPosition);
		} else {break;}
	}

return allRobotPositions;

}

