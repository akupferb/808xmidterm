/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       RobotTest.cpp
 *  @author     Ryan Bates
 *  @copyright  Copyright ARL 2019
 *  @date       10/17/2019
 *  @version    1.0
 *
 *  @brief      Unit test
 *
 */

#include <gtest/gtest.h>
#include <iostream>
#include "Robot.hpp"

TEST(RobotTest, Constructor) {

}

TEST(RobotTest, PenroseInverseMatrix) {

  Point dummyPoint1(0.0, 0.0, 0.0);
  Robot robot(dummyPoint1);
  boost::numeric::ublas::matrix<double> input(3, 3);
  input(0, 0) = 1.0; input(0, 1) = 2.0; input(0, 2) = -1.0;
  input(1, 0) = 3.0; input(1, 1) = 7.0; input(1, 2) = -10.0;
  input(2, 0) = 7.0; input(2, 1) = 16.0; input(2, 2) = -21.0;

  boost::numeric::ublas::matrix<double> result = robot.penroseInverseMatrix(input);

  
  unsigned int requiredDimension = 3;
  ASSERT_EQ(requiredDimension, result.size1());  
  ASSERT_EQ(requiredDimension, result.size2());



  EXPECT_NEAR(0.306, result(0, 0), 0.01);
  EXPECT_NEAR(-0.130, result(0, 1), 0.01);
  EXPECT_NEAR(0.044, result(0, 2), 0.01);
  EXPECT_NEAR(-0.211, result(1, 1), 0.01);
  EXPECT_NEAR(-0.220, result(2, 1), 0.01);
  EXPECT_NEAR(0.035, result(2, 2), 0.01);
}

TEST(RobotTest, ComputeATransform) {
  std::vector<double> dhRow = {0.41, M_PI/2.0, 1.0, 0.0};
  Point dummyPoint(0.0, 0.0, 0.0);
  Robot robot(dummyPoint);
  boost::numeric::ublas::matrix<double> aTransform = robot.computeATransform(dhRow);

  EXPECT_NEAR(1.00, aTransform(0, 0), 0.01);
  EXPECT_NEAR(0.00, aTransform(0, 1), 0.01);
  EXPECT_NEAR(-1.00, aTransform(1, 2), 0.01);
  EXPECT_NEAR(1.00, aTransform(2, 1), 0.01);
  EXPECT_NEAR(1.00, aTransform(2, 3), 0.01);
  EXPECT_NEAR(1.00, aTransform(3, 3), 0.01);
}

TEST(RobotTest, ComputeFK) {
  std::vector<double> jointAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Point dummyPoint(0.0, 0.0, 0.0);
  Robot robot(dummyPoint);
  std::vector<Point> jointPositions = robot.computeFk(jointAngles);

  Point firstPoint = jointPositions[2];
  Point eePosition = jointPositions.back();

  double xFirstPoint = firstPoint.getX();
  double yFirstPoint = firstPoint.getY();
  double zFirstPoint = firstPoint.getZ();
  double xEEPoint = eePosition.getX();
  double yEEPoint = eePosition.getY();
  double zEEPoint = eePosition.getZ();


  EXPECT_NEAR(1820, xFirstPoint, 0.01);
  EXPECT_NEAR(0.00, yFirstPoint, 0.01);
  EXPECT_NEAR(750, zFirstPoint, 0.01);
  EXPECT_NEAR(2020, xEEPoint, 1);
  EXPECT_NEAR(0.0, yEEPoint, 1);
  EXPECT_NEAR(-1280, zEEPoint, 1);
}

TEST(RobotTest, FindCrossProduct) {
  boost::numeric::ublas::vector<double> vector1(3);
  vector1(0) = 1.2; vector1(1) = 4.5; vector1(2) = 6.8;
  boost::numeric::ublas::vector<double> vector2(3);
  vector2(0) = 14.6; vector2(1) = 18.5; vector2(2) = 12.8;

  Point dummyPoint(0.0, 0.0, 0.0);
  Robot robot(dummyPoint);
  boost::numeric::ublas::vector<double> crossProd = robot.crossProduct(vector1, vector2);

  EXPECT_NEAR(-68.2, crossProd(0), 0.01);
  EXPECT_NEAR(83.92, crossProd(1), 0.01);
  EXPECT_NEAR(-43.5, crossProd(2), 0.01);
}

TEST(RobotTest, FindJacobian) {
  boost::numeric::ublas::vector<double> vector1(3);
  vector1(0) = 1.2; vector1(1) = 4.5; vector1(2) = 6.8;
  boost::numeric::ublas::vector<double> vector2(3);
  vector2(0) = 14.6; vector2(1) = 18.5; vector2(2) = 12.8;

  Point dummyPoint(0.0, 0.0, 0.0);
  Robot robot(dummyPoint);
  boost::numeric::ublas::vector<double> crossProd = robot.crossProduct(vector1, vector2);

  EXPECT_NEAR(-68.2, crossProd(0), 0.01);
  EXPECT_NEAR(83.92, crossProd(1), 0.01);
  EXPECT_NEAR(-43.5, crossProd(2), 0.01);
}

TEST(RobotTest, GetTranformationMatrices) {
  std::vector<double> angles = {0.0, M_PI, 0.0, M_PI/4.0, 0.0, 0.0};
  Point dummyPoint(0.0, 0.0, 0.0);
  Robot robot(dummyPoint);
  std::vector<boost::numeric::ublas::matrix<double>> transforms = robot.computeTransformationMatrices(angles);
  boost::numeric::ublas::matrix<double> transform1 = transforms[3];
  boost::numeric::ublas::matrix<double> transform2 = transforms[4];
  boost::numeric::ublas::matrix<double> transform3 = transforms[6];

  double value1 = transform1(0,3);
  double value2 = transform2(2,3);
  double value3 = transform3(3,0);
  double value4 = transform3(3,1);
  double value5 = transform3(3,2);
  double value6 = transform3(3,3);
  double value7 = transform3(2,2);

  EXPECT_NEAR(-480, value1, 1);
  EXPECT_NEAR(2455, value2, 1);
  EXPECT_NEAR(1, value7, 1);
  ASSERT_EQ(0, value3);
  ASSERT_EQ(0, value4);
  ASSERT_EQ(0, value5);
  ASSERT_EQ(1, value6);

}

TEST(RobotTest, FindingJacobian) {
  std::vector<double> jointAngles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Point dummyPoint(0.0, 0.0, 0.0);
  Robot robot(dummyPoint);
  std::vector<Point> jointPositions = robot.computeFk(jointAngles);

  RobotPosition robotPos(jointPositions, jointAngles);
  std::vector<boost::numeric::ublas::matrix<double>> transforms = robot.computeTransformationMatrices(jointAngles);
  boost::numeric::ublas::matrix<double> jacobian = robot.computeJacobian(robotPos, transforms);


  unsigned int requiredDimension = 6;
  ASSERT_EQ(requiredDimension, jacobian.size1());  
  ASSERT_EQ(requiredDimension, jacobian.size2());


  EXPECT_NEAR(0.0, jacobian(0,0), 1);
  EXPECT_NEAR(1250, jacobian(2,1), 1);
  EXPECT_NEAR(0, jacobian(3,4), 1);
  EXPECT_NEAR(0, jacobian(4,1), 1);
  EXPECT_NEAR(-1, jacobian(4,2), 1);
  EXPECT_NEAR(-1, jacobian(5,5), 1);

}

TEST(RobotTest, ComputingIK) {
  Point targetPoint(2050.0, 30.0, -1200.0);
  Point dummyPoint(0.0, 0.0, 0.0);
  Robot robot(dummyPoint);

  Point point1(0.1, 0.2, 0.3);
  Point point2(0.5, 0.6, 0.7);
  Obstacle obstacle1(point1, 0.5);
  Obstacle obstacle2(point2, 1.0);

  std::vector<Obstacle> allObstacles;;
  allObstacles.push_back(obstacle1);
  allObstacles.push_back(obstacle2);

  Environment dummyEnvironment(allObstacles);

  std::vector<RobotPosition> fullPositions = robot.computeIK(targetPoint, dummyEnvironment);
  RobotPosition lastPosition = fullPositions.back();

  std::vector<Point> computedJoints = lastPosition.getJoints();
  Point endEffector = computedJoints.back();
  double xReached = endEffector.getX();
  double yReached = endEffector.getY();
  double zReached = endEffector.getZ();

  EXPECT_NEAR(2050.0, xReached, 0.1);
  EXPECT_NEAR(30.0, yReached, 0.1);
  EXPECT_NEAR(-1200.0, zReached, 0.1);
}

