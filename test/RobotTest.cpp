/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       RobotTest.cpp
 *  @author     Ryan Bates
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/17/2019
 *  @version    1.0
 *
 *  @brief      Unit test
 *
 */
#include <gtest/gtest.h>
#include "Robot.hpp"
#include <iostream>

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
  ASSERT_EQ(3, result.size1());  
  ASSERT_EQ(3, result.size2());


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
  EXPECT_NEAR(2020, xEEPoint, 5); 
  EXPECT_NEAR(0.0, yEEPoint, 5); 
  EXPECT_NEAR(-1280, zEEPoint, 5);  
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


