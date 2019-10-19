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

TEST(RobotTest, Constructor) {

   
}

TEST(RobotTest, PenroseInverseMatrix) {
  
  Point dummyPoint1(0.0, 0.0, 0.0);
  Robot robot(dummyPoint1);
  boost::numeric::ublas::matrix<double> input(2, 2);
  input(0, 0) = 0;
  input(0, 1) = 1;
  input(1, 0) = 1;
  input(1, 1) = 2;

  boost::numeric::ublas::matrix<double> result = robot.penroseInverseMatrix(input);
  ASSERT_EQ(2, result.size1());  
  ASSERT_EQ(2, result.size2());


  EXPECT_NEAR(-2.0, result(0, 0), 0.01);
  EXPECT_NEAR(1.0, result(0, 1), 0.01);
  EXPECT_NEAR(1.0, result(1, 0), 0.01);
  EXPECT_NEAR(0.0, result(1, 1), 0.01);  
}

