/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       RobotPathTest.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright ARL 2019cp
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Unit test
 *
 */
#include <gtest/gtest.h>
#include "RobotPath.hpp"

TEST(RobotPath, Constructor) {

   Point point1(0.1, 0.2, 0.3);
   Point point2(0.5, 0.6, 0.7);

   std::vector<Point> inputJoints1;
   inputJoints1.push_back(point1);
   inputJoints1.push_back(point2);

   std::vector<double> inputAngles1;
   inputAngles1.push_back(2.0);
   inputAngles1.push_back(3.0);

   RobotPosition robotPosition1(inputJoints1, inputAngles1);

   Point point3(1.2, 1.2, 1.4);
   Point point4(1.8, 1.7, 1.6);

   std::vector<Point> inputJoints2;
   inputJoints2.push_back(point3);
   inputJoints2.push_back(point4);

   std::vector<double> inputAngles2;
   inputAngles2.push_back(5.6);
   inputAngles2.push_back(2.3);

   RobotPosition robotPosition2(inputJoints2, inputAngles2);

   std::vector<RobotPosition> newRobotPositions;
   newRobotPositions.push_back(robotPosition1);
   newRobotPositions.push_back(robotPosition2);

   RobotPath newRobotPath(newRobotPositions);

   std::vector<RobotPosition> retrievedRobotPositions = newRobotPath.getPositions();
   RobotPosition retrievedRobotPosition1 = retrievedRobotPositions[1];

   std::vector<Point> retrievedJoints = retrievedRobotPosition1.getJoints();
   std::vector<double> retrievedAngles = retrievedRobotPosition1.getAngles();

   Point retrievedJointsPoint = retrievedJoints[1];
   double newX = retrievedJointsPoint.getX();
   double newY = retrievedJointsPoint.getY();
   double newZ = retrievedJointsPoint.getZ();
   double angle1 = retrievedAngles[0];
   double angle2 = retrievedAngles[1];

   ASSERT_EQ(5.6, angle1);
   ASSERT_EQ(2.3, angle2);
   ASSERT_EQ(1.8, newX);
   ASSERT_EQ(1.7, newY);
   ASSERT_EQ(1.6, newZ);
}

TEST(RobotPath, Boolean) {

   Point point1(0.1, 0.2, 0.3);
   Point point2(0.5, 0.6, 0.7);

   std::vector<Point> inputJoints1;
   inputJoints1.push_back(point1);
   inputJoints1.push_back(point2);

   std::vector<double> inputAngles1;
   inputAngles1.push_back(2.0);
   inputAngles1.push_back(3.0);

   RobotPosition robotPosition1(inputJoints1, inputAngles1);

   Point point3(1.2, 1.2, 1.4);
   Point point4(1.8, 1.7, 1.6);

   std::vector<Point> inputJoints2;
   inputJoints2.push_back(point3);
   inputJoints2.push_back(point4);

   std::vector<double> inputAngles2;
   inputAngles2.push_back(5.6);
   inputAngles2.push_back(2.3);

   RobotPosition robotPosition2(inputJoints2, inputAngles2);

   std::vector<RobotPosition> newRobotPositions;
   newRobotPositions.push_back(robotPosition1);
   newRobotPositions.push_back(robotPosition2);

   RobotPath newRobotPath(newRobotPositions);

   std::vector<Obstacle> allObstacles;
   Environment dummyEnvironment(allObstacles);

   bool collisionCheck = newRobotPath.existsCollision(dummyEnvironment);

   ASSERT_FALSE(collisionCheck);
}

