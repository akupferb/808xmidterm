#include <gtest/gtest.h>
#include "RobotPosition.hpp"

TEST(RobotPosition, Constructor) {

   Point point1(0.1, 0.2, 0.3);
   Point point2(0.5, 0.6, 0.7);
   Obstacle obstacle1(point1, 0.5);
   Obstacle obstacle2(point2, 1.0);
   
   std::vector<Obstacle> allObstacles;;
   allObstacles.push_back(obstacle1);
   allObstacles.push_back(obstacle2);
   
   Environment dummyEnvironment(allObstacles);

   std::vector<Point> inputJoints;
   inputJoints.push_back(point1);
   inputJoints.push_back(point2);

   std::vector<double> inputAngles;
   inputAngles.push_back(2.0);
   inputAngles.push_back(3.0);

   RobotPosition robotPosition(inputJoints, inputAngles);
   std::vector<Point> retrievedJoints = robotPosition.getJoints();
   std::vector<double> retrievedAngles = robotPosition.getAngles();
  
   Point retrievedJointsPoint = retrievedJoints[1];
   double newX = retrievedJointsPoint.getX();
   double newY = retrievedJointsPoint.getY();
   double newZ = retrievedJointsPoint.getZ();
   double angle1 = retrievedAngles[0];
   double angle2 = retrievedAngles[1];

   ASSERT_EQ(2.0, angle1);
   ASSERT_EQ(3.0, angle2);
   ASSERT_EQ(0.5, newX);
   ASSERT_EQ(0.6, newY);
   ASSERT_EQ(0.7, newZ);
}

TEST(RobotPosition, Boolean) {

   Point point1(0.1, 0.2, 0.3);
   Point point2(0.5, 0.6, 0.7);
   Obstacle obstacle1(point1, 0.5);
   Obstacle obstacle2(point2, 1.0);
   
   std::vector<Obstacle> allObstacles;;
   allObstacles.push_back(obstacle1);
   allObstacles.push_back(obstacle2);
   
   Environment dummyEnvironment(allObstacles);

   std::vector<Point> inputJoints;
   inputJoints.push_back(point1);
   inputJoints.push_back(point2);

   std::vector<double> inputAngles;
   inputAngles.push_back(2.0);
   inputAngles.push_back(3.0);

   RobotPosition robotPosition(inputJoints, inputAngles);
   bool collisionCheck = robotPosition.checkCollision(dummyEnvironment);

   ASSERT_FALSE(collisionCheck);
}

