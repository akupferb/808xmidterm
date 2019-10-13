#include <gtest/gtest.h>
#include "Environment.hpp"

TEST(Environment, Constructor) {
   Point point1(0.1, 0.2, 0.3);
   Point point2(0.5, 0.6, 0.7);
   Obstacle obstacle1(point1, 0.5);
   Obstacle obstacle2(point2, 1.0);

   Point centroid1 = obstacle1.getCentroid();
   Point centroid2 = obstacle2.getCentroid();
   double radius1 = obstacle1.getRadius();
   double radius2 = obstacle2.getRadius();

   EXPECT_EQ(0.1, point1.getX());
   EXPECT_EQ(0.2, point1.getY());
   EXPECT_EQ(0.3, point1.getZ());
   EXPECT_EQ(0.5, radius1);

   EXPECT_EQ(0.5, point2.getX());
   EXPECT_EQ(0.6, point2.getY());
   EXPECT_EQ(0.7, point2.getZ());
   EXPECT_EQ(1.0, radius2);
}

