#include <gtest/gtest.h>
#include "Obstacle.hpp"

TEST(Obstacle, Constructor) {
   Point point(0.1, 0.2, 0.3);
   Obstacle obstacle(point, 0.5);
   Point secondPoint = obstacle.getCentroid();
   double radius = obstacle.getRadius();

   EXPECT_EQ(0.1, secondPoint.getX());
   EXPECT_EQ(0.2, secondPoint.getY());
   EXPECT_EQ(0.3, secondPoint.getZ());
   EXPECT_EQ(0.5, radius);
}

