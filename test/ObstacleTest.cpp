/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       ObstacleTest.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright ARL 2019
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Unit test
 *
 */

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

