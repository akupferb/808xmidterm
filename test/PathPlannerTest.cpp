/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       PathPlannerTest.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright ARL 2019
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Unit test
 *
 */

#include <gtest/gtest.h>
#include "PathPlanner.hpp"

TEST(PathPlanner, FindindStraightPath1) {
   Point startPoint(0.0, 0.0, 0.0);
   Point endPoint(1.0, 0.0, 0.0);
   PathPlanner path;
   std::vector<Point> pathPoints = path.findStraightPath(startPoint, endPoint);

   Point firstPoint = pathPoints[2];

   EXPECT_DOUBLE_EQ(0.3, firstPoint.getX());
   EXPECT_DOUBLE_EQ(0.0, firstPoint.getY());
   EXPECT_DOUBLE_EQ(0.0, firstPoint.getZ());
}

TEST(PathPlanner, FindindStraightPath2) {
   Point startPoint(0.0, 0.0, 0.0);
   Point endPoint(1.0, 1.0, 1.0);
   PathPlanner path;
   std::vector<Point> pathPoints = path.findStraightPath(startPoint, endPoint);

   Point firstPoint = pathPoints[2];

   EXPECT_NEAR(0.1732, firstPoint.getX(),0.01);
   EXPECT_NEAR(0.1732, firstPoint.getY(),0.01);
   EXPECT_NEAR(0.1732, firstPoint.getZ(),0.01);
}

