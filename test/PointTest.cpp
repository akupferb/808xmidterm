/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       PointTest.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Unit test
 *
 */
#include <gtest/gtest.h>
#include <Point.hpp>

TEST(Point, Constructor) {
   Point point(0.1, 0.2, 0.3);
   double resultX = point.getX();
   double resultY = point.getY();
   double resultZ = point.getZ();
   EXPECT_EQ(0.1, resultX);
   EXPECT_EQ(0.2, resultY);
   EXPECT_EQ(0.3, resultZ);
}
