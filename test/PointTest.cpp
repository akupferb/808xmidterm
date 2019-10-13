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
