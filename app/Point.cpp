/**
 *  @file       Point.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Definitions for Point.hpp
 *
 */

#include "Point.hpp"

Point::Point(double newX, double newY, double newZ) {
   x = newX;
   y = newY;
   z = newZ;
}

double Point::getX() {return x;}
double Point::getY() {return y;}
double Point::getZ() {return z;}
