/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       Point.hpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Header file for consructing a Point class representing cartesian coordinate points. 
 *
 */

#ifndef INCLUDE_POINT_HPP_
#define INCLUDE_POINT_HPP_

/**
 *  @brief      Class for creating a Point with (x, y, z) coordinates
 */

class Point {
 private:
   double x, y, z;

 public:
  /**
   *  @brief      Creating a Point class
   *  @param	Coordinates of a point as double
   *  @return	Instance of Point
   */
   explicit Point(double newX = 0, double newY = 0, double newZ = 0);

  /**
   *  @brief      Retrieving the x coordinate of a Point
   *  @param	None
   *  @return	Value of x double
   */
   double getX();

  /**
   *  @brief      Retrieving the y coordinate of a Point
   *  @param	None
   *  @return	Value of y double
   */
   double getY();

  /**
   *  @brief      Retrieving the z coordinate of a Point
   *  @param	None
   *  @return	Value of z double
   */
   double getZ();
};


#endif // INCLUDE_POINT_HPP_
