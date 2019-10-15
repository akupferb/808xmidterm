/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       Obstacle.hpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Header file for defining an obstacle
 *
 */

#ifndef INCLUDE_OBSTACLE_HPP_
#define INCLUDE_OBSTACLE_HPP_
#include "Point.hpp"

/**
 *  @brief      Obstacle class formed of multiple Point elements with (x, y, z) centroid coordinates and radius (assuming  
 *  obstacle is spherical) 
 */

class Obstacle {
 private:
   Point centroid;
   double radius;
   
 public:
  /**
   *  @brief      Constructor for the class Obstacle 
   *  @param	Point class of obstacle centroid position and corresponding radius
   *  @return	Instance of Obstacle 
   */
   Obstacle(Point startCentroid, double startRadius);

  /**
   *  @brief      Obtaining the centroid of the Obstacle
   *  @param	None
   *  @return	Point class of Obstacle's centroid position (which can subsequently provide the (x, y, z) coordinates
   */
   Point getCentroid();

/**
 *  @brief      Obtaining the radius of the Obstacle
 *  @param	None
 *  @return	Double of Obstacle's radius.
 */
    double getRadius();
};


#endif // INCLUDE_OBSTACLE_HPP_
