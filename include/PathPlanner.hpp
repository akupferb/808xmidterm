/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       PathPlanner.hpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Header file for generating a straight line path
 *
 */

#ifndef INCLUDE_PATHPLANNER_HPP_
#define INCLUDE_PATHPLANNER_HPP_
#include <cmath>
#include <vector>
#include <Point.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>


/**
 *  @brief      Class for generating a straight path between two points
 */

class PathPlanner {
 private:
   double resolution = 0.1;

 public:
  /**
   *  @brief      Finding the coordinates of a straight path with a pre-specified resolution
   *  @param	Point class of a starting point and a Point class of the target end point
   *  @return	Vector of Point classes containing the coordinates of the discretized straight line path
   */
   std::vector<Point> findStraightPath(Point startPoint, Point endPoint);
};


#endif // INCLUDE_PATHPLANNER_HPP_
