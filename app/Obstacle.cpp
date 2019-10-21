/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       Obstacle.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright ARL 2019
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Definitions for Obstacle.hpp
 *
 */

#include "Obstacle.hpp"

Obstacle::Obstacle(Point startCentroid, double startRadius) {
  centroid = startCentroid;
  radius = startRadius;
}

Point Obstacle::getCentroid() {return centroid;}
double Obstacle::getRadius() {return radius;}
