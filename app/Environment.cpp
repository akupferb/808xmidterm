/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       Environment.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright ARL 2019
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Definitions for Environment.hpp
 *
 */

#include "Environment.hpp"

Environment::Environment(std::vector<Obstacle> allObstacles) {
  obstacles = allObstacles;
}

std::vector<Obstacle> Environment::getObstacles() {return obstacles;}
