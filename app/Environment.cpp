/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       Environment.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
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
