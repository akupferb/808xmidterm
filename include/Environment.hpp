/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       Environment.hpp
 *  @author     Lydia Zoghbi, Ari Kupferberg
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/16/2019
 *  @version    1.0
 *
 *  @brief      Header file for defining an environment
 *
 */

#ifndef INCLUDE_ENVIRONMENT_HPP_
#define INCLUDE_ENVIRONMENT_HPP_

#include <vector>
#include <Obstacle.hpp>

/**
 *  @brief      Environment class formed of one or more obstacles
 */

class Environment {
 private:
   std::vector<Obstacle> obstacles;
   
 public:
  /**
   *  @brief      Constructor for an Environment class object containing Obstacle elemements
   *  @param      Vector of Obstacle class objects
   *  @return 	Instance of Environment
   */
   explicit Environment(std::vector<Obstacle>);

  /**
   *  @brief      Obtaining obstacles from the Environment class object
   *  @param      None
   *  @return 	Vector of Obstacle objects
   */
   std::vector<Obstacle> getObstacles();

};


#endif // INCLUDE_ENVIRONMENT_HPP_