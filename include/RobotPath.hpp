/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       RobotPath.hpp
 *  @author     Lydia Zoghbi, Ari Kupferberg
 *  @copyright  Copyright ARL 2019
 *  @date       10/16/2019
 *  @version    1.0
 *
 *  @brief      Header file for consructing a robot path 
 *
 */

#ifndef INCLUDE_ROBOTPATH_HPP_
#define INCLUDE_ROBOTPATH_HPP_

#include <vector>
#include <RobotPosition.hpp>

/**
 *  @brief      Class for creating a robot path
 */

class RobotPath {
 private:
   std::vector<RobotPosition> robotPositions;

 public:
  /**
   *  @brief    Constructor for a RobotPath object
   *  @param	Vector of RobotPosition class containing the position of Robot's joints
   *  @return	Instance of RobotPath
   */
   explicit RobotPath(std::vector<RobotPosition>);

  /**
   *  @brief    Retrieve positions from the RobotPosition class
   *  @param	None
   *  @return	Vector of RobotPosition object
   */
   std::vector<RobotPosition> getPositions();

  /**
   *  @brief    Check for collision with obstacles
   *  @param	None
   *  @return	Boolean true for collision with obstacles
   */
   bool existsCollision(Environment environment);
};

#endif // INCLUDE_ROBOTPATH_HPP_
