/*
 *  Distributed under the Boost Software License.
 *  Version 2.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       Robot.hpp
 *  @author     Lydia Zoghbi, Ari Kupferberg
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/15/2019
 *  @version    2.0
 *
 *  @brief      Header file for constructing a Robot class. 
 *
 */

#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_

#include <vector>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/math/constants/constants.hpp>
#include <RobotPosition.hpp>
#include <RobotPath.hpp>
#include <Point.hpp>
#include <PathPlanner.hpp>


/**
 *  @brief      Class for creating a Robot object containing useful information about the system
 */

class Robot {
 private:
   int numberLinks = 6;
   Point initialEEPosition;
   std::vector<double> initialJointAngles = {0, 0, 0, 0, 0, 0};
   std::vector<RobotPath> path;
    std::vector<std::vector<double>> dhParams{
     {770, M_PI/2.0, 750, 0},
     {1050, 0, 0, 0},
     {200, M_PI/2.0, 0, 0},
     {0, -M_PI/2.0, 1705, 0}, 
     {0, M_PI/2.0, 0, 0},
     {0, 0, 325, 0}};
  //std::vector<boost::numeric::ublas::matrix<double>> tTransformations;

 public:

  /**
   *  @brief    Constructor for class Robot 
   *  @param	Point of the robot's end effector position
   *  @return	Instance of robot
   */
   explicit Robot(const Point& startingPos);

  /**
   *  @brief     Compute the intermediate transformation between two DH frames
   *  @param	 DH Table as matrix double
   *  @return	'A' transformation matrix
   */
   boost::numeric::ublas::matrix<double> computeATransform(std::vector<double>);

  /**
   *  @brief    Computing the forward kinematics for the Robot
   *  @param	Vector of Robot's joint angles as double
   *  @return	Vector of Point objects depicting Robot's joint positions
   */
   std::vector<Point> computeFk(std::vector<double>);

  /**
   *  @brief    Compute the Jacobian
   *  @param	Object of Class RobotPosition 
   *  @param	A vector of 2D transformation matrices
   *  @return	Pseudo-inverse of the Jacobian matrix
   */

  /**
   *  @brief    Computing the inverse kinematics for Robot's generated path
   *  @param	Vector of Robot's path
   *  @return	Vector of RobotPosition object containing Points of joint positions
   */
   std::vector<boost::numeric::ublas::matrix<double>> computeTransformationMatrices(std::vector<double> jointAngles);

   boost::numeric::ublas::matrix<double> computeJacobian(RobotPosition robotPosition, std::vector<boost::numeric::ublas::matrix<double>> tTransforms);

  /**
   *  @brief    Computing the inverse kinematics for Robot's generated path
   *  @param	Vector of Robot's path
   *  @return	Vector of RobotPosition object containing Points of joint positions
   */
   std::vector<RobotPosition> computeIK(Point targetPoint, std::vector<boost::numeric::ublas::matrix<double>> tTransforms);

  /**
   *  @brief    Computes cross product of the two input vectors
   *  @param	Vector to compute cross product of (1 of 2)
   *  @param	Vector to compute cross product of (2 of 2)
   *  @return	Vector of the resultant cross product
   */
   boost::numeric::ublas::vector<double> crossProduct(boost::numeric::ublas::vector<double> vector1, boost::numeric::ublas::vector<double> vector2);

  /**
   *  @brief    Computes the Moore-Penrose pseudoinverse
   *  @param	Input matrix to find the pseudoinverse of
   *  @return	Pseudoinverse matrix result
   */
   boost::numeric::ublas::matrix<double> penroseInverseMatrix(boost::numeric::ublas::matrix<double> mat);

};


#endif // INCLUDE_ROBOT_HPP_
