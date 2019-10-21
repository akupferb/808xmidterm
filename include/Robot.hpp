/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       Robot.hpp
 *  @author     Lydia Zoghbi, Ari Kupferberg
 *  @copyright  Copyright ARL 2019
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
   boost::numeric::ublas::matrix<double> computeATransform(std::vector<double> dhRow);

  /**
   *  @brief    Computing the forward kinematics for the Robot
   *  @param	Vector of Robot's joint angles as double
   *  @return	Vector of Point objects depicting Robot's joint positions
   */
   std::vector<Point> computeFk(std::vector<double> jointAngles);

  /**
   *  @brief    Computing a set of transformation matrices
   *  @param	Vector of Robot's joint angles
   *  @return	Vector of matrices representing the transformation from each joint frame to the base frame
   */
   std::vector<boost::numeric::ublas::matrix<double>> computeTransformationMatrices(std::vector<double> jointAngles);

  /**
   *  @brief    Computing the geometric Jacobian matrix
   *  @param	Robot's current position for performing a cross product
   *  @param    Vector of transformation matrices
   *  @return	The Jacobian matrix
   */
   boost::numeric::ublas::matrix<double> computeJacobian(RobotPosition robotPosition, std::vector<boost::numeric::ublas::matrix<double>> tTransforms);

  /**
   *  @brief    Computing the inverse kinematics for Robot's generated path
   *  @param	The end effector's target position
   *  @param    The environment to check for collisions
   *  @return	Vector of RobotPosition object containing Points of joint positions
   */
   std::vector<RobotPosition> computeIK(Point targetPoint, Environment environment);

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
