/*
 *  Distributed under the Boost Software License.
 *  Version 1.0 (See accompanying file LICENSE_1_0.txt
 *  or copy at http://www.boost.org/LICENSE_1_0.txt)
 */
/**
 *  @file       main.cpp
 *  @author     Lydia Zoghbi
 *  @copyright  Copyright 2019 ARL. All rights reserved as per license.
 *  @date       10/13/2019
 *  @version    1.0
 *
 *  @brief      Dummy main.cpp file to be populated later on
 *
 */
#include <iostream>
//#include <Eigen/QR>
#include <Robot.hpp>
//using std::cout;
//using std::endl;

int main() {
  
  std::cout << "Running ARL Path Planner" << std::endl;
  double xInput = 0;
  double yInput = 0;
  double zInput = 0;
  std::cout << "Input starting X position: ";
  std::cin >> xInput;
  std::cout << "Input starting Y position: ";
  std::cin >> yInput;
  std::cout << "Input starting Z position: ";
  std::cin >> zInput;
  Point robotStartPoint(xInput, yInput, zInput);
  Robot robot(robotStartPoint);

  while (true) {
    std::cout << "Input destination X position: ";
    std::cin >> xInput;
    std::cout << "Input destination Y position: ";
    std::cin >> yInput;
    std::cout << "Input destination Z position: ";
    std::cin >> zInput;
    Point destinationPoint(xInput, yInput, zInput);


    //std::vector<RobotPosition> positionSequence = robot.computeIK(destinationPoint);

  //  if (positionSequence.size() > 0) {
    //    std::cout << "Path found:" << std::endl;
      //  for (auto robotPosition : positionSequence) {
        //  for (auto angleValue : robotPosition.getAngles()) {
          //  std::cout << angleValue << "	";
          //}
          //for (auto point : robotPosition.getJoints()) {
           // std::cout << point.getX() << ", " << point.getY() << "	";
          //}
       // }
        //break;
    //} else {
      //std::cout << "Destination point unreachable.  Please input a new destination." << std::endl;
    //}
  }
  return 0;
}
