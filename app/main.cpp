/*
 *  Distributed under our modified Boost Software License.
 *  Version 1.0 (see accompanying file LICENSE)
 */
/**
 *  @file       main.cpp
 *  @author     Ryan Bates
 *  @copyright  Copyright ARL 2019
 *  @date       10/21/2019
 *  @version    2.0
 *
 *  @brief      Main application to run the path planner
 *
 */

#include <iostream>

#include <Robot.hpp>


int main(int argc, char *argv[]) {
  
  int argumentsProcessed = 1;


  std::cout << "Running ARL Path Planner" << std::endl;
  double xInput = 0;
  double yInput = 0;
  double zInput = 0;
  double radius = 0;

  if (argc > 3) {
    xInput = atof(argv[1]);
    yInput = atof(argv[2]);
    zInput = atof(argv[3]);
    argumentsProcessed = 4;
    std::cout << "Input starting X position as " << xInput << std::endl;
    std::cout << "Input starting Y position as " << yInput << std::endl;
    std::cout << "Input starting Z position as " << zInput << std::endl;
  } else {
    std::cout << "Input starting X position: ";
    std::cin >> xInput;
    std::cout << "Input starting Y position: ";
    std::cin >> yInput;
    std::cout << "Input starting Z position: ";
    std::cin >> zInput;
  }
  Point robotStartPoint(xInput, yInput, zInput);
  Robot robot(robotStartPoint);

  while (true) {
    if (argc > 6) {
      xInput = atof(argv[4]);
      yInput = atof(argv[5]);
      zInput = atof(argv[6]);
      argumentsProcessed = 7;
      std::cout << "Input destination X position as " << xInput << std::endl;
      std::cout << "Input destination Y position as " << yInput << std::endl;
      std::cout << "Input destination Z position as " << zInput << std::endl;
    } else {
      std::cout << "Input destination X position: ";
      std::cin >> xInput;
      std::cout << "Input destination Y position: ";
      std::cin >> yInput;
      std::cout << "Input destination Z position: ";
      std::cin >> zInput;
    }
    Point destinationPoint(xInput, yInput, zInput);



    std::vector<Obstacle> allObstacles;



   if (argc > 10) {
      while (argc > argumentsProcessed) {
        xInput = atof(argv[argumentsProcessed]);
        yInput = atof(argv[argumentsProcessed+1]);
        zInput = atof(argv[argumentsProcessed+2]);
        radius = atof(argv[argumentsProcessed+3]);
        argumentsProcessed += 4;
        Point obstacleCentroid(xInput, yInput, zInput);
        Obstacle obstacle(obstacleCentroid, radius);
        allObstacles.push_back(obstacle);
        std::cout << "Input obstacle X position as " << xInput << std::endl;
        std::cout << "Input obstacle Y position as " << yInput << std::endl;
        std::cout << "Input obstacle Z position as " << zInput << std::endl;
        std::cout << "Input obstacle radius as " << radius << std::endl;
      }
    } else {
      int obstacleCount = 0;
      std::cout << "Input number of obstacles: ";
      std::cin >> obstacleCount;
      // we do not use a C++11 range loop here because there is no collection.
      for (int i = 0; i < obstacleCount; i++) {
      
        std::cout << "Input obstacle " << i << " X position: ";
        std::cin >> xInput;
        std::cout << "Input obstacle " << i << " Y position: ";
        std::cin >> yInput;
        std::cout << "Input obstacle " << i << " Z position: ";
        std::cin >> zInput;
        std::cout << "Input obstacle " << i << " radius: ";
        std::cin >> radius;
      
        Point obstacleCentroid(xInput, yInput, zInput);
        Obstacle obstacle(obstacleCentroid, radius);
        allObstacles.push_back(obstacle);

      }
    }

    Environment dummyEnvironment(allObstacles);


    std::cout << "Analyzing pathing.  Please be patient; this may take a few minutes..." << std::endl;

    std::vector<RobotPosition> positionSequence = robot.computeIK(destinationPoint, dummyEnvironment);
    


    if (positionSequence.size() > 0) {
      std::cout << "Path of " << positionSequence.size() << " positions found:" << std::endl;
      // solution exists
      for (auto robotPosition : positionSequence) {
        for (auto angleValue : robotPosition.getAngles()) {
          std::cout << angleValue << "	";
        }
        for (auto point : robotPosition.getJoints()) {
          std::cout << point.getX() << ", " << point.getY() << "	";
        }
         std::cout << std::endl;
      }
      return 0;
    }
    // else if no solution exists

    if (argc > 6) {
      // if program argument input, can't always request changes, so fail gracefully.
      std::cout << "Destination point unreachable.  Please rerun with new destination/environmental obstacles." << std::endl;
      return 1;
    }
    // else manual input
    std::cout << "Destination point unreachable.  Please input a new destination/environmental obstacles." << std::endl;

  }
  return 0;
}
