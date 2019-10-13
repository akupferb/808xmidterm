//
//  RobotPosition.cpp
//  Midterm
//
//  Created by Lydia Zoghbi on 10/11/19.
//  Copyright © 2019 Lydia Zoghbi. All rights reserved.
//

#include "RobotPosition.hpp"

RobotPosition::RobotPosition(std::vector<Point> newJointPositions, std::vector<double> newJointAngles){
       jointPositions = newJointPositions;
       jointAngles = newJointAngles;
}

bool RobotPosition::checkCollision(Environment environment){
return false;
}
