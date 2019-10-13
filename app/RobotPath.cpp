//
//  RobotPath.cpp
//  Midterm
//
//  Created by Lydia Zoghbi on 10/11/19.
//  Copyright © 2019 Lydia Zoghbi. All rights reserved.
//

#include "RobotPath.hpp"

RobotPath::RobotPath(std::vector<RobotPosition> newRobotPositions){
    robotPositions = newRobotPositions;
}
    std::vector<RobotPosition> RobotPath::getPositions() {return robotPositions;}

    bool RobotPath::existsCollision(){
return true;
}
    
