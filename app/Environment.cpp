//
//  Environment.cpp
//  Midterm
//
//  Created by Lydia Zoghbi on 10/11/19.
//  Copyright © 2019 Lydia Zoghbi. All rights reserved.
//

#include "Environment.hpp"

Environment::Environment(std::vector<Obstacle> allObstacles){
   obstacles = allObstacles;
}

std::vector<Obstacle> Environment::getObstacles() {return obstacles;}
