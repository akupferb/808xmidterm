//
//  Obstacle.cpp
//  Midterm
//
//  Created by Lydia Zoghbi on 10/11/19.
//  Copyright © 2019 Lydia Zoghbi. All rights reserved.
//

#include "Obstacle.hpp"

Obstacle::Obstacle(Point startCentroid, double startRadius){
   centroid = startCentroid;
   radius = startRadius;
}

Point Obstacle::getCentroid() {return centroid;}
double Obstacle::getRadius(){return radius;}
