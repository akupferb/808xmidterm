//
//  PathPlanner.cpp
//  Midterm
//
//  Created by Lydia Zoghbi on 10/11/19.
//  Copyright © 2019 Lydia Zoghbi. All rights reserved.
//
#include "PathPlanner.hpp"
#include <iostream>

std::vector<Point> PathPlanner::findStraightPath(Point startPoint, Point endPoint){
   double xDirectionStart = startPoint.getX();
   double yDirectionStart = startPoint.getY();
   double zDirectionStart = startPoint.getZ();

   double xDirectionEnd = endPoint.getX();
   double yDirectionEnd = endPoint.getY();
   double zDirectionEnd = endPoint.getZ();

   double xDirection = xDirectionEnd - xDirectionStart;
   double yDirection = yDirectionEnd - yDirectionStart;
   double zDirection = zDirectionEnd - zDirectionStart;

   boost::numeric::ublas::vector<double> direction (3);
   direction(0) = xDirection;
   direction(1) = yDirection;
   direction(2) = zDirection;

   double norm = boost::numeric::ublas::norm_2(direction);
   double increment = resolution/norm;

   int size = floor(1/increment);
   std::vector<Point> pathPoints;
   Point cartesianPoint(xDirectionStart + increment*xDirection, yDirectionStart + increment*yDirection, zDirectionStart + increment*zDirection);
   pathPoints.push_back(cartesianPoint);

   for (int i = 1; i< size; i++) {
   Point temporaryPoint = pathPoints[i-1];
   cartesianPoint = Point (temporaryPoint.getX() + increment*xDirection, temporaryPoint.getY() + increment*yDirection, temporaryPoint.getZ() + increment*zDirection);
   pathPoints.push_back(cartesianPoint);
}
   return pathPoints;
}


