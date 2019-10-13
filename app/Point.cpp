#include "Point.hpp"

Point::Point(double newX, double newY, double newZ){
   x = newX;
   y = newY;
   z = newZ;
};

double Point::getX() {return x;}
double Point::getY() {return y;}
double Point::getZ() {return z;}
