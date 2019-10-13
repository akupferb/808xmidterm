#ifndef INCLUDE_OBSTACLE_HPP_
#define INCLUDE_OBSTACLE_HPP_
#include "Point.hpp"

class Obstacle {
private:
    Point centroid;
    double radius;
public:
    Obstacle(Point startCentroid, double startRadius);
    Point getCentroid();
    double getRadius();
};
#endif // INCLUDE_OBSTACLE_HPP_
