#ifndef INCLUDE_PATHPLANNER_HPP_
#define INCLUDE_PATHPLANNER_HPP_
#include <Point.hpp>
#include <vector>
class PathPlanner {
private:
    double resolution;
public:
    std::vector<Point> findStraightPath(Point startPoint, Point endPoint);

};
#endif // INCLUDE_PATHPLANNER_HPP_
