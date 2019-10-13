#ifndef INCLUDE_PATHPLANNER_HPP_
#define INCLUDE_PATHPLANNER_HPP_
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <Point.hpp>
#include <vector>
#include <cmath>
class PathPlanner {
private:
    double resolution = 0.1;
public:
    std::vector<Point> findStraightPath(Point startPoint, Point endPoint);
};
#endif // INCLUDE_PATHPLANNER_HPP_
