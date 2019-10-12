#ifndef INCLUDE_ROBOTPOSITION_HPP_
#define INCLUDE_ROBOTPOSITION_HPP_
#include <vector>
#include <Point.hpp>
class RobotPosition {
private:
    std::vector<Point> jointPositions ;
    std::vector<double> jointAngles;
};
#endif // INCLUDE_ROBOTPOSITION_HPP_
