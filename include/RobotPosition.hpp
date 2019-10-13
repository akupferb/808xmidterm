#ifndef INCLUDE_ROBOTPOSITION_HPP_
#define INCLUDE_ROBOTPOSITION_HPP_
#include <vector>
#include <Environment.hpp>

class RobotPosition {
private:
    std::vector<Point> jointPositions ;
    std::vector<double> jointAngles;
public:
    RobotPosition(std::vector<Point> newJointPositions, std::vector<double> newJointAngles);
    bool checkCollision(Environment environment);
};
#endif // INCLUDE_ROBOTPOSITION_HPP_
