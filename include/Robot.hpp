#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_
#include <vector>
#include <RobotPosition.hpp>
#include <RobotPath.hpp>
#include <Point.hpp>

class Robot {
private:
    int linkCount;
    std::vector<double> linkLengths;
    Point initialEEPosition;
    RobotPosition currentRobotPosition, goalPosition;
    std::vector<RobotPath> path;
public:
    Robot(Point startingEEPosition);
    std::vector<Point> computeFK(std::vector<double> jointAngles);
    std::vector<std::vector<RobotPosition>> computeIK(Point endEffectorPosition);
};
#endif // INCLUDE_ROBOT_HPP_
