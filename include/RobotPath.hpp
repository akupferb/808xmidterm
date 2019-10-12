#ifndef INCLUDE_ROBOTPATH_HPP_
#define INCLUDE_ROBOTPATH_HPP_
#include <vector>
#include <RobotPosition.hpp>
class RobotPath {
private:
    std::vector<RobotPosition> robotPositions;
public:
    std::vector<RobotPosition> getPositions();
    bool existsCollision();
    

};
#endif // INCLUDE_ROBOTPATH_HPP_
