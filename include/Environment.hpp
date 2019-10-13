#ifndef INCLUDE_ENVIRONMENT_HPP_
#define INCLUDE_ENVIRONMENT_HPP_
#include <vector>
#include <Obstacle.hpp>
class Environment {
private:
    std::vector<Obstacle> obstacles;
public:
    Environment(std::vector<Obstacle> allObstacles);
    std::vector<Obstacle> getObstacles();

};
#endif // INCLUDE_ENVIRONMENT_HPP_
