add_executable(shell-app main.cpp RobotPosition.cpp RobotPath.cpp Robot.cpp Point.cpp PathPlanner.cpp Obstacle.cpp Environment.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    /usr/include/boost
    /usr/include/eigen3/
)
set(CMAKE_BUILD_TYPE Debug)
