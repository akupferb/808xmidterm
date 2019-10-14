<<<<<<< HEAD
add_executable(shell-app main.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
=======
add_executable(shell-app main.cpp RobotPosition.cpp RobotPath.cpp Robot.cpp Point.cpp PathPlanner.cpp Obstacle.cpp Environment.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    /usr/include/boost
    #/home/oooo/boost_1_71_0/boost
>>>>>>> 79ea138cbafc4d940b79be1bf366c21a190cd9dd
)
