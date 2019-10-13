set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp
    PointTest.cpp
    ObstacleTest.cpp
    EnvironmentTest.cpp
    PathPlannerTest.cpp
    ${CMAKE_SOURCE_DIR}/app/Point.cpp
    ${CMAKE_SOURCE_DIR}/app/Obstacle.cpp
    ${CMAKE_SOURCE_DIR}/app/Environment.cpp
    ${CMAKE_SOURCE_DIR}/app/PathPlanner.cpp
    test.cpp
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)
