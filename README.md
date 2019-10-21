# 808X Midterm Project
[![Build Status](https://travis-ci.org/akupferb/ARLpathplanner.svg?branch=master)](https://travis-ci.org/akupferb/ARLpathplanner)
[![Coverage Status](https://coveralls.io/repos/github/akupferb/ARLpathplanner/badge.svg?branch=master)](https://coveralls.io/github/akupferb/ARLpathplanner?branch=master)

Motion/Navigation: Manipulator arm path planner (IK solver)
```
Gotta Catch that Roadrunner!
```


-
## Project Overview

The Road Runner is currently loose in the development space
of ACME Robotics, causing havoc and distraction. ACME
maintains the Fanuc M-900, which is a
6-DOF serial manipulator. Currently, ACME is unable to
effectively provide a trajectory for the robot to catch the Road
Runner when it pauses within the reachable workspace. We are
proposing ACME to develop a path planning software package
for the manipulator, enabling the robotic arm to navigate its way
through the 3-dimensional (3D) workspace. The software will
ensure the robot succeeds in its task, increasing its value for
ACME and saving it from being replaced. In addition, the
software can be re-used for other applications, as the algorithm is
generalizable to different environments.

It allows for user-defined start and end positions of the end effector, 
and will plot a path between the two points if possible.  
In addition, it allows for an arbitrary number of obstacles to be introduced into the environment, 
modeled as spheres of arbitrary size.  Note that the difficulty of pathing will impact performance.  
Examples can be seen both in the GTests and system test. 


## Licensing

This software uses the open-source Boost (and in part, the  MPL2.0) licenses, with small modifications, for the rest of the code (see LICENSE for more information).

## Authors

* Ryan Bates (rjb3vp)*
Ryan is an embedded software engineer with five years of experience at Northrop Grumman.  
Upon graduation in December he aims to find work bringing software processes and skills to the interdisciplinary field of robotics.
* Ari Kupferberg (akupferb)*
Ari is a master's student in robotics and a TA for medical robotics.
* Lydia Zoghbi (lydiazoghbi)*
A master's recipient and a Dean's Fellowship PhD student at the University of Maryland, 
Lydia is a member of IMAPS and is an active researcher with biomedical robotic machine learning, 
including development of an autonomous robotic ultrasound system at MRE.  
When not busy with her studies or research, she enjoys spending time with her boyfriend, cat, and tasty ice cream.

## To Do
* Create unit tests for Robot Class methods
* Complete Inverse Kinematics
* Create Obstacle Space
* Create collision checker functionality

## Operation

The main application first sequentially requests the 3D coordinates 
for the initial end effector position of the robot, 
then the coordinates of the desired end effector destination.  
If the point is unreachable, the user will be prompted to input different end effector coordinates.

## Dependencies/Libraries

Make sure these are downloaded (via apt or otherwise):
  libboost-all-dev
  libeigen3-dev

Note: often these are already included in many Ubuntu distributions by default.

## Run
```
git clone https://github.com/akupferb/ARLpathplanner.git
cd ARLpathplanner
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

## Test

Run test/cpp-test to run the gtest unit tests.
Run systemTest.sh to run the full system test.

## Demo

Our system test acts as our demo, running the full kinematic and path planning chain.  
It is demonstrable through running systemTest.sh

## Notes

Branch 'codingfunctions' used for implementing the functions as defined in the UML.

## Known Issues/Bugs

As Jacobian calculations are very computationally intensive, pathing between distant points can be quite slow (>10 minutes).
This current iteration acts as 

## Doxygen/Documention

Run 'doxygen doxygen_config', then check the html or latex directory for documentation.
TODO: in docs directory

## Backlog/Worklog

Sprint 1:
https://docs.google.com/spreadsheets/d/1D3Op4N3Aqz9G33_2OEKY9-aFLKIj59PVwWTnIGA4hhI/edit?ts=5d9bb868#gid=1860513107
Sprint 2 (Updated):
https://docs.google.com/spreadsheets/d/1qinbfVLNoxSS-iE9CSkSB3Uo1HgffEGqoN9HMs9NbK0/edit?ts=5da79671#gid=1860513107

## Sprint 1 Planning/Review

https://docs.google.com/document/d/1YfSnSUDI8m4mzmk2LF2fXcpP-uipn87FAteoAezY-zQ/edit

## Sprint 2 Planning/Review
https://docs.google.com/document/d/1Jk8p1ZDvSJLDDxCwjpxif-wFkr3EQ5z6IfZfGladbIM/edit
