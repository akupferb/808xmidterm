# 808X Midterm Project
[![Build Status](https://travis-ci.org/akupferb/ARLpathplanner.svg?branch=master)](https://travis-ci.org/akupferb/ARLpathplanner)
[![Coverage Status](https://coveralls.io/repos/github/akupferb/ARLpathplanner/badge.svg?branch=master)](https://coveralls.io/github/akupferb/ARLpathplanner?branch=master)

Motion/Navigation: Manipulator arm path planner (IK solver)
```
Gotta Catch that Roadrunner!
```


-
## Licensing

This software uses the open-source Boost (and in part, the  MPL2.0) licenses, with small modifications, for the rest of the code (see LICENSE for more information).

## Authors

* Ryan Bates (rjb3vp)*
Is an embedded software engineer of 5+ years experience.
* Ari Kupferberg (akupferb)*
Ari is on another continent.
* Lydia Zoghbi (lydiazoghbi)*
She loves cats.

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
TODO:
Run systemTest.sh to run the full system test.

## Demo
TODO: explain how the demo works
Our system test acts as our demo, running the full kinematic and path planning chain.  
It is demonstrable through running systemTest.sh

## Notes

Branch 'codingfunctions' used for implementing the functions as defined in the UML.

## Known Issues/Bugs

TODO: populate this as tests show us bugs
Nothing, we are geniuses and everything is perfecto.

## Doxygen/Documention

Run 'doxygen doxygen_config', then check the html or latex directory for documentation.

## Backlog/Worklog

Sprint 1:
https://docs.google.com/spreadsheets/d/1D3Op4N3Aqz9G33_2OEKY9-aFLKIj59PVwWTnIGA4hhI/edit?ts=5d9bb868#gid=1860513107
Sprint 2 (Updated):
https://docs.google.com/spreadsheets/d/1qinbfVLNoxSS-iE9CSkSB3Uo1HgffEGqoN9HMs9NbK0/edit?ts=5da79671#gid=1860513107

## Sprint 1 Planning/Review

https://docs.google.com/document/d/1YfSnSUDI8m4mzmk2LF2fXcpP-uipn87FAteoAezY-zQ/edit

## Sprint 2 Planning/Review
https://docs.google.com/document/d/1Jk8p1ZDvSJLDDxCwjpxif-wFkr3EQ5z6IfZfGladbIM/edit
