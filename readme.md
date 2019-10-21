# ENPM808X Midterm
[![Build Status](https://travis-ci.org/EthanQuist/ENPM808X_Midterm.svg?branch=master)](https://travis-ci.org/EthanQuist/ENPM808X_Midterm)
[![Coverage Status](https://coveralls.io/repos/github/EthanQuist/ENPM808X_Midterm/badge.svg?branch=master)](https://coveralls.io/github/EthanQuist/ENPM808X_Midterm?branch=master)
---

## Overview

This repository serves as the Inverse Kinematic solver for the Acme Robotics mobile robot manipulator arm. The six degree of freedom manipulator arm requires this software to calculate the inverse kinematics, specifically the required joint angles for a desired coordinate position. The input to the system will be a desired [X Y Z] coordinate position in the world frame of the mobile robot and the system will output a vector of 6 joint angles that can then be used by the manipulator arm's controller. The software will compute the vector of 6 joint angles for a series of small movements along a trajectory. The trajectory is built by generating a discretized linear path from the current coordinate position to the goal position. These discretized points along the path are used to find the exact joint angles for each smaller movement along the trajectory. 

The IK solver integrates the software with the other components of the robot. As the robot moves it constantly scans the environment using an array of optical sensors. When the cameras detect an object that requires intervention from the manipulator arm it calculates the position of the object calibrated to the base frame of the manipulator arm. This provides easier calculations for the inverse kinematics as the inputs to the IK solver software are the cartesian coordinates [X,Y,Z]T of the grasping location. Our software then computes the necessary joint angles and output them to the PCB controlling the motors of the manipulator arm. 

In order to effectively store and modify the position of the end effector the software takes advantage of the Denavit-Hartenberg (DH) Convention to describe the system using DH parameters. These parameters are used to create a Transformation Matrix. The software implements classes to manage the responsibilities of a Transformation Matrix as well as a class that is responsible for taking DH parameters as input and producing a Transformation Matrix as output. These equations are used to transform the current joint configuration into a current end effector position. The software implements a class that takes the current and desired positions as input and outputs all of the interim end effector positions to accomplish that path. Finally, these end effector positions rely on an inverse kinematics class to compute the corresponding joint configurations.

## Personnel

The two programmers for this project are Ethan Quist and Corbyn Yhap. Currently both are aquiring their Masters in Engineering for Robotics at University of Maryland, College Park. Both Ethan and Corbyn worked as Engineers in Industry prior to returning to school for their Masters. While collaborating to produce this software the two worked as a team dividing up the work and taking turns operating as the driver or navigator for the section of code. 

## License

This software operates with a 3 clause BSD 2.0 license and can be found inside the file LICENSE. 

## Travis and Coveralls

Badges for Travis and Coveralls are located at the top of the readme file. Additional information on building the software to test for coverage is shown below. 

```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.

## AIP Backlog

The google sheet for our AIP backlog is linked below:

https://docs.google.com/spreadsheets/d/14NH808N-Kye4lGJhj0COwV8fRF2zoUQIuAs4Zp_pyWk/edit?usp=sharing

## Sprint Planning Notes

The google document for our Sprint Iteration Review is linked below This document contains our four main team meeting notes, planning for iteration 1, review of iteration 1, planning of iteration 2, and review of iteration 2:

https://docs.google.com/document/d/1Dd6p2RdcRbDvNMnmmexAMOpVTrpyoYmNtbwyHA9CDzw/edit?usp=sharing


## Operation

In order to clone and run this software follow the below steps:

```
git clone --recursive https://github.com/EthanQuist/ENPM808X_Midterm.git
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

These steps will allow for the repository to be cloned remotely. The steps include building the project as well as running the unit tests and main program which will run our demonstration. 

Running ./app/shell-app in the build folder will run out demonstration. We are using matplotlib (with a c++ wrapper) to display the trajectory of our desired path as well as the changes in all 6 joints as the robot traverses the trajectory to go from initial point to end point. 

## Demo

The demo is ran from main.cpp source code where a window will display the trajectory of the end-effector. This window is an animation from the start position to the end position. Once the animation finishes please click the x on the window to allow the code to run the second part of the demonstration. Part 2 of the demo will display all 6 joint angles and their changes as the robot moves along the path planning trajectory. When the animation ends click on the x of the window and the demo will finish. 


## Dependencies

The demonstration requires the use of matplotlibcpp which is a c++ wrapper for a python library commonly used for plotting graphs and generating graphical windows. Because the demonstration uses a python library it is required to have python on any machine running the software. The simple command line shown below will ensure the correct version of python libraries are installed:

```
sudo apt-get install python-matplotlib python-numpy python2.7-dev
```

Our code depends on code from two different third party locations. The first is Eigen a linear algebra math library. The second is a c++ wrapper for matplotlib to display plots in windows. The necessary libraries from Eigen and matplotlib can be downloaded using the script below. This script will also place all necessary dependency code in the correct folder locations dictated by our Cmakelist. 

```
cd ../
mkdir ThirdParty
mkdir ThirdParty/eigen3
cd -

wget  http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2
tar -xjf 3.3.7.tar.bz2
mv eigen-eigen-323c052e1731/Eigen ../ThirdParty/eigen3/.

git clone https://github.com/lava/matplotlib-cpp.git
mv matplotlib-cpp ../ThirdParty
```


## Known Issues and Bugs

The only known issues that could arise while running our code will come incorrectly downloading the dependancies. The code above will provide every step necessary to download the third party software as well as place each dependancy in the correct location. 

## How to build and run

The steps to build and run from the command prompt are listed above in the "Operation" section. If you choose to build and run from Eclipse the instructions are below:

To build the project, in Eclipse, unfold boilerplate-eclipse project in Project Explorer,
unfold Build Targets, double click on "all" to build all projects.

To run:

1. In Eclipse, right click on the boilerplate-eclipse in Project Explorer,
select Run As -> Local C/C++ Application

2. Choose the binaries to run (e.g. shell-app, cpp-test for unit testing)


## How to generate doxygen documentation

Doxygen documenation has been generated and included in the "docs" subfolder. These doxygen files were generated through the Doxywizard software. 

Follow the steps on the Doxygen Downloads page to download the correct doxygen files for your system:
http://www.doxygen.nl/download.html

With Doxywizard installed you can generate the documents through the following steps:

1. Specify the working directory from which doxygen will run
2. Name the Porject
3. Specify the directory to scan for source code
4. Specify the directory where doxygen should put the generated documentation
5. Under Expert/Build adjust settings for types of files to generate documenation on
6. Under Run click "Run Doxygen" - the documentation will generate in specified location


## Plugins

- CppChEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.


- Google C++ Sytle

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

- Git

    It is possible to manage version control through Eclipse and the git plugin, but it typically requires creating another project. If you're interested in this, try it out yourself and contact me on Canvas.
    
    
## License

Copyright (c) 2019, Ethan Quist, Corbyn Yhap, Acme Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
