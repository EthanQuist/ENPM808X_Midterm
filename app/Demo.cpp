/* Copyright (c) 2019, Acme Robotics, Ethan Quist, Corbyn Yhap
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "matplotlibcpp.h"
#include <Coordinate.hpp>
#include <IJoint.hpp>
#include <InverseKinematics.hpp>
#include <IPathPlanner.hpp>
#include <StraightLinePath.hpp>

#include "Demo.hpp"

// namespace plt = matplotlibcpp;

void Demo::runDemo() {
  /*
  int t = 1000;
  int i = 0;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  for (i = 0; i < t; i++) {
    x.push_back(i);
    y.push_back(sin(3.14 * i / 180));
    if (i % 10 == 0) {
      matplotlibcpp::clf();
      matplotlibcpp::plot(x, y);
      matplotlibcpp::title("Trajectory Demo");
      matplotlibcpp::pause(0.01);
    }
  }
   */

  StraightLinePath pathMaker;
  InverseKinematicAcmeArm IKsolver;

  Eigen::Matrix4d startMatrix(4, 4);
  Eigen::Matrix4d endMatrix(4, 4);
  startMatrix(0, 0) = 1.0;
  startMatrix(0, 1) = 0.0;
  startMatrix(0, 2) = 0.0;
  startMatrix(1, 0) = 0.0;
  startMatrix(1, 1) = 1.0;
  startMatrix(1, 2) = 0.0;
  startMatrix(2, 0) = 0.0;
  startMatrix(2, 1) = 0.0;
  startMatrix(2, 2) = 1.0;
  //Transform Matrix - bottom row
  startMatrix(3, 0) = 0.0;
  startMatrix(3, 1) = 0.0;
  startMatrix(3, 2) = 0.0;
  startMatrix(3, 3) = 1.0;
  //Transform Matrix - Position
  startMatrix(0, 3) = 2.0;
  startMatrix(1, 3) = 0.0;
  startMatrix(2, 3) = 2.5;

  endMatrix(0, 0) = 1.0;
  endMatrix(0, 1) = 0.0;
  endMatrix(0, 2) = 0.0;
  endMatrix(1, 0) = 0.0;
  endMatrix(1, 1) = 1.0;
  endMatrix(1, 2) = 0.0;
  endMatrix(2, 0) = 0.0;
  endMatrix(2, 1) = 0.0;
  endMatrix(2, 2) = 1.0;
  //Transform Matrix - bottom row
  endMatrix(3, 0) = 0.0;
  endMatrix(3, 1) = 0.0;
  endMatrix(3, 2) = 0.0;
  endMatrix(3, 3) = 1.0;
  //Transform Matrix - Position
  endMatrix(0, 3) = 0.0;
  endMatrix(1, 3) = 2.0;
  endMatrix(2, 3) = 2.5;

  std::vector < Eigen::Matrix4d > matrixVec;
  matrixVec = pathMaker.computePath(startMatrix, endMatrix, 0.1);
  std::vector<double> pathX;
  std::vector<double> pathY;
  std::vector<double> pathZ;
  std::vector<double> q1Vec;
  std::vector<double> q2Vec;
  std::vector<double> q3Vec;
  std::vector<double> q4Vec;
  std::vector<double> q5Vec;
  std::vector<double> q6Vec;

  for (auto& m : matrixVec) {
    //path trajectory
    pathX.push_back(m(0, 3));
    pathY.push_back(m(1, 3));
    pathZ.push_back(m(2, 3));

    //inverse kinematics
    std::vector<JointPtr> result;
    result = IKsolver.computeIK(m);
    JointPtr result1 = result[0];
    JointPtr result2 = result[1];
    JointPtr result3 = result[2];
    JointPtr result4 = result[3];
    JointPtr result5 = result[4];
    JointPtr result6 = result[5];
    double angle1 = result1->getConfig();
    double angle2 = result2->getConfig();
    double angle3 = result3->getConfig();
    double angle4 = result4->getConfig();
    double angle5 = result5->getConfig();
    double angle6 = result6->getConfig();
    q1Vec.push_back(angle1);
    q2Vec.push_back(angle2);
    q3Vec.push_back(angle3);
    q4Vec.push_back(angle4);
    q5Vec.push_back(angle5);
    q6Vec.push_back(angle6);

    //active plot
    //matplotlibcpp::clf();
    //matplotlibcpp::plot(pathX, pathY);
    //matplotlibcpp::title("Plotting Trajectory");
    //matplotlibcpp::pause(0.01);

  }

  matplotlibcpp::plot(pathX, pathY);
  matplotlibcpp::title("Plotting Trajectory");
  matplotlibcpp::show();

  std::cout << q1Vec.size() << std::endl;

  std::vector<double> qx;
  qx.push_back(1);
  qx.push_back(2);
  qx.push_back(3);
  qx.push_back(4);
  qx.push_back(5);
  qx.push_back(6);
  qx.push_back(7);
  qx.push_back(8);
  qx.push_back(9);
  qx.push_back(10);
  qx.push_back(11);
  qx.push_back(12);

  matplotlibcpp::named_plot("Joint 1", qx, q1Vec);
  matplotlibcpp::named_plot("Joint 2", qx, q2Vec);
  matplotlibcpp::named_plot("Joint 3", qx, q3Vec);
  matplotlibcpp::named_plot("Joint 4", qx, q4Vec);
  matplotlibcpp::named_plot("Joint 5", qx, q5Vec);
  matplotlibcpp::named_plot("Joint 6", qx, q6Vec);
  matplotlibcpp::title("Plotting Trajectory");
  matplotlibcpp::legend();
  matplotlibcpp::show();



  /*
   //plotting joint angles vs time
   for (int i = 0; i < q1Vec.size(); i++) {
   matplotlibcpp::clf();
   matplotlibcpp::named_plot("Q1", i, q1Vec[i]);
   matplotlibcpp::named_plot("Q2", i, q1Vec[i]);
   matplotlibcpp::named_plot("Q3", i, q1Vec[i]);
   matplotlibcpp::named_plot("Q4", i, q1Vec[i]);
   matplotlibcpp::named_plot("Q5", i, q1Vec[i]);
   matplotlibcpp::named_plot("Q6", i, q1Vec[i]);
   matplotlibcpp::title("Joint Angles");
   matplotlibcpp::pause(0.01);

   }
   */



}

