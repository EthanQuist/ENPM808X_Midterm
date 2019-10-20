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

#include <iostream>

#include "Coordinate.hpp"
#include "IJoint.hpp"
#include "InverseKinematics.hpp"
#include "IPathPlanner.hpp"
#include "StraightLinePath.hpp"


#include "Demo.hpp"

void Demo::runDemo() {

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

    //Trajectory Plot Animation
    matplotlibcpp::clf();
    matplotlibcpp::plot(pathX, pathY);
    matplotlibcpp::title("Plotting Trajectory [X-Y Plane]");
    matplotlibcpp::xlabel("X [m]");
    matplotlibcpp::ylabel("Y [m]");
    matplotlibcpp::pause(0.01);

  }

  //Holding Trajectory Plot
  matplotlibcpp::plot(pathX, pathY);
  matplotlibcpp::title("Plotting Trajectory [X-Y Plane]");
  matplotlibcpp::xlabel("X [m]");
  matplotlibcpp::ylabel("Y [m]");
  matplotlibcpp::show();

  //create x-axis for time domain
  std::vector<double> qx;
  int i;
  for (i = 0; i < (int) q1Vec.size(); i++) {
    //std::cout << i << std::endl;
    qx.push_back(i);
}


  std::vector<double> pX;
  std::vector<double> pY;
  std::vector<double> pZ;
  std::vector<double> q1V;
  std::vector<double> q2V;
  std::vector<double> q3V;
  std::vector<double> q4V;
  std::vector<double> q5V;
  std::vector<double> q6V;

  std::vector<double> xi;
  int j = 0;
  for (auto& mat : matrixVec) {

    //path trajectory
    pX.push_back(mat(0, 3));
    pY.push_back(mat(1, 3));
    pZ.push_back(mat(2, 3));

    //inverse kinematics
    std::vector<JointPtr> res;
    res = IKsolver.computeIK(mat);
    JointPtr res1 = res[0];
    JointPtr res2 = res[1];
    JointPtr res3 = res[2];
    JointPtr res4 = res[3];
    JointPtr res5 = res[4];
    JointPtr res6 = res[5];
    double a1 = res1->getConfig();
    double a2 = res2->getConfig();
    double a3 = res3->getConfig();
    double a4 = res4->getConfig();
    double a5 = res5->getConfig();
    double a6 = res6->getConfig();
    q1V.push_back(a1);
    q2V.push_back(a2);
    q3V.push_back(a3);
    q4V.push_back(a4);
    q5V.push_back(a5);
    q6V.push_back(a6);

    xi.push_back(j);
    j++;

    //Joint Angle Plot Animation
    matplotlibcpp::clf();
    matplotlibcpp::subplot(3, 2, 1);
    matplotlibcpp::plot(xi, q1V);
    matplotlibcpp::title("Joint Angle Q1");
    matplotlibcpp::xlabel("time");
    matplotlibcpp::ylabel("Q1 [rad]");
    matplotlibcpp::subplot(3, 2, 2);
    matplotlibcpp::plot(xi, q2V);
    matplotlibcpp::title("Joint Angle Q2");
    matplotlibcpp::xlabel("time");
    matplotlibcpp::ylabel("Q2 [rad]");
    matplotlibcpp::subplot(3, 2, 3);
    matplotlibcpp::plot(xi, q3V);
    matplotlibcpp::title("Joint Angle Q3");
    matplotlibcpp::xlabel("time");
    matplotlibcpp::ylabel("Q3 [rad]");
    matplotlibcpp::subplot(3, 2, 4);
    matplotlibcpp::plot(xi, q4V);
    matplotlibcpp::title("Joint Angle Q4");
    matplotlibcpp::xlabel("time");
    matplotlibcpp::ylabel("Q4 [rad]");
    matplotlibcpp::subplot(3, 2, 5);
    matplotlibcpp::plot(xi, q5V);
    matplotlibcpp::title("Joint Angle Q5");
    matplotlibcpp::xlabel("time");
    matplotlibcpp::ylabel("Q5 [rad]");
    matplotlibcpp::subplot(3, 2, 6);
    matplotlibcpp::plot(xi, q6V);
    matplotlibcpp::title("Joint Angle Q6");
    matplotlibcpp::xlabel("time");
    matplotlibcpp::ylabel("Q6 [rad]");

    std::map<std::string, double> keywords;
    keywords.insert(std::make_pair("hspace", 0.75));
    keywords.insert(std::make_pair("wspace", 0.75));
    matplotlibcpp::subplots_adjust(keywords);

    matplotlibcpp::pause(0.01);


  }

  //Hold Joint Angle plot in final position
  matplotlibcpp::subplot(3, 2, 1);
  matplotlibcpp::plot(xi, q1V);
  matplotlibcpp::title("Joint Angle Q1");
  matplotlibcpp::xlabel("time");
  matplotlibcpp::ylabel("Q1 [rad]");
  matplotlibcpp::subplot(3, 2, 2);
  matplotlibcpp::plot(xi, q2V);
  matplotlibcpp::title("Joint Angle Q2");
  matplotlibcpp::xlabel("time");
  matplotlibcpp::ylabel("Q2 [rad]");
  matplotlibcpp::subplot(3, 2, 3);
  matplotlibcpp::plot(xi, q3V);
  matplotlibcpp::title("Joint Angle Q3");
  matplotlibcpp::xlabel("time");
  matplotlibcpp::ylabel("Q3 [rad]");
  matplotlibcpp::subplot(3, 2, 4);
  matplotlibcpp::plot(xi, q4V);
  matplotlibcpp::title("Joint Angle Q4");
  matplotlibcpp::xlabel("time");
  matplotlibcpp::ylabel("Q4 [rad]");
  matplotlibcpp::subplot(3, 2, 5);
  matplotlibcpp::plot(xi, q5V);
  matplotlibcpp::title("Joint Angle Q5");
  matplotlibcpp::xlabel("time");
  matplotlibcpp::ylabel("Q5 [rad]");
  matplotlibcpp::subplot(3, 2, 6);
  matplotlibcpp::plot(xi, q6V);
  matplotlibcpp::title("Joint Angle Q6");
  matplotlibcpp::xlabel("time");
  matplotlibcpp::ylabel("Q6 [rad]");

  std::map<std::string, double> keywords;
  keywords.insert(std::make_pair("hspace", 0.75));
  keywords.insert(std::make_pair("wspace", 0.75));
  matplotlibcpp::subplots_adjust(keywords);
  matplotlibcpp::show();


}

