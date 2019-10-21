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
 * @file Demo.cpp
 *
 * @brief This File Ties together the functionality provided for the Acme
 * Robot Arm. A straight line path planner is used in conjunction with the
 * robots inverse kinematics solver to determine the corresponding joint
 * configurations along the path.
 */

#include "./matplotlibcpp.h"

#include <algorithm>

#include "Coordinate.hpp"
#include "IJoint.hpp"
#include "InverseKinematics.hpp"
#include "IPathPlanner.hpp"
#include "StraightLinePath.hpp"
#include "Demo.hpp"
#include <iostream>
void Demo::runDemo() {
  StraightLinePath pathMaker;
  InverseKinematicAcmeArm IKsolver;

  Eigen::Matrix4d startMatrix = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d endMatrix = Eigen::Matrix4d::Identity();

  // Define Relevant Parameters.
  double minPlotBorder = 0.01;
  double timeInc = 0.01;

  Coordinate startPos(2.0, 0, 2.5);
  startMatrix.block<3, 1>(0, 3) = startPos.getAsVec();

  Coordinate endPos(0, 2.0, 2.5);
  endMatrix.block<3, 1>(0, 3) = endPos.getAsVec();

  // Note Due the Trajectory being plotted in 2D min and max Z are currently
  // unused.
  double maxX = std::max(startPos.getX(), endPos.getX());
  double maxY = std::max(startPos.getY(), endPos.getY());
  // double maxZ = std::max(startPos.getZ(), endPos.getZ());
  double minX = std::min(startPos.getX(), endPos.getX());
  double minY = std::min(startPos.getY(), endPos.getY());
  // double minZ = std::min(startPos.getZ(), endPos.getZ());

  std::vector<Eigen::Matrix4d> matrixVec;
  matrixVec = pathMaker.computePath(startMatrix, endMatrix, 0.1);
  std::vector<double> pathX;
  std::vector<double> pathY;
  std::vector<double> pathZ;

  for (auto &m : matrixVec) {
    // path trajectory
    pathX.push_back(m(0, 3));
    pathY.push_back(m(1, 3));
    pathZ.push_back(m(2, 3));
  }

  plotData tTrajData;
  tTrajData.x = pathX;
  tTrajData.y = pathY;
  tTrajData.xlimMax = maxX;
  tTrajData.xlimMin = minX;
  tTrajData.ylimMax = maxY;
  tTrajData.ylimMin = minY;
  tTrajData.title = "Plotting Trajectory [X-Y Plane]";
  tTrajData.xlabel = "X [m]";
  tTrajData.ylabel = "Y [m]";
  tTrajData.timeInc = timeInc;
  tTrajData.show = true;
  animatePlot(tTrajData);

  std::vector<double> q1V;
  std::vector<double> q2V;
  std::vector<double> q3V;
  std::vector<double> q4V;
  std::vector<double> q5V;
  std::vector<double> q6V;

  for (auto &mat : matrixVec) {
    // inverse kinematics
    std::vector<JointPtr> res;
    res = IKsolver.computeIK(mat);

    q1V.push_back(res.at(0)->getConfig());
    q2V.push_back(res.at(1)->getConfig());
    q3V.push_back(res.at(2)->getConfig());
    q4V.push_back(res.at(3)->getConfig());
    q5V.push_back(res.at(4)->getConfig());
    q6V.push_back(res.at(5)->getConfig());
  }
  // create x-axis for time domain
  std::vector<double> qx(q1V.size(), 0);
  int i;
  for (auto &tQx : qx) {
    tQx = ++i;
  }

  std::vector<std::vector<double> > allQs;
  allQs.push_back(q1V);
  allQs.push_back(q2V);
  allQs.push_back(q3V);
  allQs.push_back(q4V);
  allQs.push_back(q5V);
  allQs.push_back(q6V);

  std::vector<plotData> subPlots;

  int cnt = 0;
  for (auto &tQ : allQs) {
    cnt++;
    plotData tQData;
    tQData.x = qx;
    tQData.y = tQ;

    tQData.xlimMax = *std::max_element(qx.begin(), qx.end()) + minPlotBorder;
    tQData.xlimMin = *std::min_element(qx.begin(), qx.end()) - minPlotBorder;
    tQData.ylimMax = *std::max_element(tQ.begin(), tQ.end()) + minPlotBorder;
    tQData.ylimMin = *std::min_element(tQ.begin(), tQ.end()) - minPlotBorder;
    std::stringstream ss;
    ss << "Joint Q" << cnt;
    tQData.title = ss.str();
    tQData.xlabel = "Time";
    tQData.ylabel = "Config [rad]";

    subPlots.push_back(tQData);
  }
  animateSubPlot(subPlots, 3, 2);
}

void Demo::animatePlot(const Demo::plotData &aDatum) {
  matplotlibcpp::clf();

  auto tEndXIter = aDatum.x.begin();
  auto tEndYIter = aDatum.y.begin();
  while (tEndXIter != aDatum.x.end() && tEndYIter != aDatum.y.end()) {
    std::vector<double> tAniX(aDatum.x.begin(), ++tEndXIter);
    std::vector<double> tAniY(aDatum.y.begin(), ++tEndYIter);
    Demo::plotData tAniPlotData(aDatum);
    tAniPlotData.x = tAniX;
    tAniPlotData.y = tAniY;
    tAniPlotData.show = false;

    plotDatum(tAniPlotData);
    matplotlibcpp::pause(aDatum.timeInc);

    tEndXIter++;
    tEndYIter++;
  }

  plotDatum(aDatum);
  matplotlibcpp::show();

}

void Demo::animateSubPlot(const std::vector<plotData> &aData, int rows,
                          int cols) {
  matplotlibcpp::clf();

  // Determine how many data points in the data (Assumes at least 1 item in
  // the vector
  auto numElem = aData.begin()->x.size();
  std::vector<double>::size_type tCount;
  for (tCount = 1; tCount < numElem; tCount++) {
    for (auto aDatum = aData.begin(); aDatum != aData.end(); aDatum++) {
      std::vector<double> tAniX(aDatum->x.begin(), aDatum->x.begin() + tCount);
      std::vector<double> tAniY(aDatum->y.begin(), aDatum->y.begin() + tCount);
      Demo::plotData tAniPlotData(*aDatum);
      tAniPlotData.x = tAniX;
      tAniPlotData.y = tAniY;
      tAniPlotData.show = false;

      matplotlibcpp::subplot(rows, cols, aDatum - aData.begin() + 1);
      plotDatum(tAniPlotData);
    }
    std::map<std::string, double> keywords;
    keywords.insert(std::make_pair("hspace", 0.75));
    keywords.insert(std::make_pair("wspace", 0.75));
    matplotlibcpp::subplots_adjust(keywords);
    matplotlibcpp::pause(aData.begin()->timeInc);
  }

  for (auto aDatum = aData.begin(); aDatum != aData.end(); aDatum++) {
    matplotlibcpp::subplot(rows, cols, aDatum - aData.begin() + 1);
    plotDatum(*aDatum);
  }
  std::map<std::string, double> keywords;
  keywords.insert(std::make_pair("hspace", 0.75));
  keywords.insert(std::make_pair("wspace", 0.75));
  matplotlibcpp::subplots_adjust(keywords);
  matplotlibcpp::show();
}

void Demo::plotDatum(const Demo::plotData &aDatum) {
  matplotlibcpp::plot(aDatum.x, aDatum.y);
  matplotlibcpp::xlim(aDatum.xlimMin, aDatum.xlimMax);
  matplotlibcpp::ylim(aDatum.ylimMin, aDatum.ylimMax);
  matplotlibcpp::title(aDatum.title);
  matplotlibcpp::xlabel(aDatum.xlabel);
  matplotlibcpp::ylabel(aDatum.ylabel);
}
