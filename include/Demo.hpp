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
 * @file Demo.hpp
 *
 * @brief The class header for our Demo Class
 *
 * @author Ethan Quist (driver) and Corbyn Yhap (Navigator)
 *
 * @copyright Acme Robotics, Ethan Quist, Corbyn Yhap
 */
#pragma once

#include <cmath>
#include <memory>
#include <vector>
#include <string>

#include "Coordinate.hpp"
#include "DHTable.hpp"

class Demo {
 public:
  /**

   * @brief Demo Class constructor

   * @param none

   * @return none

   */
  Demo();

  /**

   * @brief Demo Class Destructor

   * @param none

   * @return none

   */
  ~Demo();

  /**

   * @brief Method to run the demonstration of our IK Solver
   *

   * @param none

   * @return none
   *

   */
  void runDemo();


 private:
  DHTable AcmeArmFK;
  std::vector<JointPtr> jointQs;
  struct plotData {
    std::vector<double> x;
    std::vector<double> y;
    std::string title;
    std::string xlabel;
    std::string ylabel;
    double xlimMin;
    double xlimMax;
    double ylimMin;
    double ylimMax;
    double timeInc;
  };

  /**

   * @brief Function to aid in the animation of Joint configuration plots.
   * Prepares the Data for plotting.

   * @param plotData Takes the plotData struct with imporant info about what and
   * how to plot.

   * @return none
   *

   */
  void animatePlot(const plotData &aDatum);

  /**

   * @brief Function to aid in the animation of Joint configuration subplots.
   * Prepares the Data for plotting.

   * @param std::vector<plotData> Takes multiple plotData structs in order to
   * make multiple subplots.

   * @return none
   *

   */
  void animateSubPlot(const std::vector<plotData> &aData, int, int);

  /**

   * @brief Function to definte the parameters necessary to compute the forward
   * kinematics of the arm.

   * @param none

   * @return none

   */
  void defineDH();

  /**

   * @brief Function to actually plot the data.

   * @param plotData.

   * @return none

   */
  void plotDatum(const plotData &aDatum);

  /**

   * @brief Converts Joint Configurations into Frame Positions for additional
   * plotting demo file.

   * @param std::vector<JointPtr> The vector of joints with the current joint
   * configuration.

   * @return The vector of all joint positions.

   */
  std::vector<Coordinate> getJointPos(const std::vector<JointPtr>&);

  /**

   * @brief Originally intended to plot the Robot Joint positions in 3D. Due to
   * the limitations of matplotlib-cpp the data required to plot is written t a
   * file in this function. That file is later read by a python script and
   * plotted using pythons matplotlib.

   * @param std::vector<Coordinate> The vector of Positions of every joint in
   * the configuration.

   * @return none

   */
  void plotRobot(const std::vector<Coordinate>&);
};
