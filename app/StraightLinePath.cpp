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

#include <Eigen/Dense>

#include "StraightLinePath.hpp"


StraightLinePath::StraightLinePath() {
}

StraightLinePath::~StraightLinePath() {
}

#include <iostream>
std::vector<Eigen::Matrix4d> StraightLinePath::computePath(
    Eigen::Matrix4d &aStart, Eigen::Matrix4d &aEnd,
    const double &aIncrement) {

  double startX = aStart(0, 3);
  double startY = aStart(1, 3);
  double startZ = aStart(2, 3);
  double endX = aEnd(0, 3);
  double endY = aEnd(1, 3);
  double endZ = aEnd(2, 3);
  double dirX = endX - startX;
  double dirY = endY - startY;
  double dirZ = endZ - startZ;

  std::vector < Eigen::Matrix4d > points;

  for (double i = 0.0; i < 1; i = i + aIncrement) {
    std::cout << i << std::endl;
    double pointX = startX + i * dirX;
    double pointY = startY + i * dirY;
    double pointZ = startZ + i * dirZ;

    Eigen::Matrix4d newMatrix = Eigen::Matrix4d::Zero(4, 4);
    newMatrix(0, 3) = pointX;
    newMatrix(1, 3) = pointY;
    newMatrix(2, 3) = pointZ;

    points.push_back(newMatrix);
  }

  points.push_back(aEnd);

  return points;


}

Eigen::Vector3d StraightLinePath::determineDirection(Eigen::Vector3d &aStart,
                                                     Eigen::Vector3d &aEnd,
                                                const double &aIncrement) {
  Eigen::Vector3d tPnt;
  // Compute the pointing vector
  tPnt = aEnd - aStart;

  // Normalize the pointing vector for a unit vector
  tPnt.normalize();

  // Increment is not needed for this path planner to determine the direction.
  // Assuming no obstacles to avoid.
  (void) aIncrement;

  return tPnt;
}


