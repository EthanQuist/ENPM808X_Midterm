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

#include <Dense>

#include "StraightLinePath.hpp"


StraightLinePath::StraightLinePath() {
}

StraightLinePath::~StraightLinePath() {
}
//fixme remove after done coding
#include <iostream>
std::vector<Coordinate> StraightLinePath::computePath(
    const Coordinate &aStart, const Coordinate &aEnd,
    const double &aIncrement) {
  // Store end vector value as it will be used frequently.
  Eigen::Vector3d tEndVec = aEnd.getAsVec();

  // Initialize Points with Start Point
  std::vector<Coordinate> points;
  points.push_back(aStart);
  // Compute the distance to the goal from our last point
  double tDistance = (tEndVec - points.back().getAsVec()).norm();

  // Note: Because this is a straight line path the unit vector only needs to be
  // computed once.

  // Determine the direction to the goal
  Eigen::Vector3d unitVec = determineDirection(aStart, aEnd, aIncrement);

  while (tDistance > aIncrement) {

    double tNewX = unitVec(0) + points.back().getX();
    double tNewY = unitVec(1) + points.back().getY();
    double tNewZ = unitVec(2) + points.back().getZ();
    Coordinate tNewPoint(tNewX, tNewY, tNewZ);
    // Put the newly calculated point on the list
    points.push_back(tNewPoint);
    // Recompute the new distance
    tDistance = (tEndVec - points.back().getAsVec()).norm();
  }
  // Finally add the end point
  points.push_back(aEnd);

  return points;
}

Eigen::Vector3d StraightLinePath::determineDirection(const Coordinate &aStart,
                                                const Coordinate &aEnd,
                                                const double &aIncrement) {
  Eigen::Vector3d tPnt;
  // Compute the pointing vector
  tPnt = aEnd.getAsVec() - aStart.getAsVec();

  // Normalize the pointing vector for a unit vector
  tPnt.normalize();

  // Increment is not needed for this path planner to determine the direction.
  // Assuming no obstacles to avoid.
  (void) aIncrement;

  return tPnt;
}
