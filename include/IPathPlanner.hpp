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
 * @file IPathPlanner.hpp
*
 * @brief Interface/Abstract Path Planning Class (Defines required functions)
*
 * @author Corbyn Yhap (Driver)
*
 * @copyright Acme Robotics, Ethan Quist, Corbyn Yhap
*/
#pragma once

#include <memory>
#include <vector>

#include "Coordinate.hpp"

class IPathPlanner {
 public:
  /**

   * @brief Method to Compute the 3 Dimensional path from start point to end
   * point by desired increments (or less).

   * @param aStart Coordinate of the Starting Point.

   * @param aEnd Coordinate of the Ending Point.

   * @param aIncrement double. The max distance between path points.

   * @return std::vector<Coordinate> The points corresponding to the path from
   * start to end.

   */
  virtual std::vector<Coordinate> computePath(const Coordinate &aStart,
                                      const Coordinate &aEnd,
                                      const double &aIncrement) = 0;

 protected:
  /**

   * @brief Method to Determine the direction the next point should be in

   * @param aStart Coordinate of the Starting Point.

   * @param aEnd Coordinate of the Ending Point.

   * @param aIncrement double. The max distance between path points.

   * @return Coordinate The unit vector representing the direction of the next
   * point along the path.

   */
  virtual Coordinate determineDirection(const Coordinate &aStart,
                                const Coordinate &aEnd,
                                const double &aIncrement) = 0;
};
// Typedef the pointer for easy external polymorphic use.
typedef std::unique_ptr<IPathPlanner> PathPlanPtr;
