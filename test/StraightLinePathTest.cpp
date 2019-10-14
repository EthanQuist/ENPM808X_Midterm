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

#include <gtest/gtest.h>

#include "StraightLinePath.hpp"

TEST(StraightLinePath, PathAlongXAxis) {
  StraightLinePath PathMaker;
  std::vector<Coordinate> result;
  Coordinate start(0, 0, 0);
  Coordinate end(5, 0, 0);
  result = PathMaker.computePath(start, end, 1);
  std::vector<Coordinate> expected;
  expected.push_back(start);
  expected.push_back(Coordinate(1, 0, 0));
  expected.push_back(Coordinate(2, 0, 0));
  expected.push_back(Coordinate(3, 0, 0));
  expected.push_back(Coordinate(4, 0, 0));
  expected.push_back(end);
  // Test the X axis
  ASSERT_EQ(expected, result);
}

TEST(StraightLinePath, PathAlongYAxis) {
  StraightLinePath PathMaker;
  std::vector<Coordinate> result;
  Coordinate start(0, 0, 0);
  Coordinate end(0, 5, 0);
  result = PathMaker.computePath(start, end, 1);
  std::vector<Coordinate> expected;
  expected.push_back(start);
  expected.push_back(Coordinate(0, 1, 0));
  expected.push_back(Coordinate(0, 2, 0));
  expected.push_back(Coordinate(0, 3, 0));
  expected.push_back(Coordinate(0, 4, 0));
  expected.push_back(end);
  // Test the Y axis
  ASSERT_EQ(expected, result);
}

TEST(StraightLinePath, PathAlongZAxis) {
  StraightLinePath PathMaker;
  std::vector<Coordinate> result;
  Coordinate start(0, 0, 0);
  Coordinate end(0, 0, 5);
  result = PathMaker.computePath(start, end, 1);
  std::vector<Coordinate> expected;
  expected.push_back(start);
  expected.push_back(Coordinate(0, 0, 1));
  expected.push_back(Coordinate(0, 0, 2));
  expected.push_back(Coordinate(0, 0, 3));
  expected.push_back(Coordinate(0, 0, 4));
  expected.push_back(end);
  // Test the Z axis
  ASSERT_EQ(expected, result);
}

// TODO(Yhap): Consider adding a Diagonal Straight line Path.
