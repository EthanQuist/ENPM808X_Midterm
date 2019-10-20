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

void compareVecMat(const std::vector<Eigen::Matrix4d> &result,
                   const std::vector<Eigen::Matrix4d> &expected) {
  std::vector<Eigen::Matrix4d>::const_iterator resultIter = result.begin();

  ASSERT_TRUE(result.size() == expected.size());

  for (auto expect : expected) {
    bool closeEnough = expect.isApprox(*resultIter);
    resultIter++;
    ASSERT_TRUE(closeEnough);
  }
}

TEST(StraightLinePath, PathZero) {
  StraightLinePath PathMaker;
  std::vector < Eigen::Matrix4d > result;
  Eigen::Matrix4d start = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d end = Eigen::Matrix4d::Identity();
  end(0, 3) = 0.0;

  result = PathMaker.computePath(start, end, 1);
  std::vector < Eigen::Matrix4d > expected;
  expected.push_back(start);

  expected.push_back(end);
  ASSERT_EQ(expected, result);
}



TEST(StraightLinePath, PathAlongXAxis) {
  StraightLinePath PathMaker;
  double endDist = 5.0;
  std::vector < Eigen::Matrix4d > result;
  Eigen::Matrix4d start = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d end = Eigen::Matrix4d::Identity();
  end(0, 3) = endDist;
  double increment = 1;

  result = PathMaker.computePath(start, end, increment);
  std::vector<Eigen::Matrix4d> expected;

  std::vector<Eigen::Matrix4d>::size_type numIterations = endDist / increment;
  std::vector<Eigen::Matrix4d>::size_type idx;

  for (idx = 0; idx < numIterations; idx++) {
    Eigen::Matrix4d tResult = Eigen::Matrix4d::Identity();
    tResult(0, 3) += increment * idx;
    expected.push_back(tResult);
  }
  expected.push_back(end);

  std::cout << "last expected \n" << expected.back() << std::endl;
  std::cout << "last result \n" << result.back() << std::endl;


  compareVecMat(result, expected);
}



TEST(StraightLinePath, PathAlongYAxis) {
  StraightLinePath PathMaker;
  double endDist = 5.0;
  std::vector < Eigen::Matrix4d > result;
  Eigen::Matrix4d start = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d end = Eigen::Matrix4d::Identity();
  end(1, 3) = endDist;
  double increment = 1;

  result = PathMaker.computePath(start, end, increment);
  std::vector<Eigen::Matrix4d> expected;

  std::vector<Eigen::Matrix4d>::size_type numIterations = endDist / increment;
  std::vector<Eigen::Matrix4d>::size_type idx;

  for (idx = 0; idx < numIterations; idx++) {
    Eigen::Matrix4d tResult = Eigen::Matrix4d::Identity();
    tResult(1, 3) += increment * idx;
    expected.push_back(tResult);
  }
  expected.push_back(end);

  std::cout << "last expected \n" << expected.back() << std::endl;
  std::cout << "last result \n" << result.back() << std::endl;

  compareVecMat(result, expected);
}

TEST(StraightLinePath, PathAlongZAxis) {
  StraightLinePath PathMaker;
  double endDist = 5.0;
  std::vector < Eigen::Matrix4d > result;
  Eigen::Matrix4d start = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d end = Eigen::Matrix4d::Identity();
  end(2, 3) = endDist;

  double increment = 1;
  result = PathMaker.computePath(start, end, increment);
  std::vector<Eigen::Matrix4d> expected;

  std::vector<Eigen::Matrix4d>::size_type numIterations = endDist / increment;
  std::vector<Eigen::Matrix4d>::size_type idx;

  for (idx = 0; idx < numIterations; idx++) {
    Eigen::Matrix4d tResult = Eigen::Matrix4d::Identity();
    tResult(2, 3) += increment * idx;
    expected.push_back(tResult);
  }
  expected.push_back(end);

  std::cout << "last expected \n" << expected.back() << std::endl;
  std::cout << "last result \n" << result.back() << std::endl;


  compareVecMat(result, expected);
}

TEST(StraightLinePath, NonUnitIncrement) {
  StraightLinePath PathMaker;
  double endDist = 5.0;
  std::vector<Eigen::Matrix4d> result;
  Eigen::Matrix4d start = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d end = Eigen::Matrix4d::Identity();
  end(2, 3) = endDist;

  double increment = 0.2;
  result = PathMaker.computePath(start, end, increment);

  std::vector<Eigen::Matrix4d> expected;

  std::vector<Eigen::Matrix4d>::size_type numIterations = endDist / increment;
  std::vector<Eigen::Matrix4d>::size_type idx;

  for (idx = 0; idx < numIterations; idx++) {
    Eigen::Matrix4d tResult = Eigen::Matrix4d::Identity();
    tResult(2, 3) += increment * idx;
    expected.push_back(tResult);
  }
  expected.push_back(end);

  std::cout << "last expected \n" << expected.back() << std::endl;
  std::cout << "last result \n" << result.back() << std::endl;

  compareVecMat(result, expected);
}
