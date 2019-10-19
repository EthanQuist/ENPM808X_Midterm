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

bool AreSameMatrix4d(Eigen::Matrix4d A, Eigen::Matrix4d B) {
  double epsilon = 0.00001;
  bool equal = true;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      double error = fabs(A(i, j) - B(i, j));
      if (error > epsilon) {
        equal = false;
      }
    }
  }
  return equal;
}


TEST(StraightLinePath, PathZero) {
  StraightLinePath PathMaker;
  std::vector < Eigen::Matrix4d > result;
  Eigen::Matrix4d start = Eigen::Matrix4d::Zero(4, 4);
  Eigen::Matrix4d end = Eigen::Matrix4d::Zero(4, 4);
  end(0, 3) = 0.0;

  result = PathMaker.computePath(start, end, 1);
  std::vector < Eigen::Matrix4d > expected;
  expected.push_back(start);

  expected.push_back(end);
  ASSERT_EQ(expected, result);
}



TEST(StraightLinePath, PathAlongXAxis) {
  StraightLinePath PathMaker;
  std::vector < Eigen::Matrix4d > result;
  Eigen::Matrix4d start = Eigen::Matrix4d::Zero(4, 4);
  Eigen::Matrix4d end = Eigen::Matrix4d::Zero(4, 4);
  end(0, 3) = 5.0;

  result = PathMaker.computePath(start, end, 0.2);
  std::vector < Eigen::Matrix4d > expected;
  expected.push_back(start);

  Eigen::Matrix4d result1 = Eigen::Matrix4d::Zero(4, 4);
  result1(0, 3) = 1.0;
  expected.push_back(result1);

  Eigen::Matrix4d result2 = Eigen::Matrix4d::Zero(4, 4);
  result2(0, 3) = 2.0;
  expected.push_back(result2);

  Eigen::Matrix4d result3 = Eigen::Matrix4d::Zero(4, 4);
  result3(0, 3) = 3.0;
  expected.push_back(result3);

  Eigen::Matrix4d result4 = Eigen::Matrix4d::Zero(4, 4);
  result4(0, 3) = 4.0;
  expected.push_back(result4);

  expected.push_back(end);
  // Test the X axis

  std::cout << "expected[3] \n" << expected[3] << std::endl;
  std::cout << "result[3] \n" << result[3] << std::endl;

  ASSERT_EQ(AreSameMatrix4d(expected[0], result[0]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[1], result[1]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[2], result[2]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[3], result[3]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[4], result[4]), true);

}



TEST(StraightLinePath, PathAlongYAxis) {
  StraightLinePath PathMaker;
  std::vector < Eigen::Matrix4d > result;
  Eigen::Matrix4d start = Eigen::Matrix4d::Zero(4, 4);
  Eigen::Matrix4d end = Eigen::Matrix4d::Zero(4, 4);
  end(1, 3) = 5.0;

  result = PathMaker.computePath(start, end, 0.2);
  std::vector < Eigen::Matrix4d > expected;
  expected.push_back(start);

  Eigen::Matrix4d result1 = Eigen::Matrix4d::Zero(4, 4);
  result1(1, 3) = 1.0;
  expected.push_back(result1);

  Eigen::Matrix4d result2 = Eigen::Matrix4d::Zero(4, 4);
  result2(1, 3) = 2.0;
  expected.push_back(result2);

  Eigen::Matrix4d result3 = Eigen::Matrix4d::Zero(4, 4);
  result3(1, 3) = 3.0;
  expected.push_back(result3);

  Eigen::Matrix4d result4 = Eigen::Matrix4d::Zero(4, 4);
  result4(1, 3) = 4.0;
  expected.push_back(result4);

  expected.push_back(end);
  // Test the X axis

  std::cout << "expected[3] \n" << expected[3] << std::endl;
  std::cout << "result[3] \n" << result[3] << std::endl;

  ASSERT_EQ(AreSameMatrix4d(expected[0], result[0]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[1], result[1]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[2], result[2]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[3], result[3]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[4], result[4]), true);

}

TEST(StraightLinePath, PathAlongZAxis) {
  StraightLinePath PathMaker;
  std::vector < Eigen::Matrix4d > result;
  Eigen::Matrix4d start = Eigen::Matrix4d::Zero(4, 4);
  Eigen::Matrix4d end = Eigen::Matrix4d::Zero(4, 4);
  end(2, 3) = 5.0;

  result = PathMaker.computePath(start, end, 0.2);
  std::vector < Eigen::Matrix4d > expected;
  expected.push_back(start);

  Eigen::Matrix4d result1 = Eigen::Matrix4d::Zero(4, 4);
  result1(2, 3) = 1.0;
  expected.push_back(result1);

  Eigen::Matrix4d result2 = Eigen::Matrix4d::Zero(4, 4);
  result2(2, 3) = 2.0;
  expected.push_back(result2);

  Eigen::Matrix4d result3 = Eigen::Matrix4d::Zero(4, 4);
  result3(2, 3) = 3.0;
  expected.push_back(result3);

  Eigen::Matrix4d result4 = Eigen::Matrix4d::Zero(4, 4);
  result4(2, 3) = 4.0;
  expected.push_back(result4);

  expected.push_back(end);
  // Test the X axis

  std::cout << "expected[3] \n" << expected[3] << std::endl;
  std::cout << "result[3] \n" << result[3] << std::endl;

  ASSERT_EQ(AreSameMatrix4d(expected[0], result[0]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[1], result[1]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[2], result[2]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[3], result[3]), true);
  ASSERT_EQ(AreSameMatrix4d(expected[4], result[4]), true);

}


// TODO(Yhap): Consider adding a Diagonal Straight line Path.
