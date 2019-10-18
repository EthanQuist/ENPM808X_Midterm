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

#include <algorithm>

#include <gtest/gtest.h>

#include "InverseKinematics.hpp"

bool compareConfig(const JointPtr lhs, const JointPtr rhs) {
  return lhs->getConfig() == rhs->getConfig();
}

TEST(InverseKinematics, checkContract) {
  InverseKinematicAcmeArm IKsolver;
  std::vector<JointPtr> result;
  // Using the Coordinates from the Paper
  Eigen::Matrix4d T(4, 4);
  //Transform Matrix - Rotation
  T(0, 0) = 1.0;
  T(0, 1) = 0.0;
  T(0, 2) = 0.0;
  T(1, 0) = 0.0;
  T(1, 1) = 1.0;
  T(1, 2) = 0.0;
  T(2, 0) = 0.0;
  T(2, 1) = 0.0;
  T(2, 2) = 1.0;
  //Transform Matrix - bottom row
  T(3, 0) = 0.0;
  T(3, 1) = 0.0;
  T(3, 2) = 0.0;
  T(3, 3) = 1.0;
  //Transform Matrix - Position
  T(0, 3) = 2.0;
  T(1, 3) = 0.0;
  T(2, 3) = 2.5;
  result = IKsolver.computeIK(T);

  JointPtr tQ1(new RevoluteJoint(0));
  JointPtr tQ2(new RevoluteJoint(0));
  JointPtr tQ3(new RevoluteJoint(0));
  JointPtr tQ4(new RevoluteJoint(1.5708));
  JointPtr tQ5(new RevoluteJoint(1.5708));
  JointPtr tQ6(new RevoluteJoint(-1.5708));

  std::vector<JointPtr> expected;

  expected.push_back(tQ1);
  expected.push_back(tQ2);
  expected.push_back(tQ3);
  expected.push_back(tQ4);
  expected.push_back(tQ5);
  expected.push_back(tQ6);

  // Test the number of joints that were output
  ASSERT_EQ(expected.size(), result.size());
  // Test Each element in each matches (in order)
  JointPtr result1 = result[0];
  double r1 = result1->getConfig();
  double q1 = tQ1->getConfig();
  ASSERT_EQ(r1, q1);

  JointPtr result2 = result[1];
  double r2 = result2->getConfig();
  double q2 = tQ2->getConfig();
  ASSERT_EQ(r2, q2);

  JointPtr result3 = result[2];
  double r3 = result3->getConfig();
  double q3 = tQ3->getConfig();
  ASSERT_EQ(r3, q3);

  JointPtr result4 = result[3];
  double r4 = result4->getConfig();
  double q4 = tQ4->getConfig();
  ASSERT_EQ(r4, q4);

  JointPtr result5 = result[4];
  double r5 = result5->getConfig();
  double q5 = tQ5->getConfig();
  ASSERT_EQ(r5, q5);

  JointPtr result6 = result[5];
  double r6 = result6->getConfig();
  double q6 = tQ6->getConfig();
  ASSERT_EQ(r6, q6);
}
