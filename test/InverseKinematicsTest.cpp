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
  Coordinate inputPoint(3, 0, 2);
  result = IKsolver.computeIK(inputPoint);

  JointPtr tQ1(new RevoluteJoint(0));
  JointPtr tQ2(new RevoluteJoint(-49.9677));
  JointPtr tQ3(new RevoluteJoint(81.0107));
  JointPtr tQ4(new RevoluteJoint(-31.0430));
  JointPtr tQ5(new RevoluteJoint(0));

  std::vector<JointPtr> expected;

  expected.push_back(tQ1);
  expected.push_back(tQ2);
  expected.push_back(tQ3);
  expected.push_back(tQ4);
  expected.push_back(tQ5);

  // Test the number of joints that were output
  ASSERT_EQ(expected.size(), result.size());
  // Test Each element in each matches (in order)
  bool equalJoints = std::equal(expected.begin(), expected.end(),
                                result.begin(), compareConfig);
  ASSERT_TRUE(equalJoints);
}
