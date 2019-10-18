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
#include <math.h>

#include <gtest/gtest.h>


#include "DHTable.hpp"
#include "Joints.hpp"

// Test that one can build forward Kinematics
TEST(DHTable, FrameOutofBounds) {
  DHTable myTable(6);

  // This should be and error. Do we want to throw and catch exceptions?
  // FIXME: (Yhap) This would be a separate test if so
  // DHTable::Frame aFrame = myTable.modifyFrame(0);
}

TEST(DHTable, BuildFK_PositionTransform) {
  DHTable myTable(3);

  // Create Ptr to Configurable Params
  JointPtr tQ1;
  JointPtr tQ2;
  JointPtr tQ3;

  // Create Constants used in DH Parameters
  double d1 = 2;
  double a2 = 1;
  double a3 = 1;
  double alpha1 = M_PI / 2;

  // Scope the modification so it can't be modified again once it goes out of
  // scope
  {
    // Note all params are set to 0 unless otherwise specified.
    DHTable::Frame tFrame;

    // Frame 1
    tFrame = myTable.modifyFrame(1);

    tFrame.d = PrismaticJoint(d1);
    tFrame.alpha = RevoluteJoint(alpha1);
    tQ1 = std::make_shared<RevoluteJoint>(tFrame.theta);

    // Frame 2
    tFrame = myTable.modifyFrame(2);

    tFrame.a = PrismaticJoint(a2);
    tQ2 = std::make_shared<RevoluteJoint>(tFrame.theta);

    // Frame 3
    tFrame = myTable.modifyFrame(3);

    tFrame.a = PrismaticJoint(a3);
    tQ3 = std::make_shared<RevoluteJoint>(tFrame.theta);
  }

  // Set configuration values for expected output.
  tQ1->setConfig(0);
  tQ2->setConfig(0);
  tQ3->setConfig(0);

  Eigen::Matrix4d result = myTable.getTransform(0, 3);

  // Expected Transform is at position 2 0 2.5 with No rotation about the base
  // frame.
  Eigen::Vector3d position(2, 0, 2.5);
  Eigen::Matrix4d expected;
  expected.setIdentity();
  expected.block<3, 1>(0, 3) = position;

  ASSERT_EQ(result, expected);
}

TEST(DHTable, BuildFK_OrientationTransform) {
  DHTable myTable(3);
  // Create Ptr to Configurable Params
  JointPtr tQ4;
  JointPtr tQ5;
  JointPtr tQ6;

  double d6 = 0.5;
  double alpha4 = -M_PI / 2;
  double alpha5 = M_PI / 2;

  // Scope the modification so it can't be modified again once it goes out of
  // scope
  {
    // Note all params are set to 0 unless otherwise specified.
    DHTable::Frame tFrame;

    // Frame 4 ( Is frame 1 for this test)
    tFrame = myTable.modifyFrame(1);

    tFrame.alpha = RevoluteJoint(alpha4);
    tQ4 = std::make_shared<RevoluteJoint>(tFrame.theta);

    // Frame 5 ( Is frame 2 for this test)
    tFrame = myTable.modifyFrame(2);

    tFrame.alpha = RevoluteJoint(alpha5);
    tQ5 = std::make_shared<RevoluteJoint>(tFrame.theta);

    // Frame 6 ( Is frame 3 for this test)
    tFrame = myTable.modifyFrame(3);

    tFrame.d = PrismaticJoint(d6);
    tQ6 = std::make_shared<RevoluteJoint>(tFrame.theta);
  }

  Eigen::Matrix4d result = myTable.getTransform(0, 3);

  // Expected Transform is the Identity Matrix.
  Eigen::Matrix4d expected;
  expected.setIdentity();

  ASSERT_EQ(result, expected);
}
