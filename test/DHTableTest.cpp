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
#include<iostream>

#include <gtest/gtest.h>
#include <IJoint.hpp>


#include "DHTable.hpp"


TEST(DHTable, ModifyFrames) {
  DHTable myTable;
  // Create Ptr to Configurable Params
  JointPtr tQ4;
  JointPtr tQ5;
  JointPtr tQ6;

  double d6 = 0.5;
  double alpha4 = -M_PI / 2;
  double alpha5 = M_PI / 2;


  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame4;

  tFrame4.alpha->setConfig(alpha4);
  tQ4 = tFrame4.theta;

  // Frame 4 ( Is frame 1 for this test)
  myTable.addFrame(tFrame4);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame5;

  tFrame5.alpha->setConfig(alpha5);
  tQ5 = tFrame5.theta;

  // Frame 5 ( Is frame 2 for this test)
  myTable.addFrame(tFrame5);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame6;

  tFrame6.d->setConfig(d6);
  tQ6 = tFrame6.theta;
  // Frame 6 ( Is frame 3 for this test)
  myTable.addFrame(tFrame6);

  double theta4 = M_PI / 2;
  double theta5 = M_PI / 2;
  double theta6 = -M_PI / 2;

  tQ4->setConfig(theta4);
  tQ5->setConfig(theta5);
  tQ6->setConfig(theta6);


  DHTable::Frame tFrameTest = myTable.getFrame(1);

  ASSERT_EQ(tFrameTest.alpha->getConfig(), alpha4);
  ASSERT_EQ(tFrameTest.theta->getConfig(), tQ4->getConfig());

  tFrameTest = myTable.getFrame(2);
  ASSERT_EQ(tFrameTest.alpha->getConfig(), alpha5);
  ASSERT_EQ(tFrameTest.theta->getConfig(), tQ5->getConfig());

  tFrameTest = myTable.getFrame(3);
  ASSERT_EQ(tFrameTest.d->getConfig(), d6);
  ASSERT_EQ(tFrameTest.theta->getConfig(), tQ6->getConfig());

}

TEST(DHTable, FrameOutofBounds) {
  DHTable myTable;

  // This should be and error. Do we want to throw and catch exceptions?
  // FIXME: (Yhap) This would be a separate test if so
  // DHTable::Frame aFrame = myTable.modifyFrame(0);
}

TEST(DHTable, BuildFK_PositionTransform) {
  DHTable myTable;

  // Create Ptr to Configurable Params
  JointPtr tQ1;
  JointPtr tQ2;
  JointPtr tQ3;

  // Create Constants used in DH Parameters
  double d1 = 2;
  double a2 = 1;
  double a3 = 1;
  double alpha1 = M_PI / 2;


  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame1;

  tFrame1.d->setConfig(d1);
  tFrame1.alpha->setConfig(alpha1);
  tQ1 = tFrame1.theta;

  // Frame 1
  myTable.addFrame(tFrame1);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame2;

  tFrame2.a->setConfig(a2);
  tQ2 = tFrame2.theta;

  // Frame 2
  myTable.addFrame(tFrame2);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame3;

  tFrame3.a->setConfig(a3);
  tQ3 = tFrame3.theta;

  // Frame 3
  myTable.addFrame(tFrame3);

  // Set configuration values for expected output.
  tQ1->setConfig(0);
  tQ2->setConfig(0);
  tQ3->setConfig(0);

  Eigen::Matrix4d result = myTable.getTransform(0, 3);

  // Expected Transform is at position 2 0 2.5 with No rotation about the base
  // frame.
  Eigen::Matrix4d expected;
  // Organized in the matrix form instead of in one line for clarity.
  expected << 1, 0, 0, 2,
  0, 0, -1, 0,
  0, 1, 0, 2,
  0, 0, 0, 1;

  std::cout << "Result: \n" << result << std::endl;
  std::cout << "Expected: \n" << expected << std::endl;
  bool closeEnough = result.isApprox(expected);
  ASSERT_TRUE(closeEnough);
}

TEST(DHTable, BuildFK_OrientationTransform) {
  DHTable myTable;
  // Create Ptr to Configurable Params
  JointPtr tQ4;
  JointPtr tQ5;
  JointPtr tQ6;

  double d6 = 0.5;
  double alpha4 = -M_PI / 2;
  double alpha5 = M_PI / 2;


  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame4;

  tFrame4.alpha->setConfig(alpha4);
  tQ4 = tFrame4.theta;

  // Frame 4 ( Is frame 1 for this test)
  myTable.addFrame(tFrame4);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame5;

  tFrame5.alpha->setConfig(alpha5);
  tQ5 = tFrame5.theta;

  // Frame 5 ( Is frame 2 for this test)
  myTable.addFrame(tFrame5);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame6;

  tFrame6.d->setConfig(d6);
  tQ6 = tFrame6.theta;
  // Frame 6 ( Is frame 3 for this test)
  myTable.addFrame(tFrame6);

  double theta4 = M_PI / 2;
  double theta5 = M_PI / 2;
  double theta6 = -M_PI / 2;

  tQ4->setConfig(theta4);
  tQ5->setConfig(theta5);
  tQ6->setConfig(theta6);

  Eigen::Matrix4d result = myTable.getTransform(0, 3);

  // Expected Transform via Matlab Calculations
  Eigen::Matrix4d expected;

  expected << 1, 0, 0, 0,
  0, 0, 1, 0.5,
  0, -1, 0, 0,
  0, 0, 0, 1;

  std::cout << "Result: \n" << result << std::endl;
  std::cout << "Expected: \n" << expected << std::endl;
  bool closeEnough = result.isApprox(expected);
  ASSERT_TRUE(closeEnough);
}

TEST(DHTable, Base2EndEffector) {
  DHTable myTable;

  // Create Ptr to Configurable Params
  JointPtr tQ1;
  JointPtr tQ2;
  JointPtr tQ3;
  JointPtr tQ4;
  JointPtr tQ5;
  JointPtr tQ6;

  // Create Constants used in DH Parameters
  double d1 = 2;
  double a2 = 1;
  double a3 = 1;
  double alpha1 = M_PI / 2;
  double d6 = 0.5;
  double alpha4 = -M_PI / 2;
  double alpha5 = M_PI / 2;


  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame1;

  tFrame1.d->setConfig(d1);
  tFrame1.alpha->setConfig(alpha1);
  tQ1 = tFrame1.theta;

  // Frame 1
  myTable.addFrame(tFrame1);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame2;

  tFrame2.a->setConfig(a2);
  tQ2 = tFrame2.theta;

  // Frame 2
  myTable.addFrame(tFrame2);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame3;

  tFrame3.a->setConfig(a3);
  tQ3 = tFrame3.theta;

  // Frame 3
  myTable.addFrame(tFrame3);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame4;

  tFrame4.alpha->setConfig(alpha4);
  tQ4 = tFrame4.theta;

  // Frame 4 ( Is frame 1 for this test)
  myTable.addFrame(tFrame4);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame5;

  tFrame5.alpha->setConfig(alpha5);
  tQ5 = tFrame5.theta;

  // Frame 5 ( Is frame 2 for this test)
  myTable.addFrame(tFrame5);

  // Note all params are set to 0 unless otherwise specified.
  DHTable::Frame tFrame6;

  tFrame6.d->setConfig(d6);
  tQ6 = tFrame6.theta;
  // Frame 6 ( Is frame 3 for this test)
  myTable.addFrame(tFrame6);

  double theta1 = 0;
  double theta2 = 0;
  double theta3 = 0;
  double theta4 = M_PI / 2;
  double theta5 = M_PI / 2;
  double theta6 = -M_PI / 2;

  // Set configuration values for expected output.
  tQ1->setConfig(theta1);
  tQ2->setConfig(theta2);
  tQ3->setConfig(theta3);
  tQ4->setConfig(theta4);
  tQ5->setConfig(theta5);
  tQ6->setConfig(theta6);

  Eigen::Matrix4d result = myTable.getTransform(0, 6);

  // Expected Transform via Matlab Calculations
  Eigen::Matrix4d expected;

  expected << 1, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, 2.5, 0, 0, 0, 1;

  std::cout << "Result: \n" << result << std::endl;
  std::cout << "Expected: \n" << expected << std::endl;

  bool closeEnough = result.isApprox(expected);
  ASSERT_TRUE(closeEnough);
}
