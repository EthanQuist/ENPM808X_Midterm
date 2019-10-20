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

#include <cmath>

#include "InverseKinematics.hpp"
#include "RevoluteJoint.hpp"


InverseKinematicsBase::~InverseKinematicsBase() {
}

InverseKinematicAcmeArm::InverseKinematicAcmeArm() {
}

InverseKinematicAcmeArm::~InverseKinematicAcmeArm() {
}

std::vector<JointPtr> InverseKinematicAcmeArm::computeIK(
    Eigen::Matrix4d Transform) {

  double r11 = Transform(0, 0);
  double r12 = Transform(0, 1);
  double r13 = Transform(0, 2);
  double r21 = Transform(1, 0);
  double r22 = Transform(1, 1);
  double r23 = Transform(1, 2);
  double r33 = Transform(2, 2);

  double xo = Transform(0, 3);
  double yo = Transform(1, 3);
  double zo = Transform(2, 3);

  double xc, yc, zc;
  double d1, d6, a2, a3;
  double q1, q2, q3, q4, q5, q6;
  double of;
  double G;

  // Robot parameters
  d1 = 2;
  d6 = 0.5;
  a2 = 1;
  a3 = 1;

  xc = xo - d6 * r13;
  yc = yo - d6 * r23;
  zc = zo - d6 * r33;

  q1 = atan2(yc, xc);

  of = 0;
  G =
      (xc * xc + yc * yc - of * of + (zc - d1) * (zc - d1) - a2 * a2 - a3 * a3)
          / (2 * a2 * a3);

  q3 = atan2(sqrt(1 - G * G), G);

  q2 = atan2(zc - d1, sqrt(xc * xc + yc * yc - of * of))
      - atan2(a3 * sin(q3), a2 + a3 * cos(q3));

  q4 = atan2(
      -cos(q1) * sin(q2 + q3) * r13 - sin(q1) * sin(q2 + q3) * r23
          + cos(q2 + q3) * r33,
      cos(q1) * cos(q2 + q3) * r13 + sin(q1) * cos(q2 + q3) * r23
          + sin(q2 + q3) * r33);

  q5 = atan2(
      sqrt(
          1
              - (sin(q1) * r13 - cos(q1) * r23)
                  * (sin(q1) * r13 - cos(q1) * r23)),
      sin(q1) * r13);

  q6 = atan2(sin(q1) * r12 - cos(q1) * r22, -sin(q1) * r11 + cos(q1) * r21);


  JointPtr joint_q1(new RevoluteJoint(q1));
  JointPtr joint_q2(new RevoluteJoint(q2));
  JointPtr joint_q3(new RevoluteJoint(q3));
  JointPtr joint_q4(new RevoluteJoint(q4));
  JointPtr joint_q5(new RevoluteJoint(q5));
  JointPtr joint_q6(new RevoluteJoint(q6));

  std::vector<JointPtr> jointAngles;
  jointAngles.push_back(joint_q1);
  jointAngles.push_back(joint_q2);
  jointAngles.push_back(joint_q3);
  jointAngles.push_back(joint_q4);
  jointAngles.push_back(joint_q5);
  jointAngles.push_back(joint_q6);


  return jointAngles;
}
