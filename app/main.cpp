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
#include <iostream>
#include <lib.hpp>


int main() {



  /*
  //build the IK Solver Class and result
  InverseKinematicAcmeArm IKsolver;
  std::vector < JointPtr > result;

  //Set the inputs to the IK Solver
  Eigen::Matrix4d T(4, 4);

  std::cout << T << std::endl;

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

  std::cout << T << std::endl;

  //Run the IK Solver
  result = IKsolver.computeIK(T);

  //Organize the results
  JointPtr result1 = result[0];
  JointPtr result2 = result[1];
  JointPtr result3 = result[2];
  JointPtr result4 = result[3];
  JointPtr result5 = result[4];
  JointPtr result6 = result[5];
  double angle1 = result1->getConfig();
  double angle2 = result2->getConfig();
  double angle3 = result3->getConfig();
  double angle4 = result4->getConfig();
  double angle5 = result5->getConfig();
  double angle6 = result6->getConfig();

  //Print the results
  std::cout << angle1 << std::endl;
  std::cout << angle2 << std::endl;
  std::cout << angle3 << std::endl;
  std::cout << angle4 << std::endl;
  std::cout << angle5 << std::endl;
  std::cout << angle6 << std::endl;
   *
   */
  Demo demonstration;
  demonstration.runDemo();
  dummy();
  return 0;
}
