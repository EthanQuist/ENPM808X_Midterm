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
 * @file InverseKinematics.hpp
 *
 * @brief The class header for our Inverse Kinematics Class
 *
 * @author Ethan Quist (driver) and Corbyn Yhap (Navigator)
 *
 * @copyright Acme Robotics, Ethan Quist, Corbyn Yhap
 */
#pragma once

#include <memory>
#include <vector>
#include "Coordinate.hpp"
#include "Joints.hpp"

class InverseKinematicsBase {
 public:
  /**

   * @brief Virtual Destructor for the IK Base Class

   * @param None.

   * @return None.

   */

  virtual ~InverseKinematicsBase();

  /**

   * @brief Method to compute Inverse Kinematics. Required of all
   *  Derived Classes

   * @param Coordinate. A XYZ Coordinate (Meters)

   * @return std::vector<JointPtr> A vector of joints (From which configurations
   * can be retrieved)

   */
  virtual std::vector<JointPtr> computeIK(Coordinate) = 0;
};

class InverseKinematicAcmeArm : InverseKinematicsBase {
 public:
  /**

   * @brief Constructor for Inverse Kinematics Acme Arm

   * @param None.

   * @return None.

   */
  InverseKinematicAcmeArm();

  /**

   * @brief Destructor for Inverse Kinematics Acme Arm

   * @param None.

   * @return None.

   */
  virtual ~InverseKinematicAcmeArm();

  /**

   * @brief Method to compute Inverse Kinematics For the Acme Arm

   * @param Coordinate. A XYZ Coordinate (Meters)

   * @return std::vector<JointPtr> The vector of corresponding joints.

   */
  std::vector<JointPtr> computeIK(Coordinate);
};
