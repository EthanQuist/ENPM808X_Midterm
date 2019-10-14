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
 * @file Joints.hpp
*
 * @brief Interface/Abstract Joint Class (Defines required functions)
*
 * @author Corbyn Yhap (Driver)
*
 * @copyright Acme Robotics, Ethan Quist, Corbyn Yhap
*/
#pragma once

#include <memory>

class IJoint {
 public:
  /**

   * @brief Destructor for Joint Interface

   * @param None.

   * @return None

   */
  virtual ~IJoint();

  /**

   * @brief Method to retrieve current joint configuration

   * @param None.

   * @return double

   */
  virtual double getConfig() = 0;

  /**

   * @brief Method to set current joint configuration

   * @param double the joint configuration value

   * @return None.

   */
  virtual void setConfig(double) = 0;
};
// Typedef the pointer for easy external polymorphic use.
typedef std::shared_ptr<IJoint> JointPtr;

class PrismaticJoint : public IJoint {
 public:
  /**

   * @brief Prismatic Joint Constructor (Length Initialized to 0)

   * @param None

   * @return None.

   */
  PrismaticJoint();

  /**

   * @brief Alternate Prismatic Joint Constructor

   * @param double Sets the length of the Prismatic joint

   * @return None.

   */
  explicit PrismaticJoint(double);

  /**

   * @brief Prismatic Joint Destructor

   * @param None

   * @return None.

   */
  virtual ~PrismaticJoint();

  /**

   * @brief Method to retrieve Prismatic Joint Length (Meters)

   * @param None.

   * @return double Current Prismatic Joint length.

   */
  double getConfig();

  /**

   * @brief Method to set current joint configuration

   * @param double The current length of the Prismatic joint. (Meters)

   * @return None.

   */
  void setConfig(double);

 private:
  double length;
};

class RevoluteJoint : public IJoint {
 public:
  /**

   * @brief Revolute Joint Constructor (Angle Initialized to 0)

   * @param None

   * @return None.

   */
  RevoluteJoint();

  /**

   * @brief Alternate Prismatic Joint Constructor

   * @param double Sets the angle of the Revolute joint

   * @return None.

   */
  explicit RevoluteJoint(double);

  /**

   * @brief Revolute Joint Destructor

   * @param None

   * @return None.

   */
  virtual ~RevoluteJoint();

  /**

   * @brief Method to retrieve Revolute Joint Angle (Degrees)

   * @param None.

   * @return double Current Revolute Joint angle.

   */
  double getConfig();

  /**

   * @brief Method to set current joint configuration

   * @param double The current angle of the Revolute joint. (Degrees)

   * @return None.

   */
  void setConfig(double);

 private:
  double angle;
};
