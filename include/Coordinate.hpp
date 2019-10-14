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
 * @file Coordinate.hpp
*
 * @brief Class Definition of Coordinate (Units in Meters)
*
 * @author Corbyn Yhap (Driver)
*
 * @copyright Acme Robotics, Ethan Quist, Corbyn Yhap
*/
#pragma once

class Coordinate {
 public:
  /**

   * @brief Constructor for the Coordinate Class

   * @param aX This is the X Coordinate

   * @param aY This is the Y Coordinate

   * @param aZ This is the Z Coordinate

   * @return None.

   */
  Coordinate(double aX, double aY, double aZ);

  /**

   * @brief Destructor for the Coordinate Class

   * @param None.

   * @return None.

   */
  ~Coordinate();

  /**

   * @brief Retrieves the X Coordinate

   * @param None.

   * @return The X Coordinate.

   */
  double getX();

  /**

   * @brief Retrieves the Y Coordinate

   * @param None.

   * @return The Y Coordinate.

   */
  double getY();

  /**

   * @brief Retrieves the Z Coordinate

   * @param None.

   * @return The Z Coordinate.

   */
  double getZ();

  /**

   * @brief Sets the X Coordinate

   * @param aX The new value of the X Coordinate.

   * @return None.

   */
  void setX(const double &aX);

  /**

   * @brief Sets the Y Coordinate

   * @param aY The new value of the Y Coordinate.

   * @return None.

   */
  void setY(const double &aY);

  /**

   * @brief Sets the Z Coordinate

   * @param aZ The new value of the Z Coordinate.

   * @return None.

   */
  void setZ(const double &aZ);

  /**

   * @brief Sets the X, Y and Z Coordinates

   * @param aX The new value of the X Coordinate.

   * @param aY The new value of the Y Coordinate.

   * @param aZ The new value of the Z Coordinate.

   * @return None.

   */
  void setXYZ(const double &aX, const double &aY, const double &aZ);

  bool operator==(const Coordinate &rhs) const;

 private:
  /**

   * @brief Convert Meters to Feet

   * @param double The value in meters

   * @return The converted value in feet.

   */
  double convertM2F(double);
  /**

   * @brief Convert Feet to Meters

   * @param double The value in Feet

   * @return double The converted value in Meters.

   */
  double convertF2M(double);
  /**

   * @brief Constructor for the Coordinate Class

   * @param None.

   * @return None.

   */
  Coordinate();

  double x;
  double y;
  double z;
};
