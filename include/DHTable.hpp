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
 * @file DHTable.hpp
 *
 * @brief The class header for our DH Table Class
 *
 * @author Corbyn Yhap (driver) and Ethan Quist (Navigator)
 *
 * @copyright Acme Robotics, Ethan Quist, Corbyn Yhap
 */
#pragma once

#include <memory>
#include <vector>
#include "Eigen/Dense"

#include "Joints.hpp"

class DHTable {

 public:
  // Default values prevent these pointers from being null.
  struct Frame {
    std::shared_ptr<PrismaticJoint> d = std::make_shared<PrismaticJoint>(
        PrismaticJoint());
    std::shared_ptr<RevoluteJoint> theta = std::make_shared<RevoluteJoint>(
        RevoluteJoint());
    std::shared_ptr<PrismaticJoint> a = std::make_shared<PrismaticJoint>(
        PrismaticJoint());
    std::shared_ptr<RevoluteJoint> alpha = std::make_shared<RevoluteJoint>(
        RevoluteJoint());
  };

  /**

   * @brief DHTable Constructor with no Parameters.

   * @param None.

   * @return None.

   */
  DHTable();

  /**

   * @brief Destructor for the DHTable Class

   * @param None.

   * @return None.

   */
  ~DHTable();

  /**

   * @brief addFrame Method for the user to describe the Robot
   * The Frame access is 1 indexed. (Preserving 0 for the
   *  base frame) Frames must be added in the order of relation. i.e. frame 1
   *  describes the relationship from frame 0 to frame 1 and frame 2 the
   *  relationship from frame 1 to frame 2.

   * @param const Frame &, The Frame that will be added to the DH Table

   * @return None.

   */
  void addFrame(const Frame&);

  /**

   * @brief getFrame returns a copy of the frame that was stored

   * @param std::vector<Frame>::size_type 1 Indexed index of desired frame

   * @return A copy of the frame.

   */
  Frame getFrame(std::vector<Frame>::size_type);

  /**

   * @brief getTransform This function returns the Transformation Matrix from
   *  one frame to the another. Currently it is required that the first frame
   *  must precede the second frame.

   * @param size_type,  The index of the frame being transformed from.

   * @param size_type,  The index of the frame being transformed to.

   * @return Eigen::Matrix4d The Transformation Matrix to go from frame one to
   * frame two.

   */
  Eigen::Matrix4d getTransform(std::vector<Frame>::size_type,
                               std::vector<Frame>::size_type);
 private:
  std::vector<Frame> frames;

  /**

   * @brief getTransform This function acts as a helper by taking the index of
   * a frame, retrieving that frame and then using those parameters to build a
   * tranformation matrix.

   * @param size_type,  The index of the frame being transformed to. This
   * function assumed that the tranform from is from the frame that preceded it.


   * @return Eigen::Matrix4d The Transformation Matrix to go to frame index from
   * the previous frame.

   */
  Eigen::Matrix4d getTransform(std::vector<Frame>::size_type);

};
