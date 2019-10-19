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

#include "DHTable.hpp"

DHTable::~DHTable() {
}

void DHTable::addFrame(const Frame &aFrame) {
  // The Frame indices are 1 indexed to the user even though they are 0 indexed
  // when stored.
  // Need to push back a copy
  frames.push_back(aFrame);
}

DHTable::Frame DHTable::getFrame(std::vector<Frame>::size_type aFrameIdx) {
  return frames.at(aFrameIdx - 1);
}

Eigen::Matrix4d DHTable::getTransform(std::vector<Frame>::size_type aStartFrame,
                                      std::vector<Frame>::size_type aEndFrame) {
  Eigen::Matrix4d tReturn;
  tReturn.setIdentity();

  if (aStartFrame > aEndFrame) {
    // This should be flagged as an error.
  }
  if (aEndFrame > frames.size()) {
    // This should be flagged as an error.
  }

  // Added 1 to because frames go from A to B
  aStartFrame += 1;
  aEndFrame += 1;
  for (auto idx = aStartFrame; idx < aEndFrame; idx++) {
    tReturn *= getTransform(idx);
  }

  return tReturn;
}

DHTable::DHTable() {
}

Eigen::Matrix4d DHTable::getTransform(std::vector<Frame>::size_type aFrameIdx) {

  // Calls are 1 indexed though storage is 0 indexed.
  Frame frame = frames.at(aFrameIdx - 1);
  Eigen::Affine3d tTD(
      Eigen::Translation3d(Eigen::Vector3d(0, 0, frame.d->getConfig())));
  Eigen::Affine3d tTTheta(
      Eigen::AngleAxisd(frame.theta->getConfig(), Eigen::Vector3d::UnitZ()));
  Eigen::Affine3d tTA(
      Eigen::Translation3d(Eigen::Vector3d(frame.a->getConfig(), 0, 0)));
  Eigen::Affine3d tTAlpha(
      Eigen::AngleAxisd(frame.alpha->getConfig(), Eigen::Vector3d::UnitX()));

  Eigen::Matrix4d tReturn = (tTTheta * tTD * tTA * tTAlpha).matrix();

  return tReturn;
}
