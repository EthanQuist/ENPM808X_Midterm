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

DHTable::DHTable(unsigned int aNumFrames) {
  frames = std::vector<Frame>(aNumFrames);
}

DHTable::~DHTable() {
}

DHTable::Frame& DHTable::modifyFrame(unsigned int aFrameIdx) {
  DHTable::Frame &tReturn;
  // The Frame indices are 1 indexed to the user even though they are 0 indexed
  // when stored.
  tReturn = frames(aFrameIdx - 1);

  return tReturn;
}

Eigen::Matrix4d DHTable::getTransform(unsigned int aStartFrame,
                                      unsigned int aEndFrame) {
  (void) aStartFrame;
  (void) aEndFrame;

  Eigen::Matrix4d tReturn;

  return tReturn;
}

DHTable::DHTable() {
}

Eigen::Matrix4d DHTable::getTransform(unsigned int) {
  Eigen::Matrix4d tReturn;

  return tReturn;
}
