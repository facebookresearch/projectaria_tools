/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <string>

namespace projectaria::tools::calibration {

struct MicrophoneCalibration {
  MicrophoneCalibration() = default;
  MicrophoneCalibration(const std::string& label, double dSensitivity1KDbv);

  std::string getLabel() const;
  double getDSensitivity1KDbv() const;

  double rawToRectified(const double raw) const;
  double rectifiedToRaw(const double rectified) const;

 private:
  std::string label_;
  // Sensitivity difference between this instance and the reference mic at 1kHz,
  // in unit of dBV.
  // rectified = raw - dSensitivity1KDbv
  double dSensitivity1KDbv_;
};

} // namespace projectaria::tools::calibration
