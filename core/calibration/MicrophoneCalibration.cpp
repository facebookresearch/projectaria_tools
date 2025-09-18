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

#include <calibration/MicrophoneCalibration.h>

namespace projectaria::tools::calibration {

MicrophoneCalibration::MicrophoneCalibration(const std::string& label, double dSensitivity1KDbv)
    : label_(label), dSensitivity1KDbv_(dSensitivity1KDbv) {}

std::string MicrophoneCalibration::getLabel() const {
  return label_;
}

double MicrophoneCalibration::getDSensitivity1KDbv() const {
  return dSensitivity1KDbv_;
}

double MicrophoneCalibration::rawToRectified(const double raw) const {
  return raw - dSensitivity1KDbv_;
}

double MicrophoneCalibration::rectifiedToRaw(const double rectified) const {
  return rectified + dSensitivity1KDbv_;
}

} // namespace projectaria::tools::calibration
