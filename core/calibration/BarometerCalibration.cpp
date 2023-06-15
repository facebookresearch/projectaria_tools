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

#include <calibration/BarometerCalibration.h>

namespace projectaria::tools::calibration {

BarometerCalibration::BarometerCalibration(const std::string& label, double slope, double offsetPa)
    : label_(label), slope_(slope), offsetPa_(offsetPa) {}

std::string BarometerCalibration::getLabel() const {
  return label_;
}

double BarometerCalibration::rawToRectified(const double raw) const {
  return slope_ * raw + offsetPa_;
}

double BarometerCalibration::rectifiedToRaw(const double rectified) const {
  return (rectified - offsetPa_) / slope_;
}

} // namespace projectaria::tools::calibration
