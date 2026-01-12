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
class BarometerCalibration {
 public:
  BarometerCalibration(const std::string& label, double slope, double offsetPa);

  [[nodiscard]] std::string getLabel() const;
  [[nodiscard]] double getSlope() const;
  [[nodiscard]] double getOffsetPa() const;

  // convert from raw to rectified data to compensate system error
  [[nodiscard]] double rawToRectified(double raw) const;
  // inverse function of rawToRectified, for simulating raw sensor data from actual (rectified) data
  [[nodiscard]] double rectifiedToRaw(double rectified) const;

 private:
  std::string label_;
  double slope_; // unit less
  double offsetPa_; // unit is in Pascal
};
} // namespace projectaria::tools::calibration
