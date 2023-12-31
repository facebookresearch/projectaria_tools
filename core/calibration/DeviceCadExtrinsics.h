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

#include <optional>
#include <unordered_map>

#include <sophus/se3.hpp>

namespace projectaria::tools::calibration {

/**
 * @brief This class retrieves fixed CAD extrinsics values for Aria Device
 */
class DeviceCadExtrinsics {
 public:
  DeviceCadExtrinsics() {}
  /**
   * @brief Construct for Cad extrinsics based on device sub type and origin label.
   * @param[in] deviceSubType the device sub type, for Aria it can be "DVT-S" or "DVT-L" based on
   * glass size.
   * @param[in] originSensorLabel the label of the origin (`Device` coordinate frame) sensor,
   * e.g. "camera-slam-left".
   */
  DeviceCadExtrinsics(const std::string& deviceSubType, const std::string& originSensorLabel);

  /**
   * @brief Get the CAD extrinsics value of `T_Device_Sensor`, from a given sensor label. The
   * `Device` is defined by `originSensorLabel` in class constructor.
   */
  std::optional<Sophus::SE3d> getT_Device_Sensor(const std::string& label) const;
  /**
   * @brief returns relative pose between the device frame (defined by
   * `originSensorLabel` in class constructor) and CPF (central pupil frame), where CPF is a virtual
   * coordinate frame defined in CAD model
   */
  Sophus::SE3d getT_Device_Cpf() const;

 private:
  std::unordered_map<std::string, Sophus::SE3d> labelToT_Cpf_Sensor_;
  Sophus::SE3d T_Device_Cpf_;
};

} // namespace projectaria::tools::calibration
