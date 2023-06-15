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

#include <calibration/SensorCalibration.h>
#include <data_provider/ErrorHandler.h>

namespace projectaria::tools::calibration {
SensorCalibration::SensorCalibration(const SensorCalibrationVariant& calibVariant)
    : calibVariant_(calibVariant),
      sensorCalibrationType_(static_cast<SensorCalibrationType>(calibVariant.index())) {}

CameraCalibration SensorCalibration::cameraCalibration() const {
  data_provider::checkAndThrow(sensorCalibrationType_ == SensorCalibrationType::CameraCalibration);
  return std::get<CameraCalibration>(calibVariant_);
}
ImuCalibration SensorCalibration::imuCalibration() const {
  data_provider::checkAndThrow(sensorCalibrationType_ == SensorCalibrationType::ImuCalibration);
  return std::get<ImuCalibration>(calibVariant_);
}
MagnetometerCalibration SensorCalibration::magnetometerCalibration() const {
  data_provider::checkAndThrow(
      sensorCalibrationType_ == SensorCalibrationType::MagnetometerCalibration);
  return std::get<MagnetometerCalibration>(calibVariant_);
}

BarometerCalibration SensorCalibration::barometerCalibration() const {
  data_provider::checkAndThrow(
      sensorCalibrationType_ == SensorCalibrationType::BarometerCalibration);
  return std::get<BarometerCalibration>(calibVariant_);
}
MicrophoneCalibration SensorCalibration::microphoneCalibration() const {
  data_provider::checkAndThrow(
      sensorCalibrationType_ == SensorCalibrationType::MicrophoneCalibration);
  return std::get<MicrophoneCalibration>(calibVariant_);
}
AriaEtCalibration SensorCalibration::ariaEtCalibration() const {
  data_provider::checkAndThrow(sensorCalibrationType_ == SensorCalibrationType::AriaEtCalibration);
  return std::get<AriaEtCalibration>(calibVariant_);
}
AriaMicCalibration SensorCalibration::ariaMicCalibration() const {
  data_provider::checkAndThrow(sensorCalibrationType_ == SensorCalibrationType::AriaMicCalibration);
  return std::get<AriaMicCalibration>(calibVariant_);
}

SensorCalibrationType SensorCalibration::sensorCalibrationType() const {
  return sensorCalibrationType_;
}

} // namespace projectaria::tools::calibration
