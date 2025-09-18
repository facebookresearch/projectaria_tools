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

#include <calibration/BarometerCalibration.h>
#include <calibration/CameraCalibration.h>
#include <calibration/ImuMagnetometerCalibration.h>
#include <calibration/MicrophoneCalibration.h>

namespace projectaria::tools::calibration {
/**
 * @brief In AriaGen1, ET images are stitched horizontally, thus two calibrations are attached to
 * the same VRS stream.
 */
using AriaEtCalibration = std::array<CameraCalibration, 2>;

/**
 * @brief In Aria, Mic calibration is 7-channel
 */
using AriaMicCalibration = std::array<MicrophoneCalibration, 7>;

/**
 * @brief An enum to represent the sensor type of a calibration instance
 */
enum class SensorCalibrationType {
  NotValid,
  CameraCalibration,
  ImuCalibration,
  MagnetometerCalibration,
  BarometerCalibration,
  MicrophoneCalibration,
  AriaEtCalibration,
  AriaMicCalibration
};

/**
 * @brief An adaptor class to access an arbitrary sensor's calibration, which is a `std::variant` of
 * {CameraCalibration, ImuCalibration, MagnetometerCalibration, BarometerCalibration,
 * MicrophoneCalibration, AriaEtCalibration, AriaMicCalibration}.
 */
class SensorCalibration {
 public:
  using SensorCalibrationVariant = std::variant<
      std::monostate,
      CameraCalibration,
      ImuCalibration,
      MagnetometerCalibration,
      BarometerCalibration,
      MicrophoneCalibration,
      AriaEtCalibration,
      AriaMicCalibration>;

 public:
  SensorCalibration() = default;
  explicit SensorCalibration(const SensorCalibrationVariant& calibVariant);

  /**
   * @brief Try to get the SensorCalibration as a CameraCalibration. Will throw if sensor type does
   * not match.
   */
  CameraCalibration cameraCalibration() const;
  /**
   * @brief Try to get the SensorCalibration as a ImuCalibration. Will throw if sensor type does not
   * match.
   * .
   */
  ImuCalibration imuCalibration() const;
  /**
   * @brief Try to get the SensorCalibration as a MagnetometerCalibration. Will throw if sensor type
   * does not match.
   */
  MagnetometerCalibration magnetometerCalibration() const;
  /**
   * @brief Try to get the SensorCalibration as a BarometerCalibration. Will throw if sensor type
   * does not match.
   */
  BarometerCalibration barometerCalibration() const;
  /**
   * @brief Try to get the SensorCalibration as a MicrophoneCalibration. Will throw if sensor type
   * does not match.
   */
  MicrophoneCalibration microphoneCalibration() const;
  /**
   * @brief Try to get the SensorCalibration as an AriaEtCalibration.  Will throw if sensor type
   * does not match.
   */
  AriaEtCalibration ariaEtCalibration() const;
  /**
   * @brief Try to get the SensorCalibration as an AriaMicCalibration.  Will throw if sensor type
   * does not match.
   */
  AriaMicCalibration ariaMicCalibration() const;

  /**
   * @brief get the type of this sensor calibration as an enum.
   */
  SensorCalibrationType sensorCalibrationType() const;

 private:
  SensorCalibrationVariant calibVariant_;
  SensorCalibrationType sensorCalibrationType_;
};

} // namespace projectaria::tools::calibration
