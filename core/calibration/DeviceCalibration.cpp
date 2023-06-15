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

#include <calibration/DeviceCalibration.h>

#include <logging/Checks.h>
#define DEFAULT_LOG_CHANNEL "DeviceCalibration"
#include <logging/Log.h>

namespace projectaria::tools::calibration {
DeviceCalibration::DeviceCalibration(
    const std::map<std::string, CameraCalibration>& cameraCalibs,
    const std::map<std::string, ImuCalibration>& imuCalibs,
    const std::map<std::string, MagnetometerCalibration>& magnetometerCalibs,
    const std::map<std::string, BarometerCalibration>& barometerCalibs,
    const std::map<std::string, MicrophoneCalibration>& microphoneCalibs,
    const DeviceCadExtrinsics& deviceCadExtrinsics,
    const std::string& deviceSubtype,
    const std::string& originLabel)
    : cameraCalibs_(cameraCalibs),
      imuCalibs_(imuCalibs),
      magnetometerCalibs_(magnetometerCalibs),
      barometerCalibs_(barometerCalibs),
      microphoneCalibs_(microphoneCalibs),
      deviceCadExtrinsics_(deviceCadExtrinsics),
      deviceSubtype_(deviceSubtype),
      originLabel_(originLabel) {
  for (const auto& [label, calib] : cameraCalibs) {
    allCalibs_.emplace(label, calib);
  }
  for (const auto& [label, calib] : imuCalibs) {
    allCalibs_.emplace(label, calib);
  }
  for (const auto& [label, calib] : magnetometerCalibs) {
    allCalibs_.emplace(label, calib);
  }
  for (const auto& [label, calib] : barometerCalibs) {
    allCalibs_.emplace(label, calib);
  }
  for (const auto& [label, calib] : microphoneCalibs) {
    allCalibs_.emplace(label, calib);
  }
}

std::vector<std::string> DeviceCalibration::getAllLabels() const {
  std::vector<std::string> labels;
  for (const auto& [key, _] : allCalibs_) {
    labels.push_back(key);
  }
  return labels;
}

std::vector<std::string> DeviceCalibration::getCameraLabels() const {
  std::vector<std::string> cameraLabels;
  for (const auto& [key, _] : cameraCalibs_) {
    cameraLabels.push_back(key);
  }
  return cameraLabels;
}

std::vector<std::string> DeviceCalibration::getImuLabels() const {
  std::vector<std::string> imuLabels;
  for (const auto& [key, _] : imuCalibs_) {
    imuLabels.push_back(key);
  }
  return imuLabels;
}

std::vector<std::string> DeviceCalibration::getMagnetometerLabels() const {
  std::vector<std::string> magnetometerLabels;
  for (const auto& [key, _] : magnetometerCalibs_) {
    magnetometerLabels.push_back(key);
  }
  return magnetometerLabels;
}

std::vector<std::string> DeviceCalibration::getBarometerLabels() const {
  std::vector<std::string> barometerLabels;
  for (const auto& [key, _] : barometerCalibs_) {
    barometerLabels.push_back(key);
  }
  return barometerLabels;
}

std::vector<std::string> DeviceCalibration::getMicrophoneLabels() const {
  std::vector<std::string> microphoneLabels;
  for (const auto& [key, _] : microphoneCalibs_) {
    microphoneLabels.push_back(key);
  }
  return microphoneLabels;
}

template <typename T, typename T2>
std::optional<T> returnIfFound(
    const std::map<std::string, T2>& calibrationData,
    const std::string& label) {
  auto itCalib = calibrationData.find(label);
  if (itCalib != calibrationData.end()) {
    return std::optional<T>(itCalib->second);
  }
  return {};
}

std::optional<SensorCalibration> DeviceCalibration::getSensorCalib(const std::string& label) const {
  if (label == "camera-et") {
    auto maybeEtCalib = getAriaEtCameraCalib();
    if (maybeEtCalib) {
      return std::optional<SensorCalibration>(maybeEtCalib.value());
    }
  }
  if (label == "mic") {
    auto maybeMicCalib = getAriaMicrophoneCalib();
    if (maybeMicCalib) {
      return std::optional<SensorCalibration>(maybeMicCalib.value());
    }
  }
  return returnIfFound<SensorCalibration>(allCalibs_, label);
}

std::optional<CameraCalibration> DeviceCalibration::getCameraCalib(const std::string& label) const {
  if (label == "camera-et") {
    XR_LOGW(
        "camera-et contains calibrations for both left and right cameras. Please use getSensorCalib(camera-et) or getAriaEtCameraCalib() instead");
    return {};
  }
  return returnIfFound<CameraCalibration>(cameraCalibs_, label);
}

std::optional<ImuCalibration> DeviceCalibration::getImuCalib(const std::string& label) const {
  return returnIfFound<ImuCalibration>(imuCalibs_, label);
}

std::optional<MagnetometerCalibration> DeviceCalibration::getMagnetometerCalib(
    const std::string& label) const {
  return returnIfFound<MagnetometerCalibration>(magnetometerCalibs_, label);
}

std::optional<BarometerCalibration> DeviceCalibration::getBarometerCalib(
    const std::string& label) const {
  return returnIfFound<BarometerCalibration>(barometerCalibs_, label);
}

std::optional<MicrophoneCalibration> DeviceCalibration::getMicrophoneCalib(
    const std::string& label) const {
  if (label == "mic") {
    XR_LOGW(
        "Aria mic contains 7 microphones. Please use getSensorCalib(mic) or getAriaMicrophoneCalib() instead");
    return {};
  }
  return returnIfFound<MicrophoneCalibration>(microphoneCalibs_, label);
}

std::optional<AriaEtCalibration> DeviceCalibration::getAriaEtCameraCalib() const {
  auto maybeCamCalibLeft = getCameraCalib("camera-et-left");
  auto maybeCamCalibRight = getCameraCalib("camera-et-right");
  if (maybeCamCalibLeft && maybeCamCalibRight) {
    AriaEtCalibration calib = {*maybeCamCalibLeft, *maybeCamCalibRight};
    return calib;
  } else {
    return {};
  }
}

std::optional<AriaMicCalibration> DeviceCalibration::getAriaMicrophoneCalib() const {
  AriaMicCalibration calib;
  for (int i = 0; i < 7; ++i) {
    auto maybeMicCalib = getMicrophoneCalib("mic" + std::to_string(i));
    if (maybeMicCalib) {
      calib.at(i) = *maybeMicCalib;
    } else {
      return {};
    }
  }
  return calib;
}

std::string DeviceCalibration::getDeviceSubtype() const {
  return deviceSubtype_;
}

Sophus::SE3d DeviceCalibration::getT_Device_Cpf() const {
  return deviceCadExtrinsics_.getT_Device_Cpf();
}

std::optional<Sophus::SE3d> DeviceCalibration::getT_Device_Sensor(
    const std::string& label,
    bool getCadValue) const {
  // If specified getting extrinsics from CAD
  if (getCadValue) {
    return deviceCadExtrinsics_.getT_Device_Sensor(label);
  } else {
    // Get calibrated extrinsics for camera
    if (label == "camera-slam-left" || label == "camera-slam-right" || label == "camera-rgb" ||
        label == "camera-et-left" || label == "camera-et-right") {
      const auto maybeCamCalib = getCameraCalib(label);
      if (maybeCamCalib.has_value()) {
        return maybeCamCalib.value().getT_Device_Camera();
      } else {
        XR_LOGE("Camera label {} not found in calibration. Please double check label.", label);
        return {};
      }
    }
    // Get calibrated extrinsics for imu
    else if (label == "imu-left" || label == "imu-right") {
      const auto maybeImuCalib = getImuCalib(label);
      if (maybeImuCalib.has_value()) {
        return maybeImuCalib.value().getT_Device_Imu();
      } else {
        XR_LOGE("Imu label {} not found in calibration. Please double check label.", label);
        return {};
      }
    }
    // For all other sensors, they are NOT calibrated
    else {
      XR_LOGE(
          "Sensor {} is not calibrated by default. Please use ::getT_Device_SensorByLabel(label, true) to use its CAD extrinsics value.",
          label);
      return {};
    } // end if label ==
  } // end if getCadValue
}

std::optional<Sophus::SE3d> DeviceCalibration::getT_Cpf_Sensor(
    const std::string& label,
    bool getCadValue) const {
  auto const maybeT_Device_Sensor = getT_Device_Sensor(label, getCadValue);
  if (maybeT_Device_Sensor) {
    return getT_Device_Cpf().inverse() * maybeT_Device_Sensor.value();
  } else {
    return {};
  }
}

const std::string& DeviceCalibration::getOriginLabel() const {
  return originLabel_;
}
} // namespace projectaria::tools::calibration
