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

#include <calibration/DeviceCadExtrinsics.h>
#include <calibration/SensorCalibration.h>
#include <sophus/se3.hpp>
#include <map>
#include <optional>
#include <string>

namespace projectaria::tools::calibration {

/**
 * @brief A class to store and access calibration information of a device,
 * including: camera, imu, magnetometer, barometer, and microphones.
 */
class DeviceCalibration {
 public:
  /**
   * @brief Constructor that composes a collection of sensor calibrations into a DeviceCalibration
   * @param cameraCalibs map of <label, CameraCalibration>
   * @param imuCalibs map of <label, ImuCalibration>
   * @param magnetometerCalibs map of <label, MagnetometerCalibration>
   * @param barometerCalibs map of <label, BarometerCalibration>
   * @param microphoneCalibs map of <label, MicrophoneCalibration>
   * @param deviceCadExtrinsics a struct representing the CAD extrinsics info of the device sensors.
   * @param deviceSubtype the subtype of the device. For Aria, this would be "DVT-S' or "DVT-L".
   * @param originLabel the label identifying the origin of the calibration extrinsics, which needs
   to be a sensor within this device. This is basically the "Device" frame in `T_Device_Sensor`.
   */
  DeviceCalibration(
      const std::map<std::string, CameraCalibration>& cameraCalibs = {},
      const std::map<std::string, ImuCalibration>& imuCalibs = {},
      const std::map<std::string, MagnetometerCalibration>& magnetometerCalibs = {},
      const std::map<std::string, BarometerCalibration>& barometerCalibs = {},
      const std::map<std::string, MicrophoneCalibration>& microphoneCalibs = {},
      const DeviceCadExtrinsics& deviceCadExtrinsics = {},
      const std::string& deviceSubtype = {},
      const std::string& originLabel = {});

  /**
   * @brief returns all labels for all the sensors
   */
  std::vector<std::string> getAllLabels() const;
  /**
   * @brief returns all labels for cameras
   */
  std::vector<std::string> getCameraLabels() const;
  /**
   * @brief returns all labels for imus
   */
  std::vector<std::string> getImuLabels() const;
  /**
   * @brief returns all labels for magnetometers
   */
  std::vector<std::string> getMagnetometerLabels() const;
  /**
   * @brief returns all labels for barometers
   */
  std::vector<std::string> getBarometerLabels() const;
  /**
   * @brief returns all labels for microphones
   */
  std::vector<std::string> getMicrophoneLabels() const;

  /**
   * @brief returns a sensor calibration by its label. Will return `nullopt` if label does not exist
   * in device calibration.
   */
  std::optional<SensorCalibration> getSensorCalib(const std::string& label) const;
  /**
   * @brief returns a camera calibration by its label. Will return `nullopt` if label does not exist
   * in device calibration.
   */
  std::optional<CameraCalibration> getCameraCalib(const std::string& label) const;
  /**
   * @brief returns a imu calibration by its label. Will return `nullopt` if label does not exist in
   * device calibration.
   */
  std::optional<ImuCalibration> getImuCalib(const std::string& label) const;
  /**
   * @brief returns a magnetometer calibration by its label. Will return `nullopt` if label does not
   * exist in device calibration.
   */
  std::optional<MagnetometerCalibration> getMagnetometerCalib(const std::string& label) const;
  /**
   * @brief returns a barometer calibration by its label. Will return `nullopt` if label does not
   * exist in device calibration.
   */
  std::optional<BarometerCalibration> getBarometerCalib(const std::string& label) const;
  /**
   * @brief returns a microphone calibration by its label. Will return `nullopt` if label does not
   * exist in device calibration.
   */
  std::optional<MicrophoneCalibration> getMicrophoneCalib(const std::string& label) const;
  /**
   * @brief returns an array-of-2 of eye tracking cameras' calibration for an Aria device.
   * @return an array of <CameraCalibration, 2> representing left and right ET camera calibrations.
   * <br>Will return std::nullopt if device is not Aria, or it does not contain the valid ET camera
   * calibrations.
   */
  std::optional<AriaEtCalibration> getAriaEtCameraCalib() const;
  /**
   * @brief returns an array-of-7 of mic calibration for an Aria device.
   * @return an array of <MicCalibration, 7>.
   * <br>Will return std::nullopt if device is not Aria, or it does not contain the valid microphone
   * calibrations.
   */
  std::optional<AriaMicCalibration> getAriaMicrophoneCalib() const;

  /**
   * @brief Get the subtype of device. For Aria, this is 'DVT-S' or 'DVT-L' to indicate the size of
   * the Aria unit.
   */
  std::string getDeviceSubtype() const;

  /**
   * @brief returns relative pose between device frame (anchored to a particular sensor defined by
   * `originLabel`) and CPF (central pupil frame), where CPF is a virtual coordinate frame defined
   * in CAD model
   */
  Sophus::SE3d getT_Device_Cpf() const;

  /**
   * @brief returns calibrated `T_Device_Sensor` given a label.
   * You can return the CAD extrinsics value by specifying `getCadValue = true`.
   */
  std::optional<Sophus::SE3d> getT_Device_Sensor(const std::string& label, bool getCadValue = false)
      const;
  /**
   * @brief returns calibrated sensor extrinsics in CPF frame given a label.
   * You can return the CAD extrinsics value by specifying `getCadValue = true`.
   */
  std::optional<Sophus::SE3d> getT_Cpf_Sensor(const std::string& label, bool getCadValue = false)
      const;

  /**
   * @brief obtain the definition of Origin (or Device in T_Device_Sensor)
   */
  const std::string& getOriginLabel() const;

  /**
   * @brief set the folder path of the vignetting mask.
   * @param maskFolderPath The folder path of the vignetting mask.
   * @return void
   */
  void setDevignettingMaskFolderPath(const std::string& maskFolderPath);
  /**
   * @brief Get devignetting mask based on label and image size.
   * devignetting_mask = 1/vignetting_mask
   * devignetted_image = devignetting_mask * original_image
   * @param label The label of the camera
   * now supporting "camera-slam-left", "camera-slam-right", "camera-rgb"
   * @return Vignetting mask in Eigen::MatrixXf format.
   */
  Eigen::MatrixXf loadDevignettingMask(const std::string& label);

 private:
  friend void tryCropAndScaleCameraCalibration(
      DeviceCalibration& deviceCalibration,
      const std::map<std::string, Eigen::Vector2i>& labelToImageResolution);
  void setCameraCalibration(const std::string& label, CameraCalibration& cameraCalib) {
    cameraCalibs_.at(label) = cameraCalib;
    allCalibs_.at(label) = SensorCalibration(cameraCalib);
  }

 private:
  std::map<std::string, CameraCalibration> cameraCalibs_;
  std::map<std::string, ImuCalibration> imuCalibs_;
  std::map<std::string, MagnetometerCalibration> magnetometerCalibs_;
  std::map<std::string, BarometerCalibration> barometerCalibs_;
  std::map<std::string, MicrophoneCalibration> microphoneCalibs_;
  std::map<std::string, SensorCalibration> allCalibs_;

  DeviceCadExtrinsics deviceCadExtrinsics_;

  std::string deviceSubtype_;
  std::string originLabel_ = "";
  std::string devignettingMaskFolderPath_;
};

} // namespace projectaria::tools::calibration
