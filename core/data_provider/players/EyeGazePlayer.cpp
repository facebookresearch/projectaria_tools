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

#include <fmt/format.h>
#include <cmath>

#include <data_provider/players/DataPlayerUtils.h>
#include <data_provider/players/EyeGazePlayer.h>

namespace projectaria::tools::data_provider {

using namespace projectaria::tools::mps;

namespace {

// A helper function to convert a direction [xyz] to returns yaw and pitch
// angles [rad]
std::pair<float, float> getYawPitch(const Eigen::Vector3f& xyz) {
  // Convert to yaw/pitch angles, whose definition can be found in
  // `EyeGazeReader.h:getUnitVectorFromYawPitch`
  float yaw = std::atan2(xyz[0], xyz[2]);
  float pitch = std::atan2(xyz[1], xyz[2]);
  return std::make_pair(yaw, pitch);
}

// A helper function to populate gaze and origin from Device to CPF
// Returns (origin, yaw, pitch)
std::tuple<Eigen::Vector3f, float, float> populateGazeOriginAndDirection(
    const vrs::Point3Df& gazeOriginInDevice,
    const vrs::Point3Df& gazeDirectionInDevice,
    const Sophus::SE3f& T_Cpf_Device) {
  const Eigen::Vector3f gazeOriginInCpf = T_Cpf_Device * mapToEigenVector(gazeOriginInDevice);
  const Eigen::Vector3f gazeDirectionInCpf =
      T_Cpf_Device.so3() * mapToEigenVector(gazeDirectionInDevice);
  auto [yaw, pitch] = getYawPitch(gazeDirectionInCpf);

  return std::make_tuple(gazeOriginInCpf, yaw, pitch);
}

// Helper function to populate EyeGaze data from VRS data layout to OSS data type
std::optional<EyeGaze> populateOssEyeGaze(
    const datalayout::EyeGazeLayout& dataLayout,
    const Sophus::SE3f& T_Cpf_Device) {
  EyeGaze eyeData;

  // First parse timestamps
  int64_t timestampNs = 0;
  if (!dataLayout.systemTimestampNs.get(timestampNs)) {
    fmt::print("Missing systemTimestampNs in eye gaze data layout! \n");
    return std::nullopt;
  } else {
    eyeData.trackingTimestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::nanoseconds(timestampNs));
  }

  // Parsing combined gaze origin and direction, if they are valid
  vrs::Point3Df vrsGazeOrigin, vrsGazeDirection;
  if (dataLayout.gazeOriginCombinedInDeviceValid.get() &&
      dataLayout.gazeDirectionCombinedInDeviceValid.get() &&
      dataLayout.gazeOriginCombinedInDeviceMeterXyz.get(vrsGazeOrigin) &&
      dataLayout.gazeDirectionCombinedInDeviceNormalizedXyz.get(vrsGazeDirection)) {
    // Parse origin and direction
    std::tie(eyeData.combined_gaze_origin_in_cpf, eyeData.yaw, eyeData.pitch) =
        populateGazeOriginAndDirection(vrsGazeOrigin, vrsGazeDirection, T_Cpf_Device);
    // Set validity flag
    eyeData.combined_gaze_valid = true;
  } else {
    eyeData.combined_gaze_valid = false;
  }

  // Parsing spatial gaze point, if it is valid
  vrs::Point3Df vrsGazePoint;
  if (dataLayout.spatialGazePointValid.get() &&
      dataLayout.spatialGazePointInDeviceMeterXyz.get(vrsGazePoint)) {
    // Transform to CPF
    const auto spatialGazePointInDevice = mapToEigenVector(vrsGazePoint);
    eyeData.spatial_gaze_point_in_cpf = T_Cpf_Device * spatialGazePointInDevice;
    eyeData.depth =
        (eyeData.spatial_gaze_point_in_cpf - eyeData.combined_gaze_origin_in_cpf).norm();
    eyeData.spatial_gaze_point_valid = true;
  } else {
    eyeData.spatial_gaze_point_valid = false;
  }

  // Parsing left eye gaze direction and origin
  if (dataLayout.leftEye.gazeOriginValid.get() && dataLayout.leftEye.gazeDirectionValid.get() &&
      dataLayout.leftEye.gazeOriginInDeviceMeterXyz.get(vrsGazeOrigin) &&
      dataLayout.leftEye.gazeDirectionInDeviceNormalizedXyz.get(vrsGazeDirection)) {
    Eigen::Vector3f gazeOriginInCpf;
    std::tie(gazeOriginInCpf, eyeData.vergence.left_yaw, eyeData.vergence.left_pitch) =
        populateGazeOriginAndDirection(vrsGazeOrigin, vrsGazeDirection, T_Cpf_Device);
    eyeData.vergence.tx_left_eye = gazeOriginInCpf[0];
    eyeData.vergence.ty_left_eye = gazeOriginInCpf[1];
    eyeData.vergence.tz_left_eye = gazeOriginInCpf[2];

    eyeData.vergence.left_gaze_valid = true;
  } else {
    eyeData.vergence.left_gaze_valid = false;
  }

  // Parsing right eye gaze direction and origin
  if (dataLayout.rightEye.gazeOriginValid.get() && dataLayout.rightEye.gazeDirectionValid.get() &&
      dataLayout.rightEye.gazeOriginInDeviceMeterXyz.get(vrsGazeOrigin) &&
      dataLayout.rightEye.gazeDirectionInDeviceNormalizedXyz.get(vrsGazeDirection)) {
    Eigen::Vector3f gazeOriginInCpf;
    std::tie(gazeOriginInCpf, eyeData.vergence.right_yaw, eyeData.vergence.right_pitch) =
        populateGazeOriginAndDirection(vrsGazeOrigin, vrsGazeDirection, T_Cpf_Device);
    eyeData.vergence.tx_right_eye = gazeOriginInCpf[0];
    eyeData.vergence.ty_right_eye = gazeOriginInCpf[1];
    eyeData.vergence.tz_right_eye = gazeOriginInCpf[2];

    eyeData.vergence.right_gaze_valid = true;
  } else {
    eyeData.vergence.right_gaze_valid = false;
  }

  // Parsing left and right entrance pupil position, transform them to CPF space
  vrs::Point3Df vrsEntrancePupilPosition;
  if (dataLayout.leftEye.entrancePupilPositionValid.get() &&
      dataLayout.leftEye.entrancePupilPositionInDeviceMeterXyz.get(vrsEntrancePupilPosition)) {
    eyeData.vergence.left_entrance_pupil_position_valid = true;
    const auto entrance_position_in_device = mapToEigenVector(vrsEntrancePupilPosition);
    eyeData.vergence.left_entrance_pupil_position_meter =
        T_Cpf_Device * entrance_position_in_device;

  } else {
    eyeData.vergence.right_entrance_pupil_position_valid = true;
  }
  if (dataLayout.rightEye.entrancePupilPositionValid.get() &&
      dataLayout.rightEye.entrancePupilPositionInDeviceMeterXyz.get(vrsEntrancePupilPosition)) {
    eyeData.vergence.right_entrance_pupil_position_valid = true;
    const auto entrance_position_in_device = mapToEigenVector(vrsEntrancePupilPosition);
    eyeData.vergence.right_entrance_pupil_position_meter =
        T_Cpf_Device * entrance_position_in_device;
  } else {
    eyeData.vergence.right_entrance_pupil_position_valid = true;
  }

  // Parsing left and right eye pupil diameters
  float vrsPupilDiameter;
  if (dataLayout.leftEye.pupilDiameterValid.get() &&
      dataLayout.leftEye.pupilDiameterMeter.get(vrsPupilDiameter)) {
    eyeData.vergence.left_pupil_diameter_valid = true;
    eyeData.vergence.left_pupil_diameter_meter = vrsPupilDiameter;
  } else {
    eyeData.vergence.left_pupil_diameter_valid = false;
  }
  if (dataLayout.rightEye.pupilDiameterValid.get() &&
      dataLayout.rightEye.pupilDiameterMeter.get(vrsPupilDiameter)) {
    eyeData.vergence.right_pupil_diameter_valid = true;
    eyeData.vergence.right_pupil_diameter_meter = vrsPupilDiameter;
  } else {
    eyeData.vergence.right_pupil_diameter_valid = false;
  }

  // Parsing left eye blink
  if (dataLayout.leftEye.blinkValid.get()) {
    eyeData.vergence.left_blink = dataLayout.leftEye.blink.get();
    eyeData.vergence.left_blink_valid = true;
  } else {
    eyeData.vergence.left_blink_valid = false;
  }

  // Parsing right eye blink
  if (dataLayout.rightEye.blinkValid.get()) {
    eyeData.vergence.right_blink = dataLayout.rightEye.blink.get();
    eyeData.vergence.right_blink_valid = true;
  } else {
    eyeData.vergence.right_blink_valid = false;
  }

  return eyeData;
}

} // namespace

bool EyeGazePlayer::onDataLayoutRead(
    const vrs::CurrentRecord& header,
    size_t blockIndex,
    vrs::DataLayout& layout) {
  if (header.recordType == vrs::Record::Type::CONFIGURATION) {
    // Read config
    const datalayout::EyeGazeConfigurationLayout& configLayout =
        getExpectedLayout<datalayout::EyeGazeConfigurationLayout>(layout, blockIndex);
    if (!configLayout.streamId.get(configRecord_.streamId)) {
      fmt::print("Missing streamId in eye gaze config layout! \n");
      return false;
    }
    if (!configLayout.nominalRateHz.get(configRecord_.nominalRateHz)) {
      fmt::print("Missing nominalRateHz in eye gaze config layout! \n");
      return false;
    }
    if (!configLayout.userCalibrationError.get(configRecord_.userCalibrationError)) {
      fmt::print("Missing userCalibrationError in eye gaze config layout! \n");
      return false;
    }
    configRecord_.streamId = configLayout.streamId.get();
    configRecord_.nominalRateHz = configLayout.nominalRateHz.get();
    configRecord_.userCalibrated = configLayout.userCalibrated.get();
    configRecord_.userCalibrationError = configLayout.userCalibrationError.get();
    return true;
  } else if (header.recordType == vrs::Record::Type::DATA) {
    // Read data as datalayout data type
    const datalayout::EyeGazeLayout& dataLayout =
        getExpectedLayout<datalayout::EyeGazeLayout>(layout, blockIndex);

    // Check if T_Cpf_Device pose is valid
    if (!T_Cpf_Device_.has_value()) {
      throw std::runtime_error(
          "Eye Gaze Player needs to correctly initialize with or explicitly set T_Cpf_Device, in order to correctly convert Gen2 EyeGaze data to Gen1 data type");
    }
    const auto maybeEyeGaze = populateOssEyeGaze(dataLayout, T_Cpf_Device_.value());
    if (!maybeEyeGaze.has_value()) {
      fmt::print("ERROR: Failed to read in Eye Gaze data from VRS, skipping this record \n");
      return false;
    }
    dataRecord_ = maybeEyeGaze.value();

    // Invoke additional callback set by the user.
    if (callback_) {
      callback_(dataRecord_, configRecord_, verbose_);
    }

    return true;
  }
  return false;
}
} // namespace projectaria::tools::data_provider
