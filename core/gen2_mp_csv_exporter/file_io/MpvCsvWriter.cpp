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
#include <gen2_mp_csv_exporter/file_io/MpvCsvWriter.h>
#include <iostream>

namespace {
const std::string kEyeGazeCsvHeader =
    "tracking_timestamp_us,left_yaw_rads_cpf,right_yaw_rads_cpf,pitch_rads_cpf,depth_m,"
    "left_yaw_low_rads_cpf,right_yaw_low_rads_cpf,pitch_low_rads_cpf,"
    "left_yaw_high_rads_cpf,right_yaw_high_rads_cpf,pitch_high_rads_cpf,"
    "tx_left_eye_cpf,ty_left_eye_cpf,tz_left_eye_cpf,"
    "tx_right_eye_cpf,ty_right_eye_cpf,tz_right_eye_cpf,session_uid\n";
const std::string kVioHighFreqCsvHeader =
    "tracking_timestamp_us,utc_timestamp_ns,session_uid,"
    "tx_odometry_device,ty_odometry_device,tz_odometry_device,"
    "qx_odometry_device,qy_odometry_device,qz_odometry_device,qw_odometry_device,"
    "device_linear_velocity_x_odometry,device_linear_velocity_y_odometry,device_linear_velocity_z_odometry,"
    "angular_velocity_x_device,angular_velocity_y_device,angular_velocity_z_device,"
    "gravity_x_odometry,gravity_y_odometry,gravity_z_odometry,quality_score\n";

// This helper function gets the hand pose csv header. It uses a loop to generate the landmark
// headers
std::string getHandPoseCsvHeader() {
  std::string header = "tracking_timestamp_us,left_tracking_confidence";
  // Left landmarks
  for (int iLandmark = 0; iLandmark < projectaria::tools::mps::kNumHandLandmarks; ++iLandmark) {
    header += fmt::format(
        ",tx_left_landmark_{}_device,ty_left_landmark_{}_device,tz_left_landmark_{}_device",
        iLandmark,
        iLandmark,
        iLandmark);
  }
  header += ",right_tracking_confidence";
  // Right landmarks
  for (int iLandmark = 0; iLandmark < projectaria::tools::mps::kNumHandLandmarks; ++iLandmark) {
    header += fmt::format(
        ",tx_right_landmark_{}_device,ty_right_landmark_{}_device,tz_right_landmark_{}_device",
        iLandmark,
        iLandmark,
        iLandmark);
  }
  // Other fields
  header +=
      ",tx_left_device_wrist,ty_left_device_wrist,tz_left_device_wrist, "
      "qx_left_device_wrist, qy_left_device_wrist, qz_left_device_wrist, qw_left_device_wrist,"
      "tx_right_device_wrist,ty_right_device_wrist,tz_right_device_wrist,"
      "qx_right_device_wrist, qy_right_device_wrist, qz_right_device_wrist, qw_right_device_wrist,"
      "nx_left_palm_device,ny_left_palm_device,nz_left_palm_device,nx_left_wrist_device,ny_left_wrist_device,nz_left_wrist_device,"
      "nx_right_palm_device,ny_right_palm_device,nz_right_palm_device,nx_right_wrist_device,ny_right_wrist_device,nz_right_wrist_device\n";
  return header;
}
} // namespace

namespace projectaria::tools::data_provider {

bool MpvCsvWriter::openFile(const std::string& csvFilename) {
  csvFile_.open(csvFilename);
  return csvFile_.is_open();
}

void MpvCsvWriter::closeFile() {
  if (csvFile_.is_open()) {
    csvFile_.close();
  }
}

void MpvCsvWriter::insertEyeGazeData(const mps::EyeGaze& eyeGazeData) {
  if (!csvFile_.is_open()) {
    fmt::print("ERROR: File not open. Cannot insert data.\n");
    return;
  }
  if (std::isnan(eyeGazeData.depth) || std::isinf(eyeGazeData.depth)) {
    return; // Skip invalid data
  }

  // Write header if this is the first data row
  if (!eyeGazeHeaderWritten_) {
    csvFile_ << kEyeGazeCsvHeader;
    eyeGazeHeaderWritten_ = true;
  }

  // Write csv data as format string
  std::string formattedData = fmt::format(
      "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
      eyeGazeData.trackingTimestamp.count(),
      eyeGazeData.vergence.left_yaw,
      eyeGazeData.vergence.right_yaw,
      eyeGazeData.pitch,
      eyeGazeData.depth,
      eyeGazeData.vergence.left_yaw_low,
      eyeGazeData.vergence.right_yaw_low,
      eyeGazeData.pitch_low,
      eyeGazeData.vergence.left_yaw_high,
      eyeGazeData.vergence.right_yaw_high,
      eyeGazeData.pitch_high,
      eyeGazeData.vergence.tx_left_eye,
      eyeGazeData.vergence.ty_left_eye,
      eyeGazeData.vergence.tz_left_eye,
      eyeGazeData.vergence.tx_right_eye,
      eyeGazeData.vergence.ty_right_eye,
      eyeGazeData.vergence.tz_right_eye,
      eyeGazeData.session_uid);
  csvFile_ << formattedData;
}

void MpvCsvWriter::insertVioHighFrequencyData(
    const mps::OpenLoopTrajectoryPose& OpenLoopTrajectoryPoseData) {
  if (!csvFile_.is_open()) {
    fmt::print("ERROR: File not open. Cannot insert data. \n");
    return;
  }
  // Write header if this is the first data row
  if (!vioHighFreqHeaderWritten_) {
    csvFile_ << kVioHighFreqCsvHeader;
    vioHighFreqHeaderWritten_ = true;
  }
  // Write csv data as format string
  std::string formattedData = fmt::format(
      "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
      OpenLoopTrajectoryPoseData.trackingTimestamp.count(),
      OpenLoopTrajectoryPoseData.utcTimestamp.count(),
      OpenLoopTrajectoryPoseData.sessionUid,

      OpenLoopTrajectoryPoseData.T_odometry_device.translation()[0],
      OpenLoopTrajectoryPoseData.T_odometry_device.translation()[1],
      OpenLoopTrajectoryPoseData.T_odometry_device.translation()[2],

      OpenLoopTrajectoryPoseData.T_odometry_device.unit_quaternion().x(),
      OpenLoopTrajectoryPoseData.T_odometry_device.unit_quaternion().y(),
      OpenLoopTrajectoryPoseData.T_odometry_device.unit_quaternion().z(),
      OpenLoopTrajectoryPoseData.T_odometry_device.unit_quaternion().w(),

      OpenLoopTrajectoryPoseData.deviceLinearVelocity_odometry[0],
      OpenLoopTrajectoryPoseData.deviceLinearVelocity_odometry[1],
      OpenLoopTrajectoryPoseData.deviceLinearVelocity_odometry[2],

      OpenLoopTrajectoryPoseData.angularVelocity_device[0],
      OpenLoopTrajectoryPoseData.angularVelocity_device[1],
      OpenLoopTrajectoryPoseData.angularVelocity_device[2],

      OpenLoopTrajectoryPoseData.gravity_odometry[0],
      OpenLoopTrajectoryPoseData.gravity_odometry[1],
      OpenLoopTrajectoryPoseData.gravity_odometry[2],

      OpenLoopTrajectoryPoseData.qualityScore);
  csvFile_ << formattedData;
}

void MpvCsvWriter::insertHandPoseData(const mps::HandTrackingResult& handPoseData) {
  if (!csvFile_.is_open()) {
    fmt::print("ERROR: File not open. Cannot insert data. \n");
    return;
  }
  // Write header if this is the first data row
  if (!handPoseHeaderWritten_) {
    csvFile_ << getHandPoseCsvHeader();
    handPoseHeaderWritten_ = true;
  }

  // Create default single-side hand pose struct, if any of the hand data does not exist
  const Eigen::Vector3d trivialNormal(0, 0, 1);

  mps::HandTrackingResult::OneSide leftHand =
      handPoseData.leftHand.value_or(mps::HandTrackingResult::OneSide());
  mps::HandTrackingResult::OneSide rightHand =
      handPoseData.rightHand.value_or(mps::HandTrackingResult::OneSide());
  mps::HandTrackingResult::OneSide::WristAndPalmNormals leftNormals =
      leftHand.wristAndPalmNormal_device.value_or(
          mps::HandTrackingResult::OneSide::WristAndPalmNormals{trivialNormal, trivialNormal});
  mps::HandTrackingResult::OneSide::WristAndPalmNormals rightNormals =
      rightHand.wristAndPalmNormal_device.value_or(
          mps::HandTrackingResult::OneSide::WristAndPalmNormals{trivialNormal, trivialNormal});
  // Set default confidence value to -1.0 if hand data is missing
  if (!handPoseData.leftHand.has_value()) {
    leftHand.confidence = -1.0;
  }
  if (!handPoseData.rightHand.has_value()) {
    rightHand.confidence = -1.0;
  }
  // Write csv data as format string
  std::string formattedData =
      fmt::format("{},{}", handPoseData.trackingTimestamp.count(), leftHand.confidence);
  // Append landmark data
  for (const auto& leftLandmark : leftHand.landmarkPositions_device) {
    formattedData += fmt::format(",{},{},{}", leftLandmark.x(), leftLandmark.y(), leftLandmark.z());
  }
  formattedData += fmt::format(",{}", rightHand.confidence);
  for (const auto& rightLandmark : rightHand.landmarkPositions_device) {
    formattedData +=
        fmt::format(",{},{},{}", rightLandmark.x(), rightLandmark.y(), rightLandmark.z());
  }

  // Append wrist poses
  const auto& T_Device_LeftWrist = leftHand.T_Device_Wrist;
  const auto& T_Device_RightWrist = rightHand.T_Device_Wrist;
  formattedData += fmt::format(
      ",{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
      T_Device_LeftWrist.translation()[0],
      T_Device_LeftWrist.translation()[1],
      T_Device_LeftWrist.translation()[2],
      T_Device_LeftWrist.unit_quaternion().x(),
      T_Device_LeftWrist.unit_quaternion().y(),
      T_Device_LeftWrist.unit_quaternion().z(),
      T_Device_LeftWrist.unit_quaternion().w(),
      T_Device_RightWrist.translation()[0],
      T_Device_RightWrist.translation()[1],
      T_Device_RightWrist.translation()[2],
      T_Device_RightWrist.unit_quaternion().x(),
      T_Device_RightWrist.unit_quaternion().y(),
      T_Device_RightWrist.unit_quaternion().z(),
      T_Device_RightWrist.unit_quaternion().w());

  // Append palm and wrist normals
  formattedData += fmt::format(
      ",{},{},{},{},{},{},{},{},{},{},{},{}\n",
      leftNormals.palmNormal_device.x(),
      leftNormals.palmNormal_device.y(),
      leftNormals.palmNormal_device.z(),
      leftNormals.wristNormal_device.x(),
      leftNormals.wristNormal_device.y(),
      leftNormals.wristNormal_device.z(),
      rightNormals.palmNormal_device.x(),
      rightNormals.palmNormal_device.y(),
      rightNormals.palmNormal_device.z(),
      rightNormals.wristNormal_device.x(),
      rightNormals.wristNormal_device.y(),
      rightNormals.wristNormal_device.z());

  csvFile_ << formattedData;
}

} // namespace projectaria::tools::data_provider
