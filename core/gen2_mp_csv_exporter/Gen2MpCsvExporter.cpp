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

#include <gen2_mp_csv_exporter/Gen2MpCsvExporter.h>

#include <data_provider/VrsDataProvider.h>
#include <data_provider/data_types/FrontendOutput.h>
#include <gen2_mp_csv_exporter/file_io/MpvCsvWriter.h>
#include <gen2_mp_csv_exporter/file_io/OnlineCalibrationJson.h>
#include <nlohmann/json.hpp>

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::mps;
using namespace projectaria::tools::calibration;

namespace {

// A helper function to convert Frontend Output to OnlineCalibration
OnlineCalibration frontendoutputToOnlineCalibration(
    const FrontendOutput& frontendOutput,
    const DeviceCalibration& deviceCalib) {
  static const std::vector<std::string> kTrackerToSlamCameraLabels = {
      "slam-front-left", "slam-front-right", "slam-side-left", "slam-side-right"};
  static const std::vector<std::string> kTrackerToSlamImuLabels = {"imu-left", "imu-right"};
  const auto maybeT_Device_BodyImu = deviceCalib.getT_Device_Sensor("imu-left");
  if (!maybeT_Device_BodyImu.has_value()) {
    throw std::runtime_error("Cannot find imu-left in original device calibration");
  }

  OnlineCalibration calib;

  std::chrono::nanoseconds timestampInNs(frontendOutput.captureTimestampNs);
  calib.trackingTimestamp = std::chrono::duration_cast<std::chrono::microseconds>(timestampInNs);
  timestampInNs = std::chrono::nanoseconds(frontendOutput.unixTimestampNs);
  calib.utcTimestamp = std::chrono::duration_cast<std::chrono::microseconds>(timestampInNs);

  // Insert slam online calibrations
  if (!frontendOutput.onlineCalib.camParameters.empty()) {
    for (size_t i = 0; i < kTrackerToSlamCameraLabels.size(); ++i) {
      const std::string& label = kTrackerToSlamCameraLabels.at(i);
      const auto maybeOriginalCalib = deviceCalib.getCameraCalib(label);
      if (!maybeOriginalCalib.has_value()) {
        throw std::runtime_error(fmt::format(
            "Cannot find slam calibration for {} in original device calibration \n", label));
      }
      const auto& originalCalib = maybeOriginalCalib.value();

      // Obtain camera extrinsics and intrinsics from TrackerToSlam
      const auto& camParamsVector = frontendOutput.onlineCalib.camParameters.at(i).intrinsics;
      Eigen::VectorXd projectionParams =
          Eigen::VectorXf::Map(camParamsVector.data(), camParamsVector.size()).cast<double>();
      Sophus::SE3d T_Device_Camera = maybeT_Device_BodyImu.value() *
          frontendOutput.onlineCalib.T_Cam_BodyImu.at(i).cast<double>().inverse();

      CameraCalibration slamCalib(
          label,
          originalCalib.modelName(),
          projectionParams,
          T_Device_Camera,
          originalCalib.getImageSize()[0],
          originalCalib.getImageSize()[1],
          originalCalib.getValidRadius(),
          originalCalib.getMaxSolidAngle(),
          originalCalib.getSerialNumber(),
          originalCalib.getTimeOffsetSecDeviceCamera());
      calib.cameraCalibs.emplace_back(slamCalib);
    }
  }

  // Insert IMU online calibration
  if (!frontendOutput.onlineCalib.imuModelParameters.empty()) {
    for (size_t i = 0; i < kTrackerToSlamImuLabels.size(); ++i) {
      const auto& label = kTrackerToSlamImuLabels.at(i);
      const auto& onlineImuParams = frontendOutput.onlineCalib.imuModelParameters.at(i);

      ImuCalibration imuCalib(
          label,
          (onlineImuParams.accelScaleVec.asDiagonal() * onlineImuParams.accelNonorth)
              .cast<double>(),
          (onlineImuParams.accelBiasMSec2).cast<double>(),
          (onlineImuParams.gyroScaleVec.asDiagonal() * onlineImuParams.gyroNonorth).cast<double>(),
          (onlineImuParams.gyroBiasRadSec).cast<double>(),
          maybeT_Device_BodyImu.value() * (onlineImuParams.T_Imu_BodyImu.inverse().cast<double>()));
      calib.imuCalibs.push_back(imuCalib);
    }
  }

  return calib;
}

void writeEyeGazeDataToCsv(
    const std::string& outputFolder,
    std::shared_ptr<VrsDataProvider>& dataProvider,
    const vrs::StreamId& streamId) {
  MpvCsvWriter csvWriter;
  csvWriter.openFile(outputFolder + "generalized_eye_gaze.csv");

  // This function will read high frequency pose data from vrs, and write them to a csv file.
  int64_t numData = dataProvider->getNumData(streamId);
  fmt::print("Number of eye gaze data records in VRS: {}\n", numData);
  for (int i = 0; i < numData; i++) {
    const auto eyegazeData = dataProvider->getEyeGazeDataByIndex(streamId, i);
    csvWriter.insertEyeGazeData(eyegazeData);
  }
  csvWriter.closeFile();
}

void writeVioHighFrequencyPoseDataToCsv(
    const std::string& outputFolder,
    std::shared_ptr<VrsDataProvider>& dataProvider,
    const vrs::StreamId& streamId,
    int subsampleRate) {
  MpvCsvWriter csvWriter;
  csvWriter.openFile(outputFolder + "open_loop_trajectory.csv");

  // This function will read high frequency pose data from vrs, and write them to a csv file.
  int64_t numData = dataProvider->getNumData(streamId);
  fmt::print(
      "Number of vio high frequency data records in VRS: {}, subsample rate is {}\n",
      numData,
      subsampleRate);
  for (int i = 0; i < numData; i += subsampleRate) {
    const auto& vioHighFreqData = dataProvider->getVioHighFreqDataByIndex(streamId, i);
    csvWriter.insertVioHighFrequencyData(vioHighFreqData);
  }
  csvWriter.closeFile();
}

void writeHandPoseDataToCsv(
    const std::string& outputFolder,
    std::shared_ptr<VrsDataProvider>& dataProvider,
    const vrs::StreamId& streamId) {
  MpvCsvWriter csvWriter;
  csvWriter.openFile(outputFolder + "hand_tracking_results.csv");

  // This function will read high frequency pose data from vrs, and write them to a csv file.
  int64_t numData = dataProvider->getNumData(streamId);
  fmt::print("Number of handtracking data records in VRS: {}\n", numData);
  for (int i = 0; i < numData; i++) {
    const auto& handPoseData = dataProvider->getHandPoseDataByIndex(streamId, i);
    csvWriter.insertHandPoseData(handPoseData);
  }
  csvWriter.closeFile();
}

// This function will read online calibration data from vrs, and write them to a jsonl file
void writeOnlineCalibDataToJson(
    const std::string& outputFolder,
    std::shared_ptr<VrsDataProvider>& dataProvider,
    const vrs::StreamId& streamId) {
  // Create a json writer
  projectaria::tools::mps::MpvOnlineCalibJsonWriter jsonWriter;
  const std::string fileName = outputFolder + "online_calibration.jsonl";
  jsonWriter.openFile(fileName);

  // This function will read high frequency pose data from vrs, and write them to a csv file.
  const int64_t numData = dataProvider->getNumData(streamId);
  const auto& maybeDeviceCalib = dataProvider->getDeviceCalibration();
  if (!maybeDeviceCalib.has_value()) {
    throw std::runtime_error("Cannot find device calibration in vrs file");
  }

  fmt::print("Number of VIO data records in VRS: {}\n", numData);
  for (int i = 0; i < numData; i++) {
    const auto& frontendOutput = dataProvider->getVioDataByIndex(streamId, i);
    if (frontendOutput.status == VioStatus::INVALID) {
      fmt::print("Vio status is invalid, skipping this record. \n");
      continue;
    }
    // Skip if there is no online calibration
    if (frontendOutput.onlineCalib.camParameters.empty() &&
        frontendOutput.onlineCalib.imuModelParameters.empty()) {
      fmt::print("No online calibration data, skipping this record. \n");
      continue;
    }

    const auto onlineCalib =
        frontendoutputToOnlineCalibration(frontendOutput, maybeDeviceCalib.value());

    // Write to jsonl file
    jsonWriter.insertOnlineCalibData(onlineCalib);
  }
  jsonWriter.closeFile();

  fmt::print("Data from VRS has been written to {}\n", fileName);
}

} // namespace

namespace projectaria::tools::mp_csv_exporter {

void runGen2MPCsvExporter(
    const std::string& vrsPath,
    const std::string& outputFolder,
    int vioHighFreqSubsampleRate) {
  // Create Vrs data provider
  auto provider = createVrsDataProvider(vrsPath);

  // Write EyeGaze
  const auto& maybeEyegazeStreamId = provider->getStreamIdFromLabel("eyegaze");
  if (maybeEyegazeStreamId.has_value()) {
    writeEyeGazeDataToCsv(outputFolder, provider, maybeEyegazeStreamId.value());
  }

  // Write VioHighFrequencyPose
  const auto& maybeVioHighFreqStreamId = provider->getStreamIdFromLabel("vio_high_frequency");
  if (maybeVioHighFreqStreamId.has_value()) {
    writeVioHighFrequencyPoseDataToCsv(
        outputFolder, provider, maybeVioHighFreqStreamId.value(), vioHighFreqSubsampleRate);
  }

  // Write HandPose
  const auto& maybeHandPoseStreamId = provider->getStreamIdFromLabel("handtracking");
  if (maybeHandPoseStreamId.has_value()) {
    writeHandPoseDataToCsv(outputFolder, provider, maybeHandPoseStreamId.value());
  }

  // Write online calib
  const auto& maybeVioStreamId = provider->getStreamIdFromLabel("vio");
  if (maybeVioStreamId.has_value()) {
    writeOnlineCalibDataToJson(outputFolder, provider, maybeVioStreamId.value());
  }
}

} // namespace projectaria::tools::mp_csv_exporter
