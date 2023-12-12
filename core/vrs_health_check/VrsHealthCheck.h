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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <vrs/RecordFileReader.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Camera.h"
#include "SensorMisalignmentStats.h"
#include "Stream.h"

namespace projectaria::tools::vrs_check {

struct Settings {
  float maxImuSkipUs = 20000.0; // Max allowed gap in the IMU stream
  int maxFrameDropUs = 300000; // Max sequential frame drop duration in microseconds
  float physicalAccelThreshold =
      20.0; // Max change in acceleration between consecutive IMU measurements
  float maxNonPhysicalAccel = 0.1; // Max proportion [0->1) of non-physical IMU acceleration
                                   // measurements allowed
  float maxAllowedRotationAccel_radPerS2 =
      10000; // 10,000 radPerS2 is a lot - It should very rarely happen.
             // This corresponds to 570 deg/s change per millisecond.

  float defaultImuPeriodUs = 1000.0; // IMU period to use if not set in the stream
  float minImuScore = 95.0; // Score is 100 * (1 - (dropped + bad) / expected)
  float minBaroScore = 95.0; // Score is 100 * (1 - (dropped + bad) / expected)
  float minTemp = -20.0;
  float maxTemp = 70.0;
  float minCameraScore = 95.0; // Score is 100 * (1 - (dropped + bad) / expected)
  double minCameraGain = 1.0;
  double maxCameraGain = 22.0; // Default value for RGB and SLAM cameras
  double minCameraExposureMs = 1e-3; // 1us
  double maxCameraExposureMs = 20.0;
  float minTimeDomainMappingScore = 95.0; // Score is 100 * (1 - (dropped + bad) / expected)
  float minAudioScore = 99.0; // Score is 100 * (1 - (dropped + bad) / expected)
  float minAlignmentScore = 99.0; // Score is 100 * (1 - misaligned / considered))
  // Whether to ignore GPS when determining the pass/fail criteria
  bool ignoreGps = false;
  float minGpsAccuracy = 10.0; // Minimum required GPS accuracy in meters
  float defaultGpsRateHz = 10.0; // Default GPS sample rate in Hz
  bool ignoreAudio = false; // Whether to ignore audio when determining the pass/fail criteria
  bool ignoreBluetooth =
      false; // Whether to ignore bluetooth when determining the pass/fail criteria
  std::unordered_map<::vrs::RecordableTypeId, CameraCheckSetting> cameraCheckSettings;
  bool isInteractive = true; // Whether to show progress interactively
  std::function<void(const std::string&, float, float)> progressCallback = nullptr;
};

class VrsHealthCheck {
 public:
  explicit VrsHealthCheck(Settings settings);
  bool setup(const std::string& path); // Setup readers and playables
  bool run(); // Parse through all records in all files to get the health result
  void logStats(); // Log to console statistics about all streams
  void logStatsJson(const std::string& filepath); // Log to JSON statistics about all streams
  void logDroppedFrames(const std::string& filepath); // Log to csv statistics about dropped
                                                      // frames in all the streams
  // Result of the VRS verification. Determine if the VRS files are useful or not.
  bool getResult();

 private:
  bool setupFiles(const std::string& path); // Setup file readers and playables
  bool setupPlayables(const std::vector<std::string>& filePaths = {});
  void setupSensorHealthStatsMap();
  double getLastDataRecordTime();
  double getFirstDataRecordTime();
  void printProgress();
  const Settings settings_;
  std::vector<std::unique_ptr<vrs::RecordFileReader>> reader_;
  std::vector<std::unique_ptr<Stream>> streams_;
  std::shared_ptr<SensorMisalignmentStats> sensorMisalignmentStats_;
  std::unordered_map<std::string, std::unordered_map<std::string, SensorMisalignmentStatistics>>
      cachedMisalignmentStatistics_;
};

} // namespace projectaria::tools::vrs_check
