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

#include "Periodic.h"

#include <data_provider/players/ImageSensorPlayer.h>

namespace projectaria::tools::vrs_check {

struct CameraStats {
  uint64_t longestFrameDropUs = 0;
  uint64_t tempOutOfRange =
      0; // Temperature readings that will compromise the function of other sensors
};

// A struct to represent the camera roi check criteria
struct CameraCheckSetting {
  std::string cameraTypeName; // this field is for facilitating CLI11 parsing in the binary tool
  int maxFrameCounts = std::numeric_limits<int>::max();
  int minFrameCounts = -std::numeric_limits<int>::max();
  int maxRoiBadFrames = std::numeric_limits<int>::max();
  Eigen::AlignedBox2i roiToCheck = Eigen::AlignedBox2i();
  double roiLowerThresh = -std::numeric_limits<double>::max();
  double roiUpperThresh = std::numeric_limits<double>::max();
};

class Camera : public Periodic {
 public:
  Camera(
      vrs::StreamId streamId,
      float minScore,
      int maxFrameDropUs,
      double minGain,
      double maxGain,
      double minExposure,
      double maxExposure,
      float minTemp,
      float maxTemp,
      const CameraCheckSetting& cameraCheckSetting);
  bool setup(vrs::RecordFileReader& reader) override; // Setup the camera player
  CameraStats getCameraStats(); // Get stats specific to camera sensors
  void logStats() override;
  bool getResult() override; // Pass or fail for this stream
  nlohmann::json statsToJson() override;

 private:
  void processData(
      const data_provider::ImageData& data,
      const data_provider::ImageDataRecord& record);
  void processExposure(uint64_t exposureDurationUs, uint64_t frameCenterExposureUs);
  void processGain(uint64_t frameCenterExposureUs, float gain);
  void processFrameSkip(uint64_t frameCenterExposureUs);

  // This function will check if the sensor serial numbers match between the stream config record
  // and the factory calibration blob.
  // For reference, here is json dump of the config record from each stream for
  // gaia:1065417714253625 where the sensor serial numbers for rgb sensor don't match between the
  // rgb stream config record and the factory calibration blob. P537149483
  void performSensorSerialCheck(vrs::RecordFileReader& reader);

  std::unique_ptr<data_provider::ImageSensorPlayer> imageSensorPlayer_;
  const int maxFrameDropUs_;
  double minGain_;
  double maxGain_;
  double minExposureMs_;
  double maxExposureMs_;
  float minTemp_;
  float maxTemp_;
  CameraStats cameraStats_;
  const CameraCheckSetting cameraCheckSetting_;
  int totalFrames_ = 0;
  int roiBadFrames_ = 0;
  uint64_t prevExposureDurationUs_ = 0;
  uint64_t unphysicalExposureTime_ = 0;
  uint64_t gainOutOfRange_ = 0;
  uint64_t exposureOutOfRange_ = 0;
  bool calibrationSensorSerialsMatch_ = false;
  bool factoryCalibrationValid_ = false;
  bool factoryCalibrationConsistent_ = false;
};

} // namespace projectaria::tools::vrs_check
