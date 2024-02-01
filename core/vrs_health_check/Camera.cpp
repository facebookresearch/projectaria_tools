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

#include "Camera.h"

#include <calibration/DeviceCalibration.h>
#include <calibration/loader/DeviceCalibrationJson.h>

#include <cmath>
#include <iostream>
#include <iterator>
#include <string_view>
#include <utility>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:Camera"
#include <logging/Log.h>

#include "Logging.h"

namespace projectaria::tools::vrs_check {

Camera::Camera(
    const vrs::StreamId streamId,
    float minScore,
    int maxFrameDropUs,
    double minGain,
    double maxGain,
    double minExposureMs,
    double maxExposureMs,
    float minTemp,
    float maxTemp,
    const CameraCheckSetting& cameraCheckSetting)
    : Periodic(streamId, minScore),
      maxFrameDropUs_(maxFrameDropUs),
      minGain_(minGain),
      maxGain_(maxGain),
      minExposureMs_(minExposureMs),
      maxExposureMs_(maxExposureMs),
      minTemp_(minTemp),
      maxTemp_(maxTemp),
      cameraCheckSetting_(cameraCheckSetting) {}

bool Camera::setup(vrs::RecordFileReader& reader) {
  // Setup playable
  data_provider::ImageCallback callback = [this](
                                              const data_provider::ImageData& data,
                                              const data_provider::ImageDataRecord& record,
                                              const data_provider::ImageConfigRecord&,
                                              bool) {
    this->processData(data, record);
    return true;
  };
  imageSensorPlayer_ = std::make_unique<data_provider::ImageSensorPlayer>(streamId_);
  if (!imageSensorPlayer_) {
    XR_LOGE("Cannot create playable for {}", streamId_.getName());
    return false;
  }

  imageSensorPlayer_->setCallback(callback);
  reader.setStreamPlayer(streamId_, imageSensorPlayer_.get());

  // Parse the configuration record
  if (!reader.readFirstConfigurationRecord(streamId_)) {
    XR_LOGE("Stream {} is missing a configuration record", streamId_.getName());
    return false;
  }

  data_provider::ImageConfigRecord config = imageSensorPlayer_->getConfigRecord();
  periodUs_ = uint32_t(round(1.0 / config.nominalRateHz * 1e6));
  totalFrames_ = reader.getRecordCount(streamId_, ::vrs::Record::Type::DATA);
  minGain_ = config.gainMin;
  maxGain_ = config.gainMax;
  minExposureMs_ = config.exposureDurationMin * 1e3;
  maxExposureMs_ = config.exposureDurationMax * 1e3;

  performSensorSerialCheck(reader);
  setMaxDeviationFromPeriodUs();
  preprocessStream(reader);

  return true;
}

void Camera::performSensorSerialCheck(vrs::RecordFileReader& reader) {
  std::unique_lock lock{mutex_};
  data_provider::ImageConfigRecord config = imageSensorPlayer_->getConfigRecord();

  // We do a string comparison of factory calibration from the vrs file and the factory
  // calibration from the stream config record.
  const auto factoryCalibrationJsonStr = reader.getTag("calib_json");
  if (factoryCalibrationJsonStr == config.factoryCalibration) {
    factoryCalibrationConsistent_ = true;
  } else {
    XR_LOGE(
        "Factory calibration from file tag doesn't match the factory calibration in the config record.");
  }

  std::optional<calibration::DeviceCalibration> maybeFactoryCalib =
      calibration::deviceCalibrationFromJson(factoryCalibrationJsonStr);

  if (!maybeFactoryCalib) {
    XR_LOGE("Error parsing factory calibration");
    return;
  }
  factoryCalibrationValid_ = maybeFactoryCalib.has_value();
  std::unordered_set<std::string> sensorSerials;
  std::vector<calibration::CameraCalibration> camCalibs;
  for (const auto& camLabel : maybeFactoryCalib.value().getCameraLabels()) {
    auto camCalib = maybeFactoryCalib.value().getCameraCalib(camLabel);
    if (camCalib) {
      camCalibs.push_back(camCalib.value());
    }
  }

  std::transform(
      camCalibs.begin(),
      camCalibs.end(),
      std::inserter(sensorSerials, sensorSerials.begin()),
      [](const auto& calib) { return calib.getSerialNumber(); });
  if (sensorSerials.size() != camCalibs.size()) {
    XR_LOGE("Potential duplicate serial numbers found in the camera calibration");
    factoryCalibrationValid_ = false;
  }
  calibrationSensorSerialsMatch_ = (sensorSerials.count(config.sensorSerial) == 1);
}

CameraStats Camera::getCameraStats() {
  std::lock_guard lock{mutex_};
  return cameraStats_;
}

nlohmann::json Camera::statsToJson() {
  nlohmann::json jsonStats = Periodic::statsToJson();
  std::unique_lock lock{mutex_};
  const CameraStats stats = cameraStats_;
  jsonStats["longest_frame_drop_us"] = stats.longestFrameDropUs;
  jsonStats["roi_bad_frames"] = roiBadFrames_;
  jsonStats["num_frames_with_unphysical_exposure_time"] = unphysicalExposureTime_;
  jsonStats["gain_out_of_range"] = gainOutOfRange_;
  jsonStats["exposure_out_of_range"] = exposureOutOfRange_;
  jsonStats["temp_out_of_range"] = stats.tempOutOfRange;
  jsonStats["calibration_sensor_serials_match"] = calibrationSensorSerialsMatch_;
  jsonStats["factory_calibration_valid"] = factoryCalibrationValid_;
  jsonStats["factory_calibration_consistent"] = factoryCalibrationConsistent_;
  return jsonStats;
}

void Camera::logStats() {
  std::unique_lock lock{mutex_};
  std::cout
      << fmt::format(
             "{}: longestFrameDropDuration={}us roiBadFrames={} gainOutOfRange={} exposureOutOfRange={} tempOutOfRange={}",
             streamId_.getName(),
             cameraStats_.longestFrameDropUs,
             roiBadFrames_,
             gainOutOfRange_,
             exposureOutOfRange_,
             cameraStats_.tempOutOfRange)
      << std::endl;
  lock.unlock();
  Periodic::logStats();
}

bool Camera::getResult() {
  float score = getScore();

  if (score < minScore_) {
    XR_LOGE("{}: Score {}% is less than minimum {}%", streamId_.getName(), score, minScore_);
    return false;
  } else if (cameraStats_.longestFrameDropUs > maxFrameDropUs_) {
    XR_LOGE(
        "{}: Longest frame drop {}us is over max allowed {}us",
        streamId_.getName(),
        cameraStats_.longestFrameDropUs,
        maxFrameDropUs_);
    return false;
  } else if (
      totalFrames_ < cameraCheckSetting_.minFrameCounts ||
      totalFrames_ > cameraCheckSetting_.maxFrameCounts) {
    XR_LOGE(
        "{}: Total number of frames {} is out of the range of [{}, {}]",
        streamId_.getName(),
        totalFrames_,
        cameraCheckSetting_.minFrameCounts,
        cameraCheckSetting_.maxFrameCounts);
    return false;
  } else if (roiBadFrames_ > cameraCheckSetting_.maxRoiBadFrames) {
    XR_LOGE(
        "{}: number of bad frames within ROI {} is over max allowed {}",
        streamId_.getName(),
        roiBadFrames_,
        cameraCheckSetting_.maxRoiBadFrames);
    return false;
  } else if (unphysicalExposureTime_ > 0) {
    XR_LOGE(
        "{}: reported {} not physically possible exposure time",
        streamId_.getName(),
        unphysicalExposureTime_);
    return false;
  }
  return true;
}

void Camera::processExposure(uint64_t exposureDurationUs, uint64_t frameCenterExposureUs) {
  // Physically impossible exposures are a failure.
  // This checks that the exposure durations do not overlap between frames
  if (frameCenterExposureUs - exposureDurationUs / 2 <=
      prevTimestampUs_ + prevExposureDurationUs_ / 2) {
    XR_LOGE(
        "{}: "
        "Exposure times and center capture times are not physically possible: "
        "Exposure of {}us around ts={}us overlaps exposure of {}us around ts={}us",
        streamId_.getName(),
        prevExposureDurationUs_,
        prevTimestampUs_,
        exposureDurationUs,
        frameCenterExposureUs);
    unphysicalExposureTime_++;
  }
  const double exposureDurationMs = exposureDurationUs * 1e-3;
  if ((exposureDurationMs < minExposureMs_) || (exposureDurationMs > maxExposureMs_)) {
    XR_LOGE(
        "{}: Exposure time {}ms at ts={}us is out of range [{}ms, {}ms]",
        streamId_.getName(),
        exposureDurationMs,
        frameCenterExposureUs,
        minExposureMs_,
        maxExposureMs_);
    exposureOutOfRange_++;
  }
  prevExposureDurationUs_ = exposureDurationUs;
}

void Camera::processGain(uint64_t frameCenterExposureUs, float gain) {
  if ((gain < minGain_) || (gain > maxGain_)) {
    XR_LOGE(
        "{}: "
        "Gain value of frame at ts={}us is not within healthy range [{}, {}]",
        streamId_.getName(),
        frameCenterExposureUs,
        minGain_,
        maxGain_);
    gainOutOfRange_++;
  }
}

void Camera::processData(
    const data_provider::ImageData& data,
    const data_provider::ImageDataRecord& record) {
  std::lock_guard lock{mutex_};
  const uint64_t frameCenterExposureUs = record.captureTimestampNs * 1e-3;
  if (!data.isValid() ||
      data.pixelFrame->size() != data.pixelFrame->getStride() * data.pixelFrame->getHeight() ||
      data.getPixelFormat() == vrs::PixelFormat::UNDEFINED || data.pixelFrame->size() == 0 ||
      record.captureTimestampNs < 0) {
    stats_.processed++;
    stats_.bad++;
    return;
  }

  const uint64_t exposureDurationUs = record.exposureDuration * 1e6;
  processFrameSkip(frameCenterExposureUs);
  processExposure(exposureDurationUs, frameCenterExposureUs);
  processGain(frameCenterExposureUs, record.gain);
  processTimestamp(frameCenterExposureUs); // stats_

  // Process temperature
  if (!std::isnan(record.temperature)) {
    if (record.temperature < minTemp_ || record.temperature > maxTemp_) {
      cameraStats_.tempOutOfRange++;
    }
  }
}

void Camera::processFrameSkip(const uint64_t frameCenterExposureUs) {
  if (prevTimestampUs_ > 0) {
    const int frameDropDurationUs = frameCenterExposureUs - prevTimestampUs_;
    if (frameDropDurationUs > cameraStats_.longestFrameDropUs) {
      cameraStats_.longestFrameDropUs = frameDropDurationUs;
    }
    if (frameDropDurationUs > maxFrameDropUs_) {
      XR_LOGE(
          "{}: Frame drop of {}us before ts={}us, exceeds --max-frame-drop-us={}us",
          streamId_.getName(),
          frameDropDurationUs,
          frameCenterExposureUs,
          maxFrameDropUs_);
    }
  }
}

} // namespace projectaria::tools::vrs_check
