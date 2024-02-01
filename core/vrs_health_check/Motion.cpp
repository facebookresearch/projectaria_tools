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

#include "Motion.h"

#include <iostream>

#include <Eigen/Eigen>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:Motion"
#include <logging/Log.h>

#include "Logging.h"

namespace projectaria::tools::vrs_check {

Motion::Motion(
    vrs::StreamId streamId,
    float minScore,
    float maxImuSkipUs,
    float physicalAccelThreshold,
    float maxNonPhysicalAccel,
    float maxAllowedRotationAccel_radPerS2,
    float defaultPeriodUs)
    : Periodic(streamId, minScore),
      defaultPeriodUs_(defaultPeriodUs),
      maxImuSkipUs_(maxImuSkipUs),
      physicalAccelThreshold_(physicalAccelThreshold),
      maxNonPhysicalAccel_(maxNonPhysicalAccel),
      maxAllowedRotationAccel_radPerS2_(maxAllowedRotationAccel_radPerS2) {}

bool Motion::setup(vrs::RecordFileReader& reader) {
  data_provider::MotionCallback callback =
      [&](const data_provider::MotionData& data, const data_provider::MotionConfigRecord&, bool) {
        this->processData(data);
        return true;
      };
  motionSensorPlayer_ = std::make_unique<data_provider::MotionSensorPlayer>(streamId_);
  if (!motionSensorPlayer_) {
    XR_LOGE("Cannot create playable for {}", streamId_.getName());
    return false;
  }
  motionSensorPlayer_->setCallback(callback);
  reader.setStreamPlayer(streamId_, motionSensorPlayer_.get());

  if (!reader.readFirstConfigurationRecord(streamId_)) {
    XR_LOGE("Stream {} is missing a configuration record", streamId_.getName());
    return false;
  }

  const auto config = motionSensorPlayer_->getConfigRecord();

  periodUs_ = 1.0 / config.nominalRateHz * 1e6;
  if (config.hasMagnetometer && !config.hasAccelerometer && !config.hasGyroscope) {
    // Don't check max skips for magnetometer since it's not relevant
    checkMaxImuSkip_ = false;
  }
  motionConfig_.hasAccel = config.hasAccelerometer;
  motionConfig_.hasGyro = config.hasGyroscope;
  motionConfig_.hasMag = config.hasMagnetometer;

  setMaxDeviationFromPeriodUs();
  preprocessStream(reader);

  return true;
}

MotionStats Motion::getMotionStats() {
  std::lock_guard lock{mutex_};
  return motionStats_;
}

nlohmann::json Motion::statsToJson() {
  std::unique_lock lock{mutex_};
  const MotionStats stats = motionStats_;
  lock.unlock();
  nlohmann::json jsonStats = Periodic::statsToJson();
  jsonStats["repeat_acceleration"] = (stats.repeatAccel + 2) / 3;
  jsonStats["repeat_gyroscope"] = (stats.repeatGyro + 2) / 3; // normalize by rounding up.
  jsonStats["repeat_magnitude"] = stats.repeatMag; // normalize by rounding up.
  jsonStats["zero_acceleration"] = (stats.zeroAccel + 2) / 3; // normalize by rounding up.
  jsonStats["zero_gyroscope"] = (stats.zeroGyro + 2) / 3; // normalize by rounding up.
  jsonStats["zero_magnitude"] = stats.zeroMag;
  jsonStats["longest_continuous_repeat_acceleration"] = stats.longestContRepeatAccel;
  jsonStats["longest_continuous_repeat_gyroscope"] = stats.longestContRepeatGyro;
  jsonStats["non_physical_accel"] = stats.nonPhysicalAccel;
  jsonStats["non_physical_rotAccel"] = stats.numNonPhysicalRotationAccel_;
  jsonStats["max_observed_rotAccel_rad_per_s2"] = stats.maxObservedRotationAccel_radPerS2_;
  jsonStats["longest_imu_skip_us"] = stats.longestImuSkipUs;
  return jsonStats;
}

void Motion::logStats() {
  std::unique_lock lock{mutex_};
  std::cout
      << fmt::format(
             "{}: repeatAccel={} repeatGyro={} repeatMag={}, zeroAccel={}, zeroGyro={},"
             " zeroMag={}, nonPhysicalAccel={}, nonPhysicalRotAccel={} (max {}), longestImuSkipUs={}",
             streamId_.getName(),
             motionStats_.repeatAccel,
             motionStats_.repeatGyro,
             motionStats_.repeatMag,
             motionStats_.zeroAccel,
             motionStats_.zeroGyro,
             motionStats_.zeroMag,
             motionStats_.numNonPhysicalRotationAccel_,
             motionStats_.maxObservedRotationAccel_radPerS2_,
             motionStats_.nonPhysicalAccel,
             motionStats_.longestImuSkipUs)
      << std::endl;
  lock.unlock();
  Periodic::logStats();
}

bool Motion::getResult() {
  float score = getScore();
  std::lock_guard lock{mutex_};
  float proportionNonPhysicalAccel =
      static_cast<float>(motionStats_.nonPhysicalAccel) / stats_.total;
  if (score < minScore_) {
    XR_LOGE("{}: Score {}% is less than minimum {}%", streamId_.getName(), score, minScore_);
    return false;
  } else if (checkMaxImuSkip_ && motionStats_.longestImuSkipUs > maxImuSkipUs_) {
    XR_LOGE(
        "{}: Longest IMU skip {} us is over max allowed {} us",
        streamId_.getName(),
        motionStats_.longestImuSkipUs,
        maxImuSkipUs_);
    return false;
  } else if (proportionNonPhysicalAccel > maxNonPhysicalAccel_) {
    XR_LOGE(
        "{}: Proportion of non-physical IMU acceleration measurements {} exceeds max allowed {} ms",
        streamId_.getName(),
        proportionNonPhysicalAccel,
        maxNonPhysicalAccel_);
    return false;
  }
  return true;
}

namespace {
bool checkEqualVector3(const std::vector<float>& first, const std::vector<float>& second) {
  return (first.size() == 3 && second.size() == 3) &&
      std::equal(first.begin(), first.end(), second.begin(), second.end());
}
} // namespace

void Motion::processData(const data_provider::MotionData& data) {
  std::lock_guard lock{mutex_};
  bool currentSampleIsBad = false;
  // Skip if just doing preprocessing
  if (preprocess_) {
    return;
  }
  // Check for bad data
  if (data.accelValid != motionConfig_.hasAccel || data.gyroValid != motionConfig_.hasGyro ||
      data.magValid != motionConfig_.hasMag) {
    currentSampleIsBad = true;
    stats_.bad++;
  }
  // Check for suspicious accel data
  if (data.accelValid) {
    // zero sample
    const int zeroAccel =
        int(data.accelMSec2[0] == 0) + int(data.accelMSec2[1] == 0) + int(data.accelMSec2[2] == 0);
    motionStats_.zeroAccel += zeroAccel;

    // repeated sample
    const int repeatAccel = int(data.accelMSec2[0] == prevAccel_[0]) +
        int(data.accelMSec2[1] == prevAccel_[1]) + int(data.accelMSec2[2] == prevAccel_[2]);
    motionStats_.repeatAccel += repeatAccel;
    contRepeatAccel_ = (repeatAccel > 0) ? (contRepeatAccel_ + 1) : 0;
    motionStats_.longestContRepeatAccel =
        std::max(motionStats_.longestContRepeatAccel, contRepeatAccel_);

    // non physical accel
    if (std::abs(data.accelMSec2[0] - prevAccel_[0]) > physicalAccelThreshold_ ||
        std::abs(data.accelMSec2[1] - prevAccel_[1]) > physicalAccelThreshold_ ||
        std::abs(data.accelMSec2[2] - prevAccel_[2]) > physicalAccelThreshold_) {
      motionStats_.nonPhysicalAccel++;
    }
    prevAccel_ = data.accelMSec2;
  }
  if (data.gyroValid) {
    // zero sample
    const int zeroGyro =
        int(data.gyroRadSec[0] == 0) + int(data.gyroRadSec[1] == 0) + int(data.gyroRadSec[2] == 0);
    motionStats_.zeroGyro += zeroGyro;

    // repeated sample
    const int repeatGyro = int(data.gyroRadSec[0] == prevGyro_[0]) +
        int(data.gyroRadSec[1] == prevGyro_[1]) + int(data.gyroRadSec[2] == prevGyro_[2]);
    motionStats_.repeatGyro += repeatGyro;
    contRepeatGyro_ = (repeatGyro > 0) ? (contRepeatGyro_ + 1) : 0;
    motionStats_.longestContRepeatGyro =
        std::max(motionStats_.longestContRepeatGyro, contRepeatGyro_);

    if (prevGyroTimeStampNs && data.captureTimestampNs > *prevGyroTimeStampNs) {
      Eigen::Vector3f ra(
          data.gyroRadSec[0] - prevGyro_[0],
          data.gyroRadSec[1] - prevGyro_[1],
          data.gyroRadSec[2] - prevGyro_[2]);
      ra = 1e9f * ra / (float)(data.captureTimestampNs - *prevGyroTimeStampNs);

      if (ra.matrix().cwiseAbs().maxCoeff() > maxAllowedRotationAccel_radPerS2_) {
        motionStats_.numNonPhysicalRotationAccel_++;
      }

      motionStats_.maxObservedRotationAccel_radPerS2_ = std::max(
          motionStats_.maxObservedRotationAccel_radPerS2_, ra.matrix().cwiseAbs().maxCoeff());
    }

    prevGyro_ = data.gyroRadSec;
    prevGyroTimeStampNs = data.captureTimestampNs;
  }
  if (data.magValid) {
    if (checkEqualVector3(data.magTesla, {0.0, 0.0, 0.0})) {
      motionStats_.zeroMag++;
    } else if (checkEqualVector3(data.magTesla, prevMag_)) {
      motionStats_.repeatMag++;
    } else {
      prevMag_ = data.magTesla;
    }
  }

  if (!currentSampleIsBad) {
    if (lastValidTimestampNs_) {
      const uint64_t imuSkipUs = (data.captureTimestampNs - *lastValidTimestampNs_) / 1e3;

      motionStats_.longestImuSkipUs = std::max(imuSkipUs, motionStats_.longestImuSkipUs);

      // note: [maxImuSkipUs_] only affects logging, and is only used for streams that contain
      // Accel/Gyro data. the largest skip period is kept track of regardless.
      if (imuSkipUs > maxImuSkipUs_ && (data.accelValid || data.gyroValid)) {
        XR_LOGE(
            "{}: IMU skip {}us at ts={}us is over max allowed {}us",
            streamId_.getName(),
            imuSkipUs,
            data.captureTimestampNs / 1e3,
            maxImuSkipUs_);
      }
    }
    lastValidTimestampNs_ = data.captureTimestampNs;
  }

  processTimestamp(data.captureTimestampNs / 1e3);
}

} // namespace projectaria::tools::vrs_check
