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

#include "VrsHealthCheck.h"

#include <fmt/format.h>
#include <format/Format.h>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility> // for move
#include <vector>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:VrsHealthCheck"
#include <logging/Log.h>

#include <vrs/ErrorCode.h>
#include <vrs/StreamId.h>
#include <filesystem>

#include "nlohmann/json.hpp"

#include "Audio.h"
#include "Barometer.h"
#include "Bluetooth.h"
#include "Camera.h"
#include "Gps.h"
#include "Motion.h"
#include "TimeDomainMapping.h"
#include "Utils.h"
#include "Wifi.h"

namespace fs = std::filesystem;
using namespace std::literals::chrono_literals;

namespace projectaria::tools::vrs_check {

VrsHealthCheck::VrsHealthCheck(Settings settings) : settings_{settings} {}

bool VrsHealthCheck::setup(const std::string& path) {
  bool result = setupFiles(path);
  /*
  Although it would seem more natural to setup SensorMisalignmentStats
  prior to setting up path, the SensorMisalignmentStats
  constructor requires the period of each sensor to be known hence why
  this is run here after the playables have been setup.
  */
  setupSensorHealthStatsMap();
  return result;
}

bool VrsHealthCheck::setupFiles(const std::string& path) {
  // Load all VRS files in the path
  std::vector<std::string> filePaths;
  if (fs::is_directory(path)) {
    for (const auto& p : fs::directory_iterator(path)) {
      if (p.path().extension() == ".vrs") {
        filePaths.emplace_back(p.path().filename().string());
        reader_.emplace_back(std::make_unique<vrs::RecordFileReader>());
        int rc = reader_.back()->openFile(p.path().string());
        if (rc) {
          XR_LOGE("Failed to open file {}, error = {}", p.path().string(), rc);
          return false;
        }
      }
    }
  } else if (fs::path(path).extension() == ".vrs") {
    filePaths.emplace_back(fs::path(path).filename().string());
    reader_.emplace_back(std::make_unique<vrs::RecordFileReader>());
    int rc = reader_.back()->openFile(path);
    if (rc) {
      XR_LOGE("Failed to open file {}, error = {}", path, rc);
      return false;
    }
  }
  if (reader_.empty()) {
    XR_LOGE("No VRS file found");
    return false;
  }
  return setupPlayables(filePaths);
}

bool VrsHealthCheck::setupPlayables(const std::vector<std::string>& filePaths) {
  // Add all playables
  for (const auto& reader : reader_) {
    const std::set<vrs::StreamId>& recordables = reader->getStreams();
    for (const auto& recordable : recordables) {
      std::unique_ptr<Stream> stream = nullptr;
      switch (recordable.getTypeId()) {
        case vrs::RecordableTypeId::StereoAudioRecordableClass:
          stream = std::make_unique<Audio>(recordable, settings_.minAudioScore);
          break;
        case vrs::RecordableTypeId::BarometerRecordableClass:
          stream = std::make_unique<Barometer>(
              recordable, settings_.minBaroScore, settings_.minTemp, settings_.maxTemp);
          break;
        case vrs::RecordableTypeId::BluetoothBeaconRecordableClass:
          stream = std::make_unique<Bluetooth>(recordable);
          break;
        case vrs::RecordableTypeId::GpsRecordableClass:
          stream = std::make_unique<Gps>(
              recordable, settings_.defaultGpsRateHz, settings_.minGpsAccuracy);
          break;
        case vrs::RecordableTypeId::EyeCameraRecordableClass:
        case vrs::RecordableTypeId::RgbCameraRecordableClass:
        case vrs::RecordableTypeId::SlamCameraData: {
          const auto& roiCheckSettingItor =
              settings_.cameraCheckSettings.find(recordable.getTypeId());
          if (roiCheckSettingItor != settings_.cameraCheckSettings.end()) {
            // Found roi check settings
            stream = std::make_unique<Camera>(
                recordable,
                settings_.minCameraScore,
                settings_.maxFrameDropUs,
                settings_.minCameraGain,
                settings_.maxCameraGain,
                settings_.minCameraExposureMs,
                settings_.maxCameraExposureMs,
                settings_.minTemp,
                settings_.maxTemp,
                roiCheckSettingItor->second);
          } // end if roiCheckSettingItor
          break;
        }
        case vrs::RecordableTypeId::SlamImuData:
        case vrs::RecordableTypeId::SlamMagnetometerData: {
          stream = std::make_unique<Motion>(
              recordable,
              settings_.minImuScore,
              settings_.maxImuSkipUs,
              settings_.physicalAccelThreshold,
              settings_.maxNonPhysicalAccel,
              settings_.maxAllowedRotationAccel_radPerS2,
              settings_.defaultImuPeriodUs);
          break;
        }
        case vrs::RecordableTypeId::TimeRecordableClass: {
          stream =
              std::make_unique<TimeDomainMapping>(recordable, settings_.minTimeDomainMappingScore);
          break;
        }
        case vrs::RecordableTypeId::WifiBeaconRecordableClass:
          stream = std::make_unique<Wifi>(recordable);
          break;
        default:
          XR_LOGW("Unhandled stream {}", recordable.getName());
          break;
      }
      if (stream) {
        stream->setup(*reader);
        streams_.push_back(std::move(stream));
      }
    }
  }

  return true;
}

void VrsHealthCheck::setupSensorHealthStatsMap() {
  std::map<std::string, std::unique_ptr<SensorHealthStats>> sensorHealthStatsMap;
  for (const auto& stream : streams_) {
    const std::string streamName = stream->getStreamId().getName();
    sensorHealthStatsMap.emplace(
        streamName, std::make_unique<SensorHealthStats>(streamName, stream->getPeriodUs()));
  }
  Periodic::setSensorMisalignmentStats(sensorHealthStatsMap);
}

double VrsHealthCheck::getFirstDataRecordTime() {
  double minTimestamp = std::numeric_limits<double>::infinity();
  for (const auto& reader : reader_) {
    double timestamp = reader->getFirstDataRecordTime();
    if (timestamp < minTimestamp) {
      minTimestamp = timestamp;
    }
  }
  return minTimestamp;
}

double VrsHealthCheck::getLastDataRecordTime() {
  double maxTimestamp = 0.0;
  for (const auto& reader : reader_) {
    for (const auto& streamId : reader->getStreams()) {
      const auto record = reader->getLastRecord(streamId, vrs::Record::Type::DATA);
      if (record != nullptr && record->timestamp > maxTimestamp) {
        maxTimestamp = record->timestamp;
      }
    }
  }
  return maxTimestamp;
}

void VrsHealthCheck::printProgress() {
  for (const auto& stream : streams_) {
    const auto stats = stream->getStats();
    std::string streamName = stream->getStreamId().getName();

    float current = static_cast<float>(stats.processed);
    float total = static_cast<float>(stats.total);

    Utils::printBar(streamName, static_cast<bool>(total) ? current / total : 0.0f);

    if (settings_.progressCallback != nullptr) {
      settings_.progressCallback(streamName, current, total);
    }
  }
}

bool VrsHealthCheck::run() {
  bool result = true;
  Utils::enableColoredText(settings_.isInteractive);
  XR_LOGI("Reading all records!");
  auto readStart = std::chrono::high_resolution_clock::now();
  std::vector<std::future<int>> readFut;
  readFut.reserve(reader_.size());
  for (const auto& reader : reader_) {
    readFut.emplace_back(std::async(&vrs::RecordFileReader::readAllRecords, reader.get()));
  }
  // Print progress information periodically
  bool keepGoing;
  do {
    keepGoing = false;
    if (settings_.isInteractive) {
      printProgress();
      for (int i = 0; i < streams_.size(); i++) {
        std::cout << Utils::kClearStr << Utils::kMoveUpStr;
      }
    }
    for (const auto& fut : readFut) {
      if (fut.wait_for(100ms) == std::future_status::timeout) {
        keepGoing = true;
        break;
      }
    }
  } while (keepGoing);
  if (settings_.isInteractive) {
    printProgress();
  }

  // Gather the results to get any exceptions.
  for (auto& fut : readFut) {
    try {
      (void)fut.get(); // Re-throw any exception.
    } catch (const std::exception& e) {
      XR_LOGE("Exception while reading: {}", e.what());
      for (const auto& stream : streams_) {
        XR_LOGE(
            "{} at processed={}", stream->getStreamId().getName(), stream->getStats().processed);
      }
      result = false;
    }
  }

  auto readEnd = std::chrono::high_resolution_clock::now();
  double vrsTimeSec = getLastDataRecordTime() - getFirstDataRecordTime();
  double readTimeSec =
      std::chrono::duration_cast<std::chrono::microseconds>(readEnd - readStart).count() / 1e6;
  std::cout
      << fmt::format(
             "Processing stats: vrs_length={:.3f} secs, proc_time={:.3f} secs, speedup={:.3f}%",
             vrsTimeSec,
             readTimeSec,
             100 * (vrsTimeSec - readTimeSec) / readTimeSec)
      << std::endl;

  Periodic::getSensorMisalignmentStats()->computeScores();

  cachedMisalignmentStatistics_ =
      Periodic::getSensorMisalignmentStats()->misalignmentStatisticsMap();

  XR_LOGI("Finished reading all records!");
  return result;
}

void VrsHealthCheck::logStats() {
  for (const auto& stream : streams_) {
    stream->logStats();
    stream->logScore();
  }
  for (const auto& sensor : cachedMisalignmentStatistics_) {
    auto sensorName = sensor.first;
    for (const auto& sensorToAlign : sensor.second) {
      auto sensorToAlignName = sensorToAlign.first;
      const auto& sensorMisalignmentScore = sensorToAlign.second.score;
      const std::string misalignmentScoreName =
          fmt::format("Sensor Misalignment ({} - {})", sensorName, sensorToAlignName);
      Utils::logScore(misalignmentScoreName, sensorMisalignmentScore, settings_.minAlignmentScore);
    }
  }
}

void VrsHealthCheck::logStatsJson(const std::string& filepath) {
  nlohmann::json aggregatedJsonResults;
  for (const auto& stream : streams_) {
    nlohmann::json statsJson = stream->statsToJson();
    if (std::isfinite(stream->getScore())) {
      statsJson["score"] = stream->getScore();
    }
    aggregatedJsonResults[stream->getStreamId().getName()] = statsJson;
  }

  for (const auto& sensor : cachedMisalignmentStatistics_) {
    auto sensorName = sensor.first;
    for (const auto& sensorToAlign : sensor.second) {
      auto sensorToAlignName = sensorToAlign.first;

      aggregatedJsonResults[fmt::format("{} - {}", sensorName, sensorToAlignName)]["score"] =
          sensorToAlign.second.score;
      aggregatedJsonResults[fmt::format("{} - {}", sensorName, sensorToAlignName)]
                           ["num_frames_misaligned"] = sensorToAlign.second.misaligned;
      aggregatedJsonResults[fmt::format("{} - {}", sensorName, sensorToAlignName)]
                           ["largest_misalignment_us"] = sensorToAlign.second.max_misalignment_us;
      aggregatedJsonResults[fmt::format("{} - {}", sensorName, sensorToAlignName)]
                           ["num_frames_checked"] = sensorToAlign.second.total;
    }
  }

  if (!filepath.empty()) {
    std::ofstream outputFile(filepath);
    if (!outputFile.is_open()) {
      throw std::runtime_error("Unable to open file for writing: " + filepath);
    }
    outputFile << std::setw(2) << aggregatedJsonResults;
    outputFile.close();
  } else {
    std::cout << std::setw(2) << aggregatedJsonResults;
  }
}

void VrsHealthCheck::logDroppedFrames(const std::string& filepath) {
  std::ofstream csvWriter(filepath);
  if (!csvWriter) {
    XR_LOGW("Unable to open destination file {}", filepath);
    return;
  }
  for (const auto& stream : streams_) {
    Periodic* periodicStream = dynamic_cast<Periodic*>(stream.get());
    if (periodicStream) {
      periodicStream->logDroppedFrames(csvWriter);
    }
  }
  csvWriter.close();
}

bool VrsHealthCheck::getResult() {
  bool result = true;
  for (const auto& stream : streams_) {
    result &= stream->getResult();
  }

  // Check that alignment scores are > minimum tolerance
  for (const auto& sensor : cachedMisalignmentStatistics_) {
    auto sensorName = sensor.first;
    for (const auto& sensorToAlign : sensor.second) {
      auto sensorToAlignName = sensorToAlign.first;
      const auto& sensorMisalignmentScore = sensorToAlign.second.score;
      if (sensorMisalignmentScore < settings_.minAlignmentScore) {
        XR_LOGE(
            "Sensor Misalignment {} - {}: Score {}% is less than minimum {}%",
            sensorName,
            sensorToAlignName,
            sensorMisalignmentScore,
            settings_.minAlignmentScore);
        result = false;
      }
    }
  }
  // Print the result as text:
  const char* beginColor = "";
  const char* endColor = "";
  if (settings_.isInteractive) {
    beginColor = (result ? Utils::kGreenStr : Utils::kRedStr);
    endColor = Utils::kResetStr;
  }
  const char* const resultText = result ? "PASS" : "FAIL";
  std::cout << "VRS validation result: " << beginColor << resultText << endColor << std::endl;
  return result;
}

} // namespace projectaria::tools::vrs_check
