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

#include <data_provider/RecordReaderInterface.h>

#include <sstream>

#include <data_provider/ErrorHandler.h>
#include <nlohmann/json.hpp>

#define DEFAULT_LOG_CHANNEL "RecordReaderInterface"
#include <logging/Log.h>

namespace projectaria::tools::data_provider {

namespace {
// Gen1 and Gen2 have different metadata formats, these helpers would parse the metadata json
void fillMetadataForGen1(
    VrsMetadata& metadata,
    const nlohmann::json& metadataJson,
    const std::shared_ptr<TimeSyncMapper>& timeSyncMapper) {
  std::string tempKey = "recording_profile";
  if (metadataJson.contains(tempKey)) {
    metadata.recordingProfile = metadataJson[tempKey];
  }

  tempKey = "shared_session_id";
  if (metadataJson.contains(tempKey)) {
    metadata.sharedSessionId = metadataJson[tempKey];
  }

  tempKey = "filename";
  if (metadataJson.contains(tempKey)) {
    metadata.filename = metadataJson[tempKey];
  }

  metadata.timeSyncMode = MetadataTimeSyncMode::NotEnabled;
  if (metadataJson.contains("ticsync_mode")) {
    const std::string& ticsyncMode = metadataJson["ticsync_mode"];
    if (ticsyncMode == "client") {
      metadata.timeSyncMode = MetadataTimeSyncMode::TicSyncClient;
    } else if (ticsyncMode == "server") {
      metadata.timeSyncMode = MetadataTimeSyncMode::TicSyncServer;
    }
  } else if (metadataJson.contains("ntp_time_enabled") && metadataJson["ntp_time_enabled"]) {
    metadata.timeSyncMode = MetadataTimeSyncMode::Ntp;
  } else if (metadataJson.contains("timecode_enabled") && metadataJson["timecode_enabled"]) {
    metadata.timeSyncMode = MetadataTimeSyncMode::Timecode;
  } else if (timeSyncMapper->supportsMode(TimeSyncMode::SUBGHZ)) {
    metadata.timeSyncMode = MetadataTimeSyncMode::SubGhz;
  } else if (timeSyncMapper->supportsMode(TimeSyncMode::UTC)) {
    metadata.timeSyncMode = MetadataTimeSyncMode::Utc;
  }

  tempKey = "device_id";
  if (metadataJson.contains(tempKey)) {
    metadata.deviceId = metadataJson[tempKey];
  }

  tempKey = "start_time";
  if (metadataJson.contains(tempKey)) {
    metadata.startTimeEpochSec = metadataJson[tempKey].get<int64_t>();
  }
}

void fillMetadataForGen2(VrsMetadata& metadata, const nlohmann::json& metadataJson) {
  std::string tempKey_1 = "recording";
  std::string tempKey_2 = "profile";
  std::string tempKey_3 = "name";
  if (metadataJson.contains(tempKey_1) && metadataJson[tempKey_1].contains(tempKey_2) &&
      metadataJson[tempKey_1][tempKey_2].contains(tempKey_3)) {
    metadata.recordingProfile = metadataJson[tempKey_1][tempKey_2][tempKey_3];
  }

  tempKey_1 = "device";
  tempKey_2 = "uuid";
  if (metadataJson.contains(tempKey_1) && metadataJson[tempKey_1].contains(tempKey_2)) {
    metadata.deviceId = metadataJson[tempKey_1][tempKey_2];
  }

  tempKey_1 = "recording";
  tempKey_2 = "start_time";
  if (metadataJson.contains(tempKey_1) && metadataJson[tempKey_1].contains(tempKey_2)) {
    metadata.startTimeEpochSec = metadataJson[tempKey_1][tempKey_2];
  }

  // TODO: parse this when OS team adds this to the metadata
  metadata.timeSyncMode = MetadataTimeSyncMode::NotEnabled;
}
} // namespace

RecordReaderInterface::RecordReaderInterface(
    std::shared_ptr<vrs::MultiRecordFileReader> reader,
    std::map<vrs::StreamId, std::shared_ptr<ImageSensorPlayer>>& imagePlayers,
    std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>>& motionPlayers,
    std::map<vrs::StreamId, std::shared_ptr<GpsPlayer>>& gpsPlayers,
    std::map<vrs::StreamId, std::shared_ptr<WifiBeaconPlayer>>& wpsPlayers,
    std::map<vrs::StreamId, std::shared_ptr<AudioPlayer>>& audioPlayers,
    std::map<vrs::StreamId, std::shared_ptr<BarometerPlayer>>& barometerPlayers,
    std::map<vrs::StreamId, std::shared_ptr<BatteryStatusPlayer>>& batteryStatusPlayers,
    std::map<vrs::StreamId, std::shared_ptr<BluetoothBeaconPlayer>>& bluetoothPlayers,
    std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>>& magnetometerPlayers,
    std::map<vrs::StreamId, std::shared_ptr<PpgPlayer>>& ppgPlayers,
    std::map<vrs::StreamId, std::shared_ptr<AlsPlayer>>& alsPlayers,
    std::map<vrs::StreamId, std::shared_ptr<TemperaturePlayer>>& temperaturePlayers,
    std::map<vrs::StreamId, std::shared_ptr<VioPlayer>>& vioPlayers,
    std::map<vrs::StreamId, std::shared_ptr<VioHighFrequencyPlayer>>& vioHighFreqPlayers,
    std::map<vrs::StreamId, std::shared_ptr<EyeGazePlayer>>& eyegazePlayers,
    std::map<vrs::StreamId, std::shared_ptr<HandPosePlayer>>& handPosePlayers,
    const std::shared_ptr<TimeSyncMapper>& timeSyncMapper)
    : reader_(reader),
      imagePlayers_(imagePlayers),
      motionPlayers_(motionPlayers),
      gpsPlayers_(gpsPlayers),
      wpsPlayers_(wpsPlayers),
      audioPlayers_(audioPlayers),
      barometerPlayers_(barometerPlayers),
      batteryStatusPlayers_(batteryStatusPlayers),
      bluetoothPlayers_(bluetoothPlayers),
      magnetometerPlayers_(magnetometerPlayers),
      ppgPlayers_(ppgPlayers),
      alsPlayers_(alsPlayers),
      temperaturePlayers_(temperaturePlayers),
      vioPlayers_(vioPlayers),
      vioHighFreqPlayers_(vioHighFreqPlayers),
      eyegazePlayers_(eyegazePlayers),
      handPosePlayers_(handPosePlayers),
      timeSyncMapper_(timeSyncMapper),
      readerMutex_(std::make_unique<std::mutex>()) {
  // Determine device version
  const std::string deviceTypeString = reader_->getTag("device_type");
  deviceVersion_ = calibration::fromDeviceClassName(deviceTypeString);

  for (const auto& [streamId, _] : imagePlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Image);
  }

  for (const auto& [streamId, _] : motionPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Imu);
  }

  for (const auto& [streamId, _] : gpsPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Gps);
  }

  for (const auto& [streamId, _] : wpsPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Wps);
  }

  for (const auto& [streamId, _] : audioPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Audio);
  }

  for (const auto& [streamId, _] : barometerPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Barometer);
  }

  for (const auto& [streamId, _] : batteryStatusPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::BatteryStatus);
  }

  for (const auto& [streamId, _] : bluetoothPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Bluetooth);
  }

  for (const auto& [streamId, _] : magnetometerPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Magnetometer);
  }

  for (const auto& [streamId, _] : ppgPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Ppg);
  }

  for (const auto& [streamId, _] : alsPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Als);
  }

  for (const auto& [streamId, _] : temperaturePlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Temperature);
  }

  for (const auto& [streamId, _] : vioPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Vio);
  }

  for (const auto& [streamId, _] : vioHighFreqPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::VioHighFreq);
  }

  for (const auto& [streamId, _] : eyegazePlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::EyeGaze);
  }

  for (const auto& [streamId, _] : handPosePlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::HandPose);
  }

  for (const auto& streamId : streamIds_) {
    streamIdToPlayerMutex_.emplace(streamId, std::make_unique<std::mutex>());
    streamIdToCondition_.emplace(streamId, std::make_unique<std::condition_variable>());
    streamIdToLastReadRecord_.emplace(streamId, nullptr);
  }
  fileTags_ = reader_->getTags();
  vrsMetadata_ = getMetadata();

  // Initialize Opus detection cache
  initializeOpusDetection();
}

std::set<vrs::StreamId> RecordReaderInterface::getStreamIds() const {
  return streamIds_;
}

std::map<std::string, std::string> RecordReaderInterface::getFileTags() const {
  return fileTags_;
}

std::optional<VrsMetadata> RecordReaderInterface::getMetadata() const {
  // VRS file tags are only available if a VRS file was loaded.
  if (fileTags_.empty()) {
    return {};
  }

  VrsMetadata metadata;

  // Some of our metadata comes from other tags; copy them first.
  std::string tempKey = "device_serial";
  if (fileTags_.find(tempKey) != fileTags_.end()) {
    metadata.deviceSerial = fileTags_.at(tempKey);
  }

  // No point in going on if there is no metadata tag. If
  // there is a metadata tag, parse the metadata json.
  tempKey = "metadata";
  if (fileTags_.find(tempKey) == fileTags_.end()) {
    XR_LOGE("Tag 'metadata' was not found in the VRS file tags");
    return metadata;
  }
  auto metadataJson = nlohmann::json::parse(fileTags_.at(tempKey));

  // Parse metadata keys based on device version
  switch (deviceVersion_) {
    case calibration::DeviceVersion::Gen1:
      fillMetadataForGen1(metadata, metadataJson, timeSyncMapper_);
      break;
    case calibration::DeviceVersion::Gen2:
      fillMetadataForGen2(metadata, metadataJson);
      break;
    default:
      throw std::runtime_error(
          fmt::format(
              "Unsupported device version for loading metadata: {}", getName(deviceVersion_)));
  }

  return metadata;
}

std::optional<MetadataTimeSyncMode> RecordReaderInterface::getTimeSyncMode() const {
  if (vrsMetadata_.has_value()) {
    return vrsMetadata_.value().timeSyncMode;
  }
  return {};
}

SensorDataType RecordReaderInterface::getSensorDataType(const vrs::StreamId& streamId) const {
  auto it = streamIdToSensorDataType_.find(streamId);
  if (it == streamIdToSensorDataType_.end()) {
    return SensorDataType::NotValid;
  } else {
    return it->second;
  }
}

size_t RecordReaderInterface::getNumData(const vrs::StreamId& streamId) const {
  return reader_->getRecordCount(streamId, vrs::Record::Type::DATA);
}

std::map<vrs::StreamId, std::vector<const vrs::IndexRecord::RecordInfo*>>
RecordReaderInterface::getStreamIdToDataRecords() {
  std::map<vrs::StreamId, std::vector<const vrs::IndexRecord::RecordInfo*>> streamIdToDataRecords;

  // filter all records and cache those which are of DATA type
  const uint32_t totalRecords = reader_->getRecordCount();

  for (uint32_t globalIndex = 0; globalIndex < totalRecords; ++globalIndex) {
    const auto& recordInfo = reader_->getRecord(globalIndex);
    vrs::StreamId streamId = recordInfo->streamId;
    if (streamIds_.count(streamId) > 0 && recordInfo->recordType == vrs::Record::Type::DATA) {
      auto& dataRecords = streamIdToDataRecords[streamId];
      if (dataRecords.empty()) {
        dataRecords.reserve(reader_->getRecordCount(streamId, vrs::Record::Type::DATA));
      }
      dataRecords.push_back(recordInfo);
    }
  }
  return streamIdToDataRecords;
}

// according to (streamId, index) pair if records exists
const vrs::IndexRecord::RecordInfo* RecordReaderInterface::readRecordByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  std::lock_guard<std::mutex> lockGuard(*readerMutex_);

  // Check bounds first to prevent invalid pre-roll attempts
  if (index < 0 || index >= reader_->getRecordCount(streamId, vrs::Record::Type::DATA)) {
    return nullptr;
  }

  // Handle Opus audio pre-roll before reading the target record
  if (getSensorDataType(streamId) == SensorDataType::Audio && isOpusAudioStream(streamId) &&
      needsOpusPreroll(streamId, index)) {
    performOpusPreroll(streamId, index);
  }
  const vrs::IndexRecord::RecordInfo* recordInfo =
      reader_->getRecord(streamId, vrs::Record::Type::DATA, static_cast<uint32_t>(index));
  checkAndThrow(
      recordInfo, fmt::format("getRecord failed for {}, index {}", streamId.getName(), index));
  const int errorCode = reader_->readRecord(*recordInfo);
  streamIdToLastReadRecord_[streamId] = recordInfo;

  // Update tracking for audio streams after successful read
  if (getSensorDataType(streamId) == SensorDataType::Audio) {
    audioStreamLastReadIndex_[streamId] = index;

    // Update Opus detection cache after first successful read
    auto audioPlayerIt = audioPlayers_.find(streamId);
    if (audioPlayerIt != audioPlayers_.end()) {
      vrs::AudioFormat detectedFormat = audioPlayerIt->second->getDetectedAudioFormat();
      if (detectedFormat != vrs::AudioFormat::UNDEFINED) {
        updateOpusDetection(streamId, detectedFormat == vrs::AudioFormat::OPUS);
      }
    }
  }

  if (errorCode != 0) {
    XR_LOGE(
        "Fail to read record {} from streamId {} with code {}",
        index,
        streamId.getNumericName(),
        errorCode);
    return nullptr;
  }
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  streamIdToCondition_.at(streamId)->wait(readLock, [] { return true; });
  return recordInfo;
}

void RecordReaderInterface::setReadImageContent(vrs::StreamId streamId, bool readContent) {
  auto it = imagePlayers_.find(streamId);
  if (it != imagePlayers_.end()) {
    it->second->setReadContent(readContent);
  }
}

/* read the last cached sensor data in player */
SensorData RecordReaderInterface::getLastCachedSensorData(const vrs::StreamId& streamId) {
  SensorDataType sensorDataType = getSensorDataType(streamId);
  const int64_t recordTimeNs =
      static_cast<int64_t>(streamIdToLastReadRecord_.at(streamId)->timestamp * 1e9);

  switch (sensorDataType) {
    case SensorDataType::Image: {
      auto data = getLastCachedImageData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs = timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(
            data.second.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return {streamId, data, sensorDataType, recordTimeNs, timeSyncData};
    }
    case SensorDataType::Imu: {
      auto data = getLastCachedImuData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::Audio: {
      auto data = getLastCachedAudioData(streamId);
      if (data.second.captureTimestampsNs.empty()) {
        return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, {});
      }
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs = timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(
            data.second.captureTimestampsNs.back(), mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::Barometer: {
      auto data = getLastCachedBarometerData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::Gps: {
      auto data = getLastCachedGpsData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::Wps: {
      auto data = getLastCachedWpsData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.boardTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::Magnetometer: {
      auto data = getLastCachedMagnetometerData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::Bluetooth: {
      auto data = getLastCachedBluetoothData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.boardTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::Ppg: {
      auto data = getLastCachedPpgData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::Als: {
      auto data = getLastCachedAlsData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return {streamId, data, sensorDataType, recordTimeNs, timeSyncData};
    }
    case SensorDataType::Temperature: {
      auto data = getLastCachedTemperatureData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::BatteryStatus: {
      auto data = getLastCachedBatteryStatusData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::Vio: {
      auto data = getLastCachedVioData(streamId);
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(data.captureTimestampNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::VioHighFreq: {
      auto data = getLastCachedVioHighFreqData(streamId);
      int64_t deviceTimeNs =
          std::chrono::duration_cast<std::chrono::nanoseconds>(data.trackingTimestamp).count();
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(deviceTimeNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::EyeGaze: {
      auto data = getLastCachedEyeGazeData(streamId);
      int64_t deviceTimeNs =
          std::chrono::duration_cast<std::chrono::nanoseconds>(data.trackingTimestamp).count();
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(deviceTimeNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::HandPose: {
      auto data = getLastCachedHandPoseData(streamId);
      int64_t deviceTimeNs =
          std::chrono::duration_cast<std::chrono::nanoseconds>(data.trackingTimestamp).count();
      std::map<TimeSyncMode, int64_t> timeSyncData;
      for (const auto& mode : timeSyncMapper_->getTimeSyncModes()) {
        const int64_t& syncTimeNs =
            timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(deviceTimeNs, mode);
        timeSyncData.emplace(mode, syncTimeNs);
      }
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
    }
    case SensorDataType::NotValid:
    default:
      break;
  }
  return SensorData(streamId, std::monostate{}, SensorDataType::NotValid, -1, {});
}

ImageDataAndRecord RecordReaderInterface::getLastCachedImageData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  const auto& imagePlayer = imagePlayers_.at(streamId);
  auto imageData = imagePlayer->getData();
  auto imageRecord = imagePlayer->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return {imageData, imageRecord};
}

MotionData RecordReaderInterface::getLastCachedImuData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto imuData = motionPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return imuData;
}

GpsData RecordReaderInterface::getLastCachedGpsData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto gpsData = gpsPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return gpsData;
}

WifiBeaconData RecordReaderInterface::getLastCachedWpsData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto wpsData = wpsPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return wpsData;
}

AudioDataAndRecord RecordReaderInterface::getLastCachedAudioData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  const auto& audioPlayer = audioPlayers_[streamId];
  auto audioData = audioPlayer->getData();
  auto audioRecord = audioPlayer->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return {std::move(audioData), audioRecord};
}

BluetoothBeaconData RecordReaderInterface::getLastCachedBluetoothData(
    const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto bluetoothData = bluetoothPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return bluetoothData;
}

BarometerData RecordReaderInterface::getLastCachedBarometerData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto barometerData = barometerPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return barometerData;
}

MotionData RecordReaderInterface::getLastCachedMagnetometerData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto magnetometerData = magnetometerPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();

  // For Aria-Gen1, magnetometer sensor readout are actually in uT rather than Tesla (as the vrs
  // layout specifies). Not changing the data layout to conform to vrs convention
  // For Aria-Gen2, magnetometer sensor readout are in Tesla.
  if (deviceVersion_ == calibration::DeviceVersion::Gen1) {
    magnetometerData.magTesla[0] = magnetometerData.magTesla[0] * 1e-6;
    magnetometerData.magTesla[1] = magnetometerData.magTesla[1] * 1e-6;
    magnetometerData.magTesla[2] = magnetometerData.magTesla[2] * 1e-6;
  }
  return magnetometerData;
}

PpgData RecordReaderInterface::getLastCachedPpgData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto ppgData = ppgPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return ppgData;
}

AlsData RecordReaderInterface::getLastCachedAlsData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto alsData = alsPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return alsData;
}

TemperatureData RecordReaderInterface::getLastCachedTemperatureData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto temperatureData = temperaturePlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return temperatureData;
}

BatteryStatusData RecordReaderInterface::getLastCachedBatteryStatusData(
    const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto batteryStatusData = batteryStatusPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return batteryStatusData;
}

uint32_t RecordReaderInterface::getRgbIspTuningVersion() const {
  if (fileTags_.empty()) {
    return 0;
  }
  if (fileTags_.find("metadata") == fileTags_.end()) {
    return 0;
  }
  auto metadataJson = nlohmann::json::parse(fileTags_.at("metadata"));
  if (metadataJson.contains("rgb_isp_tuning_version")) {
    return static_cast<uint32_t>(metadataJson["rgb_isp_tuning_version"]);
  }
  return 0;
}

FrontendOutput RecordReaderInterface::getLastCachedVioData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto vioData = vioPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return vioData;
}

OnDeviceVioHighFreqData RecordReaderInterface::getLastCachedVioHighFreqData(
    const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  auto vioHighFreqData = vioHighFreqPlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return vioHighFreqData;
}

OnDeviceEyeGazeData RecordReaderInterface::getLastCachedEyeGazeData(const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  // TODO: Decide if we want to return OSS type data
  auto eyeGazeData = eyegazePlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return eyeGazeData;
}

OnDeviceHandPoseData RecordReaderInterface::getLastCachedHandPoseData(
    const vrs::StreamId& streamId) {
  std::unique_lock<std::mutex> readLock(*(streamIdToPlayerMutex_.at(streamId)));
  // TODO: Decide if we want to return OSS type data
  auto handPoseData = handPosePlayers_[streamId]->getDataRecord();
  streamIdToCondition_.at(streamId)->notify_one();
  return handPoseData;
}

bool RecordReaderInterface::needsOpusPreroll(const vrs::StreamId& streamId, int targetIndex) {
  auto lastIndexIt = audioStreamLastReadIndex_.find(streamId);

  if (lastIndexIt == audioStreamLastReadIndex_.end()) {
    // First read for this stream - need pre-roll if not starting from index 0
    return targetIndex > 0;
  }

  int lastIndex = lastIndexIt->second;
  // Need pre-roll if not sequential (not lastIndex + 1) and not same index
  return (targetIndex != lastIndex + 1 && targetIndex != lastIndex);
}

void RecordReaderInterface::performOpusPreroll(const vrs::StreamId& streamId, int targetIndex) {
  // Reset Opus decoder first
  resetOpusDecoder(streamId);

  // Read previous frames for pre-roll (but don't update lastReadIndex yet)
  int prerollStart = std::max(0, targetIndex - kAudioDecodingPrerollLength);
  for (int prerollIndex = prerollStart; prerollIndex < targetIndex; ++prerollIndex) {
    const vrs::IndexRecord::RecordInfo* prerollRecord =
        reader_->getRecord(streamId, vrs::Record::Type::DATA, static_cast<uint32_t>(prerollIndex));
    if (prerollRecord) {
      reader_->readRecord(*prerollRecord);
      // Audio data is processed by the player but we don't cache or return it
    }
  }
}

bool RecordReaderInterface::isOpusAudioStream(const vrs::StreamId& streamId) {
  // Use cached Opus detection result
  auto it = audioStreamIsOpus_.find(streamId);
  return it != audioStreamIsOpus_.end() && it->second;
}

void RecordReaderInterface::initializeOpusDetection() {
  // Initialize all audio streams as undetermined (will be detected lazily on first read)
  for (const auto& [streamId, audioPlayer] : audioPlayers_) {
    // Initialize as undetermined - will be detected during first actual read
    // when the AudioPlayer's onAudioRead() method can access the content block format
    audioStreamIsOpus_[streamId] = false;
  }
}

void RecordReaderInterface::updateOpusDetection(const vrs::StreamId& streamId, bool isOpus) {
  audioStreamIsOpus_[streamId] = isOpus;
}

void RecordReaderInterface::resetOpusDecoder(const vrs::StreamId& streamId) {
  if (audioPlayers_.find(streamId) != audioPlayers_.end()) {
    audioPlayers_[streamId]->resetOpusDecoder();
  } else {
    fmt::print(
        "Warning: streamId {} not found in audioPlayers_, Opus decoder cannot be reset\n",
        streamId.getNumericName());
  }
}

} // namespace projectaria::tools::data_provider
