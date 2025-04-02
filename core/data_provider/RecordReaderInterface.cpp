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

#include <data_provider/ErrorHandler.h>
#include <data_provider/RecordReaderInterface.h>
#include <sstream>

#define DEFAULT_LOG_CHANNEL "RecordReaderInterface"
#include <logging/Log.h>

#include <nlohmann/json.hpp>

namespace projectaria::tools::data_provider {
RecordReaderInterface::RecordReaderInterface(
    std::shared_ptr<vrs::MultiRecordFileReader> reader,
    std::map<vrs::StreamId, std::shared_ptr<ImageSensorPlayer>>& imagePlayers,
    std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>>& motionPlayers,
    std::map<vrs::StreamId, std::shared_ptr<GpsPlayer>>& gpsPlayers,
    std::map<vrs::StreamId, std::shared_ptr<WifiBeaconPlayer>>& wpsPlayers,
    std::map<vrs::StreamId, std::shared_ptr<AudioPlayer>>& audioPlayers,
    std::map<vrs::StreamId, std::shared_ptr<BarometerPlayer>>& barometerPlayers,
    std::map<vrs::StreamId, std::shared_ptr<BluetoothBeaconPlayer>>& bluetoothPlayers,
    std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>>& magnetometerPlayers,
    const std::shared_ptr<TimeSyncMapper>& timeSyncMapper)
    : reader_(reader),
      imagePlayers_(imagePlayers),
      motionPlayers_(motionPlayers),
      gpsPlayers_(gpsPlayers),
      wpsPlayers_(wpsPlayers),
      audioPlayers_(audioPlayers),
      barometerPlayers_(barometerPlayers),
      bluetoothPlayers_(bluetoothPlayers),
      magnetometerPlayers_(magnetometerPlayers),
      timeSyncMapper_(timeSyncMapper),
      readerMutex_(std::make_unique<std::mutex>()) {
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

  for (const auto& [streamId, _] : bluetoothPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Bluetooth);
  }

  for (const auto& [streamId, _] : magnetometerPlayers_) {
    streamIds_.insert(streamId);
    streamIdToSensorDataType_.emplace(streamId, SensorDataType::Magnetometer);
  }

  for (const auto& streamId : streamIds_) {
    streamIdToPlayerMutex_.emplace(streamId, std::make_unique<std::mutex>());
    streamIdToCondition_.emplace(streamId, std::make_unique<std::condition_variable>());
    streamIdToLastReadRecord_.emplace(streamId, nullptr);
  }
  fileTags_ = reader_->getTags();
  vrsMetadata_ = getMetadata();
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

  tempKey = "recording_profile";
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
  }

  tempKey = "device_id";
  if (metadataJson.contains(tempKey)) {
    metadata.deviceId = metadataJson[tempKey];
  }

  tempKey = "start_time";
  if (metadataJson.contains(tempKey)) {
    metadata.startTimeEpochSec = metadataJson[tempKey].get<int64_t>();
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
      if (dataRecords.size() == 0) {
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

  if (index < 0 || index >= reader_->getRecordCount(streamId, vrs::Record::Type::DATA)) {
    return nullptr;
  }
  const vrs::IndexRecord::RecordInfo* recordInfo =
      reader_->getRecord(streamId, vrs::Record::Type::DATA, static_cast<uint32_t>(index));
  checkAndThrow(
      recordInfo, fmt::format("getRecord failed for {}, index {}", streamId.getName(), index));
  const int errorCode = reader_->readRecord(*recordInfo);
  streamIdToLastReadRecord_[streamId] = recordInfo;
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
      return SensorData(streamId, std::move(data), sensorDataType, recordTimeNs, timeSyncData);
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
  return {audioData, audioRecord};
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

  // Aria magnetometer sensor readout are actually in uT rather than Tesla (as the vrs layout
  // specifies). Not changing the data layout to conform to vrs convention
  magnetometerData.magTesla[0] = magnetometerData.magTesla[0] * 1e-6;
  magnetometerData.magTesla[1] = magnetometerData.magTesla[1] * 1e-6;
  magnetometerData.magTesla[2] = magnetometerData.magTesla[2] * 1e-6;
  return magnetometerData;
}

uint32_t RecordReaderInterface::getRgbIspTuningVersion() const {
  if (fileTags_.empty()) {
    return 0;
  }
  std::string tempKey = "rgb_isp_tuning_version";
  auto it = fileTags_.find(tempKey);
  if (it != fileTags_.end()) {
    auto metadataJson = nlohmann::json::parse(it->second);
    if (metadataJson.contains(tempKey)) {
      return static_cast<uint32_t>(metadataJson[tempKey]);
    }
  }
  return 0;
}
} // namespace projectaria::tools::data_provider
