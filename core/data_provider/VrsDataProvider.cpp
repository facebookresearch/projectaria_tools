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
#include <data_provider/SensorDataSequence.h>
#include <data_provider/VrsDataProvider.h>

#define DEFAULT_LOG_CHANNEL "VrsDataProvider"
#include <logging/Log.h>

#include <limits>

namespace projectaria::tools::data_provider {
VrsDataProvider::VrsDataProvider(
    const std::shared_ptr<RecordReaderInterface>& interface,
    const std::shared_ptr<StreamIdConfigurationMapper>& configMap,
    const std::shared_ptr<TimeSyncMapper>& timeSyncMapper,
    const std::shared_ptr<StreamIdLabelMapper>& streamIdLabelMapper,
    const std::optional<calibration::DeviceCalibration>& maybeDeviceCalib)
    : interface_(interface),
      configMap_(configMap),
      timeQuery_(std::make_shared<TimestampIndexMapper>(interface_)),
      timeSyncMapper_(timeSyncMapper),
      streamIdLabelMapper_(streamIdLabelMapper),
      maybeDeviceCalib_(maybeDeviceCalib) {}

SensorDataType VrsDataProvider::getSensorDataType(const vrs::StreamId& streamId) const {
  return interface_->getSensorDataType(streamId);
}

std::optional<std::string> VrsDataProvider::getLabelFromStreamId(
    const vrs::StreamId& streamId) const {
  return streamIdLabelMapper_->getLabelFromStreamId(streamId);
}

std::optional<vrs::StreamId> VrsDataProvider::getStreamIdFromLabel(const std::string& label) const {
  return streamIdLabelMapper_->getStreamIdFromLabel(label);
}

/*******************************
 get configurations
 *******************************/
SensorConfiguration VrsDataProvider::getConfiguration(const vrs::StreamId& streamId) const {
  SensorDataType sensorDataType = interface_->getSensorDataType(streamId);
  switch (sensorDataType) {
    case SensorDataType::Image:
      return SensorConfiguration(getImageConfiguration(streamId), sensorDataType);
    case SensorDataType::Imu:
      return SensorConfiguration(getImuConfiguration(streamId), sensorDataType);
    case SensorDataType::Audio:
      return SensorConfiguration(getAudioConfiguration(streamId), sensorDataType);
    case SensorDataType::Barometer:
      return SensorConfiguration(getBarometerConfiguration(streamId), sensorDataType);
    case SensorDataType::Gps:
      return SensorConfiguration(getGpsConfiguration(streamId), sensorDataType);
    case SensorDataType::Wps:
      return SensorConfiguration(getWpsConfiguration(streamId), sensorDataType);
    case SensorDataType::Magnetometer:
      return SensorConfiguration(getMagnetometerConfiguration(streamId), sensorDataType);
    case SensorDataType::Bluetooth:
      return SensorConfiguration(getBluetoothConfiguration(streamId), sensorDataType);
    case SensorDataType::NotValid:
    default:
      break;
  }
  return SensorConfiguration(std::monostate{}, SensorDataType::NotValid);
}

double VrsDataProvider::getNominalRateHz(const vrs::StreamId& streamId) const {
  return getConfiguration(streamId).getNominalRateHz();
}

ImageConfigRecord VrsDataProvider::getImageConfiguration(const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Image);
  return configMap_->getImageConfiguration(streamId);
}

MotionConfigRecord VrsDataProvider::getImuConfiguration(const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Imu);
  return configMap_->getImuConfiguration(streamId);
}

GpsConfigRecord VrsDataProvider::getGpsConfiguration(const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Gps);
  return configMap_->getGpsConfiguration(streamId);
}

WifiBeaconConfigRecord VrsDataProvider::getWpsConfiguration(const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Wps);
  return configMap_->getWpsConfiguration(streamId);
}

AudioConfig VrsDataProvider::getAudioConfiguration(const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Audio);
  return configMap_->getAudioConfiguration(streamId);
}

BarometerConfigRecord VrsDataProvider::getBarometerConfiguration(
    const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Barometer);
  return configMap_->getBarometerConfiguration(streamId);
}

BluetoothBeaconConfigRecord VrsDataProvider::getBluetoothConfiguration(
    const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Bluetooth);
  return configMap_->getBluetoothConfiguration(streamId);
}

MotionConfigRecord VrsDataProvider::getMagnetometerConfiguration(
    const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Magnetometer);
  return configMap_->getMagnetometerConfiguration(streamId);
}

/*******************************
 random access interfaces
 *******************************/
/* get dimension of data */
const std::set<vrs::StreamId> VrsDataProvider::getAllStreams() const {
  return interface_->getStreamIds();
}

std::map<std::string, std::string> VrsDataProvider::getFileTags() const {
  return interface_->getFileTags();
}

size_t VrsDataProvider::getNumData(const vrs::StreamId& streamId) const {
  return interface_->getNumData(streamId);
}

DeliverQueuedOptions VrsDataProvider::getDefaultDeliverQueuedOptions() const {
  std::map<vrs::StreamId, size_t> streamIdToSubsampleRate;
  for (const auto& streamId : interface_->getStreamIds()) {
    streamIdToSubsampleRate.emplace(streamId, 1);
  }
  return DeliverQueuedOptions(0, 0, streamIdToSubsampleRate);
}

std::optional<calibration::DeviceCalibration> VrsDataProvider::getDeviceCalibration() const {
  return maybeDeviceCalib_;
}

std::optional<calibration::SensorCalibration> VrsDataProvider::getSensorCalibration(
    const vrs::StreamId& streamId) const {
  if (!maybeDeviceCalib_) {
    return {};
  } else {
    auto maybeLabel = getLabelFromStreamId(streamId);
    if (!maybeLabel) {
      return {};
    } else {
      return maybeDeviceCalib_->getSensorCalib(*maybeLabel);
    }
  }
}

/* get data from index */
SensorData VrsDataProvider::getSensorDataByIndex(const vrs::StreamId& streamId, const int index) {
  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedSensorData(streamId);
  } else {
    return SensorData(streamId, std::monostate{}, SensorDataType::NotValid, -1, {});
  }
}

/* get data sequentially based on sensor data device time */
SensorDataSequence VrsDataProvider::deliverQueuedSensorData() {
  auto options = getDefaultDeliverQueuedOptions();
  return deliverQueuedSensorData(options);
}

SensorDataSequence VrsDataProvider::deliverQueuedSensorData(DeliverQueuedOptions options) {
  return SensorDataSequence(this, options);
}

ImageDataAndRecord VrsDataProvider::getImageDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Image);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedImageData(streamId);
  } else {
    return {};
  }
}

MotionData VrsDataProvider::getImuDataByIndex(const vrs::StreamId& streamId, const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Imu);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedImuData(streamId);
  } else {
    return {};
  }
}

GpsData VrsDataProvider::getGpsDataByIndex(const vrs::StreamId& streamId, const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Gps);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedGpsData(streamId);
  } else {
    return {};
  }
}

WifiBeaconData VrsDataProvider::getWpsDataByIndex(const vrs::StreamId& streamId, const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Wps);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedWpsData(streamId);
  } else {
    return {};
  }
}

AudioDataAndRecord VrsDataProvider::getAudioDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Audio);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedAudioData(streamId);
  } else {
    return {};
  }
}

BarometerData VrsDataProvider::getBarometerDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Barometer);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedBarometerData(streamId);
  } else {
    return {};
  }
}

BluetoothBeaconData VrsDataProvider::getBluetoothDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Bluetooth);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedBluetoothData(streamId);
  } else {
    return {};
  }
}

MotionData VrsDataProvider::getMagnetometerDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Magnetometer);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedMagnetometerData(streamId);
  } else {
    return {};
  }
}

/* get data before time stamps */
SensorData VrsDataProvider::getSensorDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getSensorDataByIndex(streamId, index);
}

ImageDataAndRecord VrsDataProvider::getImageDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getImageDataByIndex(streamId, index);
}

MotionData VrsDataProvider::getImuDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getImuDataByIndex(streamId, index);
}

GpsData VrsDataProvider::getGpsDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getGpsDataByIndex(streamId, index);
}

WifiBeaconData VrsDataProvider::getWpsDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getWpsDataByIndex(streamId, index);
}

AudioDataAndRecord VrsDataProvider::getAudioDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getAudioDataByIndex(streamId, index);
}

BarometerData VrsDataProvider::getBarometerDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getBarometerDataByIndex(streamId, index);
}

BluetoothBeaconData VrsDataProvider::getBluetoothDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getBluetoothDataByIndex(streamId, index);
}

MotionData VrsDataProvider::getMagnetometerDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getMagnetometerDataByIndex(streamId, index);
}

/* get time range */
int64_t VrsDataProvider::getFirstTimeNs(const vrs::StreamId& streamId, const TimeDomain& timeDomain)
    const {
  checkAndThrow(
      interface_->getStreamIds().count(streamId) > 0,
      fmt::format("Cannot find streamId {}", streamId.getNumericName()));
  if (timeDomain == TimeDomain::TimeCode) {
    checkAndThrow(supportsTimeDomain(streamId, timeDomain));
    return convertFromDeviceTimeToTimeCodeNs(
        timeQuery_->getFirstTimeNs(streamId, TimeDomain::DeviceTime));
  } else if (timeDomain == TimeDomain::TicSync) {
    checkAndThrow(supportsTimeDomain(streamId, timeDomain));
    return convertFromDeviceTimeToSyncTimeNs(
        timeQuery_->getFirstTimeNs(streamId, TimeDomain::DeviceTime), TimeSyncMode::TIC_SYNC);
  }
  return timeQuery_->getFirstTimeNs(streamId, timeDomain);
}

int64_t VrsDataProvider::getLastTimeNs(const vrs::StreamId& streamId, const TimeDomain& timeDomain)
    const {
  checkAndThrow(
      interface_->getStreamIds().count(streamId) > 0,
      fmt::format("Cannot find streamId {}", streamId.getNumericName()));
  if (timeDomain == TimeDomain::TimeCode) {
    checkAndThrow(supportsTimeDomain(streamId, timeDomain));
    return convertFromDeviceTimeToTimeCodeNs(
        timeQuery_->getLastTimeNs(streamId, TimeDomain::DeviceTime));
  } else if (timeDomain == TimeDomain::TicSync) {
    checkAndThrow(supportsTimeDomain(streamId, timeDomain));
    return convertFromDeviceTimeToSyncTimeNs(
        timeQuery_->getLastTimeNs(streamId, TimeDomain::DeviceTime), TimeSyncMode::TIC_SYNC);
  }
  return timeQuery_->getLastTimeNs(streamId, timeDomain);
}

int64_t VrsDataProvider::getFirstTimeNsAllStreams(const TimeDomain& timeDomain) const {
  int64_t minTimestamp = std::numeric_limits<std::int64_t>::max();
  for (const auto& streamId : getAllStreams()) {
    if (!checkStreamIsActive(streamId)) {
      continue;
    }
    int64_t timestamp = getFirstTimeNs(streamId, timeDomain);
    if (timestamp == -1) {
      continue;
    }
    minTimestamp = std::min(minTimestamp, timestamp);
  }
  return (minTimestamp == std::numeric_limits<std::int64_t>::max()) ? -1 : minTimestamp;
}

int64_t VrsDataProvider::getLastTimeNsAllStreams(const TimeDomain& timeDomain) const {
  int64_t maxTimestamp = std::numeric_limits<std::int64_t>::lowest();
  for (const auto& streamId : getAllStreams()) {
    if (!checkStreamIsActive(streamId)) {
      continue;
    }
    maxTimestamp = std::max(maxTimestamp, getLastTimeNs(streamId, timeDomain));
  }
  return (maxTimestamp == std::numeric_limits<std::int64_t>::lowest()) ? -1 : maxTimestamp;
}

std::vector<int64_t> VrsDataProvider::getTimestampsNs(
    const vrs::StreamId& streamId,
    const TimeDomain& timeDomain) {
  checkAndThrow(
      supportsTimeDomain(streamId, timeDomain),
      fmt::format(
          "Timedomain {} not supported by stream {}", getName(timeDomain), streamId.getName()));
  return timeQuery_->getTimestampsNs(streamId, timeDomain);
}

bool VrsDataProvider::supportsTimeDomain(
    const vrs::StreamId& streamId,
    const TimeDomain& timeDomain) const {
  if (timeDomain == TimeDomain::RecordTime || timeDomain == TimeDomain::DeviceTime) {
    return true; // both record and device time are supported
  } else if (timeDomain == TimeDomain::HostTime) {
    // some modality does not support host timestamp
    return supportsHostTimeDomain(getSensorDataType(streamId));
  } else if (timeDomain == TimeDomain::TimeCode) {
    return timeSyncMapper_->supportsMode(TimeSyncMode::TIMECODE);
  } else { // timeDomain == TimeDomain::TicSync
    return timeSyncMapper_->supportsMode(TimeSyncMode::TIC_SYNC);
  }
}

int VrsDataProvider::getIndexByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNsInTimeDomain,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  checkAndThrow(
      supportsTimeDomain(streamId, timeDomain),
      fmt::format(
          "Time domain {} not supported for the stream {}",
          getName(timeDomain),
          streamId.getName()));
  // if timedomain is timecode, convert it to device time
  if (timeDomain == TimeDomain::TimeCode) {
    int64_t deviceTimeNs = convertFromTimeCodeToDeviceTimeNs(timeNsInTimeDomain);
    return getIndexByTimeNs(streamId, deviceTimeNs, TimeDomain::DeviceTime, timeQueryOptions);
  } else if (timeDomain == TimeDomain::TicSync) {
    int64_t deviceTimeNs =
        convertFromSyncTimeToDeviceTimeNs(timeNsInTimeDomain, TimeSyncMode::TIC_SYNC);
    return getIndexByTimeNs(streamId, deviceTimeNs, TimeDomain::DeviceTime, timeQueryOptions);
  } else {
    return timeQuery_->getIndexByTimeNs(streamId, timeNsInTimeDomain, timeDomain, timeQueryOptions);
  }
}

int64_t VrsDataProvider::convertFromTimeCodeToDeviceTimeNs(const int64_t timecodeTimeNs) const {
  return timeSyncMapper_->convertFromTimeCodeToDeviceTimeNs(timecodeTimeNs);
}

int64_t VrsDataProvider::convertFromDeviceTimeToTimeCodeNs(const int64_t deviceTimeNs) const {
  return timeSyncMapper_->convertFromDeviceTimeToTimeCodeNs(deviceTimeNs);
}

int64_t VrsDataProvider::convertFromSyncTimeToDeviceTimeNs(
    const int64_t syncTimeNs,
    const TimeSyncMode mode) const {
  return timeSyncMapper_->convertFromSyncTimeToDeviceTimeNs(syncTimeNs, mode);
}

int64_t VrsDataProvider::convertFromDeviceTimeToSyncTimeNs(
    const int64_t deviceTimeNs,
    const TimeSyncMode mode) const {
  return timeSyncMapper_->convertFromDeviceTimeToSyncTimeNs(deviceTimeNs, mode);
}

/* validate streamId */
bool VrsDataProvider::checkStreamIsActive(const vrs::StreamId& streamId) const {
  return interface_->getStreamIds().count(streamId) > 0;
}

bool VrsDataProvider::checkStreamIsType(const vrs::StreamId& streamId, SensorDataType type) const {
  checkAndThrow(interface_->getStreamIds().count(streamId) > 0);

  SensorDataType streamIdType = getSensorDataType(streamId);
  return streamIdType == type;
}

void VrsDataProvider::assertStreamIsActive(const vrs::StreamId& streamId) const {
  checkAndThrow(
      checkStreamIsActive(streamId),
      fmt::format("StreamId {} not activated", streamId.getNumericName()));
}

void VrsDataProvider::assertStreamIsType(const vrs::StreamId& streamId, SensorDataType type) const {
  checkAndThrow(
      checkStreamIsType(streamId, type),
      fmt::format("StreamId {} is not {} type streamId", streamId.getName(), getName(type)));
}
} // namespace projectaria::tools::data_provider
