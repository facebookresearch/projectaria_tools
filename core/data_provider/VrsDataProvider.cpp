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
#include <image/ImageVariant.h>
#include <image/utility/ColorCorrect.h>
#define DEFAULT_LOG_CHANNEL "VrsDataProvider"
#include <fmt/core.h>
#include <image/CopyToPixelFrame.h>
#include <image/utility/Devignetting.h>
#include <stdexcept>

#include <limits>
using namespace projectaria::tools::image;
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
      maybeDeviceCalib_(maybeDeviceCalib) {
  rgbIspTuningVersion_ = interface_ ? interface_->getRgbIspTuningVersion() : 0;
}

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
    case SensorDataType::Ppg:
      return SensorConfiguration(getPpgConfiguration(streamId), sensorDataType);
    case SensorDataType::Als:
      return SensorConfiguration(getAlsConfiguration(streamId), sensorDataType);
    case SensorDataType::Temperature:
      return SensorConfiguration(getTemperatureConfiguration(streamId), sensorDataType);
    case SensorDataType::BatteryStatus:
      return SensorConfiguration(getBatteryStatusConfiguration(streamId), sensorDataType);
    case SensorDataType::VioHighFreq:
      return SensorConfiguration(getVioHighFreqConfiguration(streamId), sensorDataType);
    case SensorDataType::Vio:
      return SensorConfiguration(getVioConfiguration(streamId), sensorDataType);
    case SensorDataType::EyeGaze:
      return SensorConfiguration(getEyeGazeConfiguration(streamId), sensorDataType);
    case SensorDataType::HandPose:
      return SensorConfiguration(getHandPoseConfiguration(streamId), sensorDataType);
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

PpgConfiguration VrsDataProvider::getPpgConfiguration(const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Ppg);
  return configMap_->getPpgConfiguration(streamId);
}

AlsConfiguration VrsDataProvider::getAlsConfiguration(const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Als);
  return configMap_->getAlsConfiguration(streamId);
}

TemperatureConfiguration VrsDataProvider::getTemperatureConfiguration(
    const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Temperature);
  return configMap_->getTemperatureConfiguration(streamId);
}

BatteryStatusConfiguration VrsDataProvider::getBatteryStatusConfiguration(
    const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::BatteryStatus);
  return configMap_->getBatteryStatusConfiguration(streamId);
}

EyeGazeConfiguration VrsDataProvider::getEyeGazeConfiguration(const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::EyeGaze);
  return configMap_->getEyeGazeConfiguration(streamId);
}

HandPoseConfiguration VrsDataProvider::getHandPoseConfiguration(
    const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::HandPose);
  return configMap_->getHandPoseConfiguration(streamId);
}

VioConfiguration VrsDataProvider::getVioConfiguration(const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Vio);
  return configMap_->getVioConfiguration(streamId);
}

VioHighFreqConfiguration VrsDataProvider::getVioHighFreqConfiguration(
    const vrs::StreamId& streamId) const {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::VioHighFreq);
  return configMap_->getVioHighFreqConfiguration(streamId);
}
/*******************************
 random access interfaces
 *******************************/
/* get dimension of data */
std::set<vrs::StreamId> VrsDataProvider::getAllStreams() const {
  return interface_->getStreamIds();
}

std::map<std::string, std::string> VrsDataProvider::getFileTags() const {
  return interface_->getFileTags();
}

std::optional<VrsMetadata> VrsDataProvider::getMetadata() const {
  return interface_->getMetadata();
}

std::optional<MetadataTimeSyncMode> VrsDataProvider::getTimeSyncMode() const {
  return interface_->getTimeSyncMode();
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

calibration::DeviceVersion VrsDataProvider::getDeviceVersion() const {
  if (maybeDeviceCalib_.has_value()) {
    return maybeDeviceCalib_->getDeviceVersion();
  } else {
    fmt::print("Device calibration is not found. Returned DeviceVersion::NotValid.\n");
    return calibration::DeviceVersion::NotValid;
  }
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
    SensorData sensorData = interface_->getLastCachedSensorData(streamId);
    // only post process rgb and slam images
    if (sensorData.sensorDataType() == SensorDataType::Image &&
        ((streamId.getTypeId() == vrs::RecordableTypeId::SlamCameraData) ||
         (streamId.getTypeId() == vrs::RecordableTypeId::RgbCameraRecordableClass))) {
      ImageDataPostProcessing(
          std::get<ImageDataAndRecord>(sensorData.dataVariant_).first, streamId);
    }
    return sensorData;
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
    ImageDataAndRecord imageDataAndRecord = interface_->getLastCachedImageData(streamId);
    if ((streamId.getTypeId() == vrs::RecordableTypeId::SlamCameraData) ||
        (streamId.getTypeId() == vrs::RecordableTypeId::RgbCameraRecordableClass)) {
      ImageDataPostProcessing(imageDataAndRecord.first, streamId);
    }
    return imageDataAndRecord;
  } else {
    return {};
  }
}

void VrsDataProvider::setDevignetting(bool applyDevignetting) {
  if (maybeDeviceCalib_.has_value() &&
      maybeDeviceCalib_->getDeviceVersion() == calibration::DeviceVersion::Gen2) {
    fmt::print(
        "Devignetting is now only supported for Aria Gen1, therefore it is NOT performed.\n");
    return;
  }
  applyDevignetting_ = applyDevignetting;
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

  // Issue an warning for Gen2 data readiness
  if (getDeviceVersion() == calibration::DeviceVersion::Gen2) {
    fmt::print("WARNING: GPS data quality is not yet fully validated in Aria Gen2. \n");
  }

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedGpsData(streamId);
  } else {
    return {};
  }
}

WifiBeaconData VrsDataProvider::getWpsDataByIndex(const vrs::StreamId& streamId, const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Wps);

  // Issue an warning for Gen2 data readiness
  if (getDeviceVersion() == calibration::DeviceVersion::Gen2) {
    fmt::print("WARNING: Wifi Beacon data quality is not yet fully validated in Aria Gen2. \n");
  }

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

  // Issue an warning for Gen2 data readiness
  if (getDeviceVersion() == calibration::DeviceVersion::Gen2) {
    fmt::print(
        "WARNING: Bluetooth Beacon data quality is not yet fully validated in Aria Gen2. \n");
  }
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

PpgData VrsDataProvider::getPpgDataByIndex(const vrs::StreamId& streamId, const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Ppg);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedPpgData(streamId);
  } else {
    return {};
  }
}

AlsData VrsDataProvider::getAlsDataByIndex(const vrs::StreamId& streamId, const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Als);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedAlsData(streamId);
  } else {
    return {};
  }
}

TemperatureData VrsDataProvider::getTemperatureDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Temperature);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedTemperatureData(streamId);
  } else {
    return {};
  }
}

BatteryStatusData VrsDataProvider::getBatteryStatusDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::BatteryStatus);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedBatteryStatusData(streamId);
  } else {
    return {};
  }
}

FrontendOutput VrsDataProvider::getVioDataByIndex(const vrs::StreamId& streamId, const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::Vio);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedVioData(streamId);
  } else {
    return {};
  }
}

OnDeviceVioHighFreqData VrsDataProvider::getVioHighFreqDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::VioHighFreq);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedVioHighFreqData(streamId);
  } else {
    return {};
  }
}

OnDeviceEyeGazeData VrsDataProvider::getEyeGazeDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::EyeGaze);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedEyeGazeData(streamId);
  } else {
    return {};
  }
}

OnDeviceHandPoseData VrsDataProvider::getHandPoseDataByIndex(
    const vrs::StreamId& streamId,
    const int index) {
  assertStreamIsActive(streamId);
  assertStreamIsType(streamId, SensorDataType::HandPose);

  if (interface_->readRecordByIndex(streamId, index)) {
    return interface_->getLastCachedHandPoseData(streamId);
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

PpgData VrsDataProvider::getPpgDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getPpgDataByIndex(streamId, index);
}

AlsData VrsDataProvider::getAlsDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getAlsDataByIndex(streamId, index);
}

TemperatureData VrsDataProvider::getTemperatureDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getTemperatureDataByIndex(streamId, index);
}

BatteryStatusData VrsDataProvider::getBatteryStatusDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getBatteryStatusDataByIndex(streamId, index);
}

FrontendOutput VrsDataProvider::getVioDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getVioDataByIndex(streamId, index);
}

OnDeviceVioHighFreqData VrsDataProvider::getVioHighFreqDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getVioHighFreqDataByIndex(streamId, index);
}

OnDeviceEyeGazeData VrsDataProvider::getEyeGazeDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getEyeGazeDataByIndex(streamId, index);
}

OnDeviceHandPoseData VrsDataProvider::getHandPoseDataByTimeNs(
    const vrs::StreamId& streamId,
    const int64_t timeNs,
    const TimeDomain& timeDomain,
    const TimeQueryOptions& timeQueryOptions) {
  const int index = getIndexByTimeNs(streamId, timeNs, timeDomain, timeQueryOptions);
  return getHandPoseDataByIndex(streamId, index);
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
  } else if (timeDomain == TimeDomain::SubGhz) {
    checkAndThrow(supportsTimeDomain(streamId, timeDomain));
    return convertFromDeviceTimeToSyncTimeNs(
        timeQuery_->getFirstTimeNs(streamId, TimeDomain::DeviceTime), TimeSyncMode::SUBGHZ);
  } else if (timeDomain == TimeDomain::Utc) {
    checkAndThrow(supportsTimeDomain(streamId, timeDomain));
    return convertFromDeviceTimeToSyncTimeNs(
        timeQuery_->getFirstTimeNs(streamId, TimeDomain::DeviceTime), TimeSyncMode::UTC);
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
  } else if (timeDomain == TimeDomain::SubGhz) {
    checkAndThrow(supportsTimeDomain(streamId, timeDomain));
    return convertFromDeviceTimeToSyncTimeNs(
        timeQuery_->getLastTimeNs(streamId, TimeDomain::DeviceTime), TimeSyncMode::SUBGHZ);
  } else if (timeDomain == TimeDomain::Utc) {
    checkAndThrow(supportsTimeDomain(streamId, timeDomain));
    return convertFromDeviceTimeToSyncTimeNs(
        timeQuery_->getLastTimeNs(streamId, TimeDomain::DeviceTime), TimeSyncMode::UTC);
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
  } else if (timeDomain == TimeDomain::SubGhz) {
    return timeSyncMapper_->supportsMode(TimeSyncMode::SUBGHZ);
  } else if (timeDomain == TimeDomain::Utc) {
    return timeSyncMapper_->supportsMode(TimeSyncMode::UTC);
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
  } else if (timeDomain == TimeDomain::SubGhz) {
    int64_t deviceTimeNs =
        convertFromSyncTimeToDeviceTimeNs(timeNsInTimeDomain, TimeSyncMode::SUBGHZ);
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

void VrsDataProvider::lazyLoadDevignettingMasks() {
  std::vector<std::string> labels = {"camera-rgb", "camera-slam-left", "camera-slam-right"};
  for (const auto& label : labels) {
    auto devignettingMask = loadDevignettingMask(label);
    devignettingMasks_[label] =
        std::make_shared<image::ManagedImage3F32>(std::move(devignettingMask));
  }
}

void VrsDataProvider::ImageDataPostProcessing(
    ImageData& srcImageData,
    const vrs::StreamId& streamId) {
  // This function applies devignetting and color correction to image data
  // Load the vignetting mask from cache if it exists, otherwise load it from file
  auto maybeLabel = getLabelFromStreamId(streamId);
  if (!maybeLabel) {
    std::string typeIdStr = vrs::toString(streamId.getTypeId());
    std::string instanceIdStr = std::to_string(streamId.getInstanceId());
    throw std::runtime_error(
        "Label not found for streamId during post processing, stream numeric name: " +
        streamId.getNumericName());
  }
  const std::string label = *maybeLabel;
  bool willApplyColorCorrection =
      (applyColorCorrection_ && rgbIspTuningVersion_ == 0 && label == "camera-rgb");
  if (applyDevignetting_ && devignettingMasks_.empty()) {
    lazyLoadDevignettingMasks();
  }
  if (std::optional<ImageVariant> imageVariantOpt = srcImageData.imageVariant()) {
    // case 0: without color correction and without devignetting
    if (!applyDevignetting_ && !willApplyColorCorrection) {
      return;
    }
    // case 1: with color correction and without devignetting
    else if (willApplyColorCorrection && !applyDevignetting_) {
      ManagedImageVariant colorCorrectedManagedImageVariant =
          colorCorrect(imageVariantOpt.value(), maybeDeviceCalib_->getDeviceVersion());
      copyToPixelFrame(colorCorrectedManagedImageVariant, *srcImageData.pixelFrame);
    }
    // case 2: without color correction and with devignetting
    else if (!willApplyColorCorrection && applyDevignetting_) {
      ManagedImageVariant devignettedManagedImageVariant =
          devignetting(imageVariantOpt.value(), *devignettingMasks_[label]);
      copyToPixelFrame(devignettedManagedImageVariant, *srcImageData.pixelFrame);
    }
    // case 3: with color correction and with devignetting
    else {
      ManagedImageVariant colorCorrectedManagedImageVariant =
          colorCorrect(imageVariantOpt.value(), maybeDeviceCalib_->getDeviceVersion());
      ManagedImageVariant devignettedManagedImageVariant = devignetting(
          toImageVariant(colorCorrectedManagedImageVariant), *devignettingMasks_[label]);
      copyToPixelFrame(devignettedManagedImageVariant, *srcImageData.pixelFrame);
    }
  }
}

void VrsDataProvider::setDevignettingMaskFolderPath(const std::string& maskFolderPath) {
  checkAndThrow(maybeDeviceCalib_.has_value(), "Device calibration is not found");
  if (maybeDeviceCalib_->getDeviceVersion() == calibration::DeviceVersion::Gen2) {
    fmt::print(
        "Color correction is now only supported for Aria Gen1, therefore it is NOT performed.\n");
    return;
  }
  maybeDeviceCalib_->setDevignettingMaskFolderPath(maskFolderPath);
}

projectaria::tools::image::ManagedImage3F32 VrsDataProvider::loadDevignettingMask(
    const std::string& label) {
  checkAndThrow(maybeDeviceCalib_.has_value(), "Device calibration is not found. \n");
  checkAndThrow(
      maybeDeviceCalib_->getDeviceVersion() == calibration::DeviceVersion::Gen1,
      "Devignetting is now only supported for Aria Gen1, therefore it is NOT performed. \n");
  return maybeDeviceCalib_->loadDevignettingMask(label, rgbIspTuningVersion_);
}

void VrsDataProvider::setColorCorrection(bool applyColorCorrection) {
  if (maybeDeviceCalib_.has_value() &&
      maybeDeviceCalib_->getDeviceVersion() == calibration::DeviceVersion::Gen2) {
    fmt::print(
        "Color correction is now only supported for Aria Gen1, therefore it is NOT performed.\n");
    return;
  }
  if (rgbIspTuningVersion_ == 1 && applyColorCorrection == true) {
    fmt::print("Aria recording is already color corrected. No need to set this flag.\n");
    return;
  }
  applyColorCorrection_ = applyColorCorrection;
}
} // namespace projectaria::tools::data_provider
