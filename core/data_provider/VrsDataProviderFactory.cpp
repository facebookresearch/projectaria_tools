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
#include <data_provider/VrsDataProvider.h>

#include <map>
#include <set>

#include <calibration/loader/AriaCalibRescaleAndCrop.h>
#include <calibration/loader/DeviceCalibrationJson.h>

#include <vrs/MultiRecordFileReader.h>

#define DEFAULT_LOG_CHANNEL "VrsDataProvider"
#include <logging/Log.h>

namespace projectaria::tools::data_provider {
namespace {
SensorDataType getSensorDataType(const vrs::RecordableTypeId& id) {
  static const std::map<vrs::RecordableTypeId, SensorDataType> sensorTypeMap = {
      // Image
      {vrs::RecordableTypeId::SlamCameraData, SensorDataType::Image},
      {vrs::RecordableTypeId::RgbCameraRecordableClass, SensorDataType::Image},
      {vrs::RecordableTypeId::EyeCameraRecordableClass, SensorDataType::Image},
      {vrs::RecordableTypeId::GroundTruthDepthRecordableClass, SensorDataType::Image},
      {vrs::RecordableTypeId::AnnotationRecordableClass, SensorDataType::Image},

      // Imu
      {vrs::RecordableTypeId::SlamImuData, SensorDataType::Imu},
      {vrs::RecordableTypeId::ImuRecordableClass, SensorDataType::Imu},

      // GPS
      {vrs::RecordableTypeId::GpsRecordableClass, SensorDataType::Gps},

      // WPS
      {vrs::RecordableTypeId::WifiBeaconRecordableClass, SensorDataType::Wps},

      // Audio
      {vrs::RecordableTypeId::StereoAudioRecordableClass, SensorDataType::Audio},

      // Barometer
      {vrs::RecordableTypeId::BarometerRecordableClass, SensorDataType::Barometer},

      // bluetooth
      {vrs::RecordableTypeId::BluetoothBeaconRecordableClass, SensorDataType::Bluetooth},

      // Magnetometer: data is played through MotionPlayer
      {vrs::RecordableTypeId::SlamMagnetometerData, SensorDataType::Magnetometer},
  };

  auto iteratorIdAndType = sensorTypeMap.find(id);
  if (iteratorIdAndType != sensorTypeMap.end()) {
    return iteratorIdAndType->second;
  } else {
    return SensorDataType::NotValid;
  }
}

class VrsDataProviderFactory {
 public:
  explicit VrsDataProviderFactory(std::shared_ptr<vrs::MultiRecordFileReader> reader);

  std::shared_ptr<VrsDataProvider> createProvider();

 private:
  // load streams
  void addPlayers();
  // load calibration by reader_.get_tag
  void loadCalibration();
  // establish stream id <=> label mapping to associate streams with calibration
  void loadStreamIdLabelMapper();
  // change intrinsics based on camera configs
  void checkCalibrationConfigConsistency();
  // add the stream player that contains time code data
  void tryAddTimeSyncPlayer(const vrs::StreamId& streamId);

 private:
  std::shared_ptr<vrs::MultiRecordFileReader> reader_;

  std::map<vrs::StreamId, std::shared_ptr<ImageSensorPlayer>> imagePlayers_;
  std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>> motionPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<GpsPlayer>> gpsPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<WifiBeaconPlayer>> wpsPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<AudioPlayer>> audioPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<BarometerPlayer>> barometerPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<BluetoothBeaconPlayer>> bluetoothPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>> magnetometerPlayers_;
  std::shared_ptr<TimeSyncPlayer> timesyncPlayer_;

  std::shared_ptr<StreamIdLabelMapper> streamIdLabelMapper_;
  std::optional<calibration::DeviceCalibration> maybeDeviceCalib_;
};

VrsDataProviderFactory::VrsDataProviderFactory(std::shared_ptr<vrs::MultiRecordFileReader> reader)
    : reader_(reader) {
  addPlayers();
  loadCalibration();
  loadStreamIdLabelMapper();
  checkCalibrationConfigConsistency();
}

void VrsDataProviderFactory::addPlayers() {
  for (const auto& streamId : reader_->getStreams()) {
    const SensorDataType sensorDataType = getSensorDataType(streamId.getTypeId());

    switch (sensorDataType) {
      case SensorDataType::Image: {
        std::shared_ptr<ImageSensorPlayer> imagePlayer =
            std::make_shared<ImageSensorPlayer>(streamId);
        imagePlayers_[streamId] = std::move(imagePlayer);
        reader_->setStreamPlayer(streamId, imagePlayers_[streamId].get());
        XR_LOGI("streamId {} activated", streamId.getNumericName());
        break;
      }
      case SensorDataType::Imu: {
        std::shared_ptr<MotionSensorPlayer> motionPlayer =
            std::make_shared<MotionSensorPlayer>(streamId);
        motionPlayers_[streamId] = std::move(motionPlayer);
        reader_->setStreamPlayer(streamId, motionPlayers_[streamId].get());
        XR_LOGI("streamId {} activated", streamId.getNumericName());
        break;
      }
      case SensorDataType::Gps: {
        std::shared_ptr<GpsPlayer> gpsPlayer = std::make_shared<GpsPlayer>(streamId);
        gpsPlayers_[streamId] = std::move(gpsPlayer);
        reader_->setStreamPlayer(streamId, gpsPlayers_[streamId].get());
        XR_LOGI("streamId {} activated", streamId.getNumericName());
        break;
      }
      case SensorDataType::Wps: {
        std::shared_ptr<WifiBeaconPlayer> wpsPlayer = std::make_shared<WifiBeaconPlayer>(streamId);
        wpsPlayers_[streamId] = std::move(wpsPlayer);
        reader_->setStreamPlayer(streamId, wpsPlayers_[streamId].get());
        XR_LOGI("streamId {} activated", streamId.getNumericName());
        break;
      }
      case SensorDataType::Audio: {
        std::shared_ptr<AudioPlayer> audioPlayer = std::make_shared<AudioPlayer>(streamId);
        audioPlayers_[streamId] = std::move(audioPlayer);
        reader_->setStreamPlayer(streamId, audioPlayers_[streamId].get());
        XR_LOGI("streamId {} activated", streamId.getNumericName());
        break;
      }
      case SensorDataType::Barometer: {
        std::shared_ptr<BarometerPlayer> barometerPlayer =
            std::make_shared<BarometerPlayer>(streamId);
        barometerPlayers_[streamId] = std::move(barometerPlayer);
        reader_->setStreamPlayer(streamId, barometerPlayers_[streamId].get());
        XR_LOGI("streamId {} activated", streamId.getNumericName());
        break;
      }
      case SensorDataType::Bluetooth: {
        std::shared_ptr<BluetoothBeaconPlayer> bluetoothPlayer =
            std::make_shared<BluetoothBeaconPlayer>(streamId);
        bluetoothPlayers_[streamId] = std::move(bluetoothPlayer);
        reader_->setStreamPlayer(streamId, bluetoothPlayers_[streamId].get());
        XR_LOGI("streamId {} activated", streamId.getNumericName());
        break;
      }
      case SensorDataType::Magnetometer: {
        std::shared_ptr<MotionSensorPlayer> magnetometerPlayer =
            std::make_shared<MotionSensorPlayer>(streamId);
        magnetometerPlayers_[streamId] = std::move(magnetometerPlayer);
        reader_->setStreamPlayer(streamId, magnetometerPlayers_[streamId].get());
        XR_LOGI("streamId {} activated", streamId.getNumericName());
        break;
      }
      case SensorDataType::NotValid: {
        if (streamId.getTypeId() == vrs::RecordableTypeId::TimeRecordableClass) {
          tryAddTimeSyncPlayer(streamId);
        } else {
          XR_LOGI("Fail to activate streamId {}", streamId.getNumericName());
        }
        break;
      }
    }
  }
  checkAndThrow(reader_->readFirstConfigurationRecords(), "Fail to read all configuration records");
}

void VrsDataProviderFactory::loadCalibration() {
  const std::string calibJsonStr = reader_->getTag("calib_json");
  if (calibJsonStr.empty()) {
    XR_LOGW("VRS file does not contain calib_json field in VRS tags.");
    return;
  }
  maybeDeviceCalib_ = calibration::deviceCalibrationFromJson(calibJsonStr);
  checkAndThrow(
      maybeDeviceCalib_.has_value(),
      fmt::format("Cannot parse calib json string.\n{}", calibJsonStr));
}

void VrsDataProviderFactory::loadStreamIdLabelMapper() {
  streamIdLabelMapper_ = getAriaStreamIdLabelMapper();
}

void VrsDataProviderFactory::checkCalibrationConfigConsistency() {
  if (!maybeDeviceCalib_) { // no op if calibration doesn't exist
    return;
  }

  std::map<std::string, Eigen::Vector2i> labelToCameraResolution;
  for (const auto& label : maybeDeviceCalib_->getCameraLabels()) {
    bool isAriaEt = (label.rfind("camera-et", 0) == 0);
    const auto maybeStreamId = isAriaEt ? streamIdLabelMapper_->getStreamIdFromLabel("camera-et")
                                        : streamIdLabelMapper_->getStreamIdFromLabel(label);
    checkAndThrow(
        maybeStreamId.has_value(),
        fmt::format(
            "Label {} exists in camera calibration that doesn't appear to be an Aria stream",
            label));
    vrs::StreamId streamId = maybeStreamId.value();

    auto it = imagePlayers_.find(streamId);
    if (it == imagePlayers_.end()) {
      // Note: some labels maybe present in calibration but not in player
      continue;
    }
    const auto& player = it->second;
    const auto& config = player->getConfigRecord();
    int configWidth = isAriaEt ? config.imageWidth / 2 : config.imageWidth;
    int configHeight = config.imageHeight;

    Eigen::Vector2i resolution(configWidth, configHeight);

    auto camCalib = maybeDeviceCalib_->getCameraCalib(label);
    if (camCalib->getImageSize() != resolution) {
      labelToCameraResolution.emplace(label, resolution);
    }
  }
  calibration::tryCropAndScaleCameraCalibration(maybeDeviceCalib_.value(), labelToCameraResolution);
}

void VrsDataProviderFactory::tryAddTimeSyncPlayer(const vrs::StreamId& streamId) {
  checkAndThrow(streamId.getTypeId() == vrs::RecordableTypeId::TimeRecordableClass);
  auto timesyncPlayer = std::make_shared<TimeSyncPlayer>(streamId);
  reader_->readFirstConfigurationRecord(streamId, timesyncPlayer.get());
  const std::string mode = timesyncPlayer->getConfigRecord().mode;
  if (mode == "TIMECODE") {
    timesyncPlayer_ = timesyncPlayer;
    reader_->setStreamPlayer(streamId, timesyncPlayer_.get());
    XR_LOGI("Timecode stream found: {}", streamId.getNumericName());
  }
  // ignore streams where mode is not TIMECODE
}

std::shared_ptr<VrsDataProvider> VrsDataProviderFactory::createProvider() {
  bool hasStreamPlayer = false;
  if (imagePlayers_.size()) {
    hasStreamPlayer = true;
  }
  if (motionPlayers_.size()) {
    hasStreamPlayer = true;
  }
  checkAndThrow(hasStreamPlayer, "No stream activated, cannot create provider");

  auto timeCodeMapper = std::make_shared<TimeCodeMapper>(reader_, timesyncPlayer_);

  auto interface = std::make_shared<RecordReaderInterface>(
      reader_,
      imagePlayers_,
      motionPlayers_,
      gpsPlayers_,
      wpsPlayers_,
      audioPlayers_,
      barometerPlayers_,
      bluetoothPlayers_,
      magnetometerPlayers_,
      timeCodeMapper);

  auto configMap = std::make_shared<StreamIdConfigurationMapper>(
      reader_,
      imagePlayers_,
      motionPlayers_,
      gpsPlayers_,
      wpsPlayers_,
      audioPlayers_,
      barometerPlayers_,
      bluetoothPlayers_,
      magnetometerPlayers_);

  return std::make_shared<VrsDataProvider>(
      interface, configMap, timeCodeMapper, streamIdLabelMapper_, maybeDeviceCalib_);
}
} // namespace

std::shared_ptr<VrsDataProvider> createVrsDataProvider(const std::string& vrsFilename) {
  auto reader = std::make_shared<vrs::MultiRecordFileReader>();
  if (reader->open({vrsFilename})) {
    XR_LOGE("Cannot open vrsFile {}.", vrsFilename);
    return {};
  }
  VrsDataProviderFactory factory(reader);
  return factory.createProvider();
}

} // namespace projectaria::tools::data_provider
