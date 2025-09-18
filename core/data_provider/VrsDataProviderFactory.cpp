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
#include "vrs/StreamId.h"

#include <calibration/loader/AriaCalibRescaleAndCrop.h>
#include <calibration/loader/DeviceCalibrationJson.h>
#include <vrs/MultiRecordFileReader.h>
#include <vrs/utils/xprs/XprsDecoder.h>

#define DEFAULT_LOG_CHANNEL "VrsDataProvider"
#include <logging/Log.h>

namespace projectaria::tools::data_provider {
namespace {
SensorDataType getSensorDataType(const vrs::RecordableTypeId& id, const std::string& streamFlavor) {
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

      // Ppg
      {vrs::RecordableTypeId::PhotoplethysmogramRecordableClass, SensorDataType::Ppg},

      // ALS
      {vrs::RecordableTypeId::AmbientLightRecordableClass, SensorDataType::Als},

      // Temperature
      {vrs::RecordableTypeId::TemperatureRecordableClass, SensorDataType::Temperature},

      // eyegaze
      {vrs::RecordableTypeId::GazeRecordableClass, SensorDataType::EyeGaze},
  };

  // First, check for PoseRecordableClass, which is a special case that can map to Vio, VioHighFreq,
  // and HandTracking. Need to determine from stream flavor
  if (id == vrs::RecordableTypeId::PoseRecordableClass) {
    if (streamFlavor == "device/oatmeal/vio_high_frequency") {
      return SensorDataType::VioHighFreq;
    } else if (streamFlavor == "device/oatmeal/vio") {
      return SensorDataType::Vio;
    } else if (streamFlavor == "device/oatmeal/hand") {
      return SensorDataType::HandPose;
    } else {
      fmt::print("WARNING: Unsupported flavor for PoseRecordableClass: {}\n", streamFlavor);
      return SensorDataType::NotValid;
    }
  }

  // For all other cases, check the static map
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
  // load device version (Gen1, Gen2)
  void loadDeviceVersion();
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
  // backfill T_Cpf_Device values to EyeGazePlayers
  void setT_Cpf_DeviceForEyeGazePlayers();
  // SEt T_BodyImu_Device for VioPlayers
  void setT_BodyImu_DeviceForVioPlayers();

 private:
  std::shared_ptr<vrs::MultiRecordFileReader> reader_;
  calibration::DeviceVersion deviceVersion_;

  std::map<vrs::StreamId, std::shared_ptr<ImageSensorPlayer>> imagePlayers_;
  std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>> motionPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<GpsPlayer>> gpsPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<WifiBeaconPlayer>> wpsPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<AudioPlayer>> audioPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<BarometerPlayer>> barometerPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<BatteryStatusPlayer>> batteryStatusPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<BluetoothBeaconPlayer>> bluetoothPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>> magnetometerPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<PpgPlayer>> ppgPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<AlsPlayer>> alsPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<TemperaturePlayer>> temperaturePlayers_;
  std::map<vrs::StreamId, std::shared_ptr<VioPlayer>> vioPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<VioHighFrequencyPlayer>> vioHighFreqPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<EyeGazePlayer>> eyeGazePlayers_;
  std::map<vrs::StreamId, std::shared_ptr<HandPosePlayer>> handPosePlayers_;

  std::map<TimeSyncMode, std::shared_ptr<TimeSyncPlayer>> timesyncPlayers_;

  std::shared_ptr<StreamIdLabelMapper> streamIdLabelMapper_;
  std::optional<calibration::DeviceCalibration> maybeDeviceCalib_;
};

VrsDataProviderFactory::VrsDataProviderFactory(std::shared_ptr<vrs::MultiRecordFileReader> reader)
    : reader_(reader) {
  loadDeviceVersion();
  loadStreamIdLabelMapper();
  addPlayers();
  loadCalibration();
  checkCalibrationConfigConsistency();
  setT_Cpf_DeviceForEyeGazePlayers();
  setT_BodyImu_DeviceForVioPlayers();
}

void VrsDataProviderFactory::loadDeviceVersion() {
  const std::string deviceTypeString = reader_->getTag("device_type");
  deviceVersion_ = calibration::fromDeviceClassName(deviceTypeString);
  if (deviceVersion_ == calibration::DeviceVersion::NotValid) {
    fmt::print(
        "WARNING: Invalid device version, vrs tag for device type is {}. This may be a non-Aria recording. Setting this to Aria Gen1. \n",
        deviceTypeString);
    deviceVersion_ = calibration::DeviceVersion::Gen1;
  }
}

void VrsDataProviderFactory::addPlayers() {
  for (const auto& streamId : reader_->getStreams()) {
    const SensorDataType sensorDataType =
        getSensorDataType(streamId.getTypeId(), reader_->getFlavor(streamId));

    // Define a lambda that sets the StreamPlayer to the reader and log its streamId
    auto setStreamAndLog = [=, this](
                               const vrs::StreamId, vrs::RecordFormatStreamPlayer* player) -> void {
      reader_->setStreamPlayer(streamId, player);
      XR_LOGI(
          "streamId {}/{} activated",
          streamId.getNumericName(),
          streamIdLabelMapper_->getLabelFromStreamId(streamId)
              ? streamIdLabelMapper_->getLabelFromStreamId(streamId).value()
              : "NA");
    };

    switch (sensorDataType) {
      case SensorDataType::Image: {
        std::shared_ptr<ImageSensorPlayer> imagePlayer =
            std::make_shared<ImageSensorPlayer>(streamId);
        imagePlayers_[streamId] = std::move(imagePlayer);
        setStreamAndLog(streamId, imagePlayers_[streamId].get());
        break;
      }
      case SensorDataType::Imu: {
        std::shared_ptr<MotionSensorPlayer> motionPlayer =
            std::make_shared<MotionSensorPlayer>(streamId);
        motionPlayers_[streamId] = std::move(motionPlayer);
        setStreamAndLog(streamId, motionPlayers_[streamId].get());
        break;
      }
      case SensorDataType::Gps: {
        std::shared_ptr<GpsPlayer> gpsPlayer = std::make_shared<GpsPlayer>(streamId);
        gpsPlayers_[streamId] = std::move(gpsPlayer);
        setStreamAndLog(streamId, gpsPlayers_[streamId].get());
        break;
      }
      case SensorDataType::Wps: {
        std::shared_ptr<WifiBeaconPlayer> wpsPlayer = std::make_shared<WifiBeaconPlayer>(streamId);
        wpsPlayers_[streamId] = std::move(wpsPlayer);
        setStreamAndLog(streamId, wpsPlayers_[streamId].get());
        break;
      }
      case SensorDataType::Audio: {
        std::shared_ptr<AudioPlayer> audioPlayer = std::make_shared<AudioPlayer>(streamId);
        audioPlayers_[streamId] = std::move(audioPlayer);
        setStreamAndLog(streamId, audioPlayers_[streamId].get());
        break;
      }
      case SensorDataType::Barometer: {
        std::shared_ptr<BarometerPlayer> barometerPlayer =
            std::make_shared<BarometerPlayer>(streamId);
        barometerPlayers_[streamId] = std::move(barometerPlayer);
        setStreamAndLog(streamId, barometerPlayers_[streamId].get());
        break;
      }
      case SensorDataType::BatteryStatus: {
        std::shared_ptr<BatteryStatusPlayer> batteryStatusPlayer =
            std::make_shared<BatteryStatusPlayer>(streamId);
        batteryStatusPlayers_[streamId] = std::move(batteryStatusPlayer);
        setStreamAndLog(streamId, batteryStatusPlayers_[streamId].get());
        break;
      }
      case SensorDataType::Bluetooth: {
        std::shared_ptr<BluetoothBeaconPlayer> bluetoothPlayer =
            std::make_shared<BluetoothBeaconPlayer>(streamId);
        bluetoothPlayers_[streamId] = std::move(bluetoothPlayer);
        setStreamAndLog(streamId, bluetoothPlayers_[streamId].get());
        break;
      }
      case SensorDataType::Magnetometer: {
        std::shared_ptr<MotionSensorPlayer> magnetometerPlayer =
            std::make_shared<MotionSensorPlayer>(streamId);
        magnetometerPlayers_[streamId] = std::move(magnetometerPlayer);
        setStreamAndLog(streamId, magnetometerPlayers_[streamId].get());
        break;
      }
      case SensorDataType::Ppg: {
        std::shared_ptr<PpgPlayer> ppgPlayer = std::make_shared<PpgPlayer>(streamId);
        ppgPlayers_[streamId] = std::move(ppgPlayer);
        setStreamAndLog(streamId, ppgPlayers_[streamId].get());
        break;
      }
      case SensorDataType::Als: {
        std::shared_ptr<AlsPlayer> alsPlayer = std::make_shared<AlsPlayer>(streamId);
        alsPlayers_[streamId] = std::move(alsPlayer);
        setStreamAndLog(streamId, alsPlayers_[streamId].get());
        break;
      }
      case SensorDataType::Temperature: {
        std::shared_ptr<TemperaturePlayer> temperaturePlayer =
            std::make_shared<TemperaturePlayer>(streamId);
        temperaturePlayers_[streamId] = std::move(temperaturePlayer);
        setStreamAndLog(streamId, temperaturePlayers_[streamId].get());
        break;
      }
      case SensorDataType::Vio: {
        std::shared_ptr<VioPlayer> vioPlayer = std::make_shared<VioPlayer>(streamId);
        vioPlayers_[streamId] = std::move(vioPlayer);
        setStreamAndLog(streamId, vioPlayers_[streamId].get());
        break;
      }
      case SensorDataType::VioHighFreq: {
        std::shared_ptr<VioHighFrequencyPlayer> vioHighFreqPlayer =
            std::make_shared<VioHighFrequencyPlayer>(streamId);
        vioHighFreqPlayers_[streamId] = std::move(vioHighFreqPlayer);
        setStreamAndLog(streamId, vioHighFreqPlayers_[streamId].get());
        break;
      }
      case SensorDataType::EyeGaze: {
        std::shared_ptr<EyeGazePlayer> eyeGazePlayer = std::make_shared<EyeGazePlayer>(streamId);
        eyeGazePlayers_[streamId] = std::move(eyeGazePlayer);
        setStreamAndLog(streamId, eyeGazePlayers_[streamId].get());
        break;
      }
      case SensorDataType::HandPose: {
        std::shared_ptr<HandPosePlayer> handPosePlayer = std::make_shared<HandPosePlayer>(streamId);
        handPosePlayers_[streamId] = std::move(handPosePlayer);
        setStreamAndLog(streamId, handPosePlayers_[streamId].get());
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
  streamIdLabelMapper_ = getAriaStreamIdLabelMapper(deviceVersion_, reader_);
}

void VrsDataProviderFactory::checkCalibrationConfigConsistency() {
  if (!maybeDeviceCalib_) { // no op if calibration doesn't exist
    return;
  }
  if (maybeDeviceCalib_->getDeviceSubtype() == "SimulatedDevice") { // skip SimulatedDevice type
    return;
  }

  std::map<std::string, Eigen::Vector2i> labelToCameraResolution;
  for (const auto& label : maybeDeviceCalib_->getCameraLabels()) {
    // Aria Gen1 ET needs to be handled differently, where the vrs stream contains stitched images
    // from left and right ET cameras.
    bool isAriaGen1Et =
        (label.rfind("camera-et", 0) == 0) && (deviceVersion_ == calibration::DeviceVersion::Gen1);
    const auto maybeStreamId = isAriaGen1Et
        ? streamIdLabelMapper_->getStreamIdFromLabel("camera-et")
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
    int configWidth = isAriaGen1Et ? config.imageWidth / 2 : config.imageWidth;
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

  if (mode == "TIC_SYNC") {
    timesyncPlayers_[TimeSyncMode::TIC_SYNC] = std::move(timesyncPlayer);
    reader_->setStreamPlayer(streamId, timesyncPlayers_[TimeSyncMode::TIC_SYNC].get());
    XR_LOGI("TicSync stream found: {}", streamId.getNumericName());
  } else if (mode == "TIMECODE") {
    timesyncPlayers_[TimeSyncMode::TIMECODE] = std::move(timesyncPlayer);
    reader_->setStreamPlayer(streamId, timesyncPlayers_[TimeSyncMode::TIMECODE].get());
    XR_LOGI("Timecode stream found: {}", streamId.getNumericName());
  } else if (mode == "SUBGHZ") {
    timesyncPlayers_[TimeSyncMode::SUBGHZ] = std::move(timesyncPlayer);
    reader_->setStreamPlayer(streamId, timesyncPlayers_[TimeSyncMode::SUBGHZ].get());
    XR_LOGI("Subghz stream found: {}", streamId.getNumericName());
  }
  // "APP" mode obtains UTC timestamps, in an interval of 1 min.
  else if (mode == "APP") {
    timesyncPlayers_[TimeSyncMode::UTC] = std::move(timesyncPlayer);
    reader_->setStreamPlayer(streamId, timesyncPlayers_[TimeSyncMode::UTC].get());
    XR_LOGI("Utc stream found: {}", streamId.getNumericName());
  }

  else {
    XR_LOGW("Unsupported TimeSync mode: {}, ignoring.", mode);
  }
}

void VrsDataProviderFactory::setT_Cpf_DeviceForEyeGazePlayers() {
  // Only invoke this function if EyeGazePlayers are present
  if (eyeGazePlayers_.empty()) {
    return;
  }

  if (!maybeDeviceCalib_.has_value()) {
    fmt::print("Device calibration empty, T_Cpf_Device not set for EyeGazePlayers. \n");
    return;
  }

  for (auto& [stream_id, player] : eyeGazePlayers_) {
    player->setT_Cpf_Device(maybeDeviceCalib_->getT_Device_Cpf().inverse().cast<float>());
  }
}

void VrsDataProviderFactory::setT_BodyImu_DeviceForVioPlayers() {
  if (vioPlayers_.empty()) {
    return;
  }

  if (!maybeDeviceCalib_.has_value()) {
    throw std::runtime_error(
        "Device calibration needs to be valid in order to set T_BodyImu_Device to Vio Players");
  }

  Sophus::SE3f T_BodyImu_Device =
      maybeDeviceCalib_->getImuCalib("imu-left")->getT_Device_Imu().inverse().cast<float>();

  for (auto& [stream_id, player] : vioPlayers_) {
    player->setT_BodyImu_Device(T_BodyImu_Device);
  }
}

std::shared_ptr<VrsDataProvider> VrsDataProviderFactory::createProvider() {
  bool hasStreamPlayer = false;
  if (!imagePlayers_.empty()) {
    hasStreamPlayer = true;
  }
  if (!motionPlayers_.empty()) {
    hasStreamPlayer = true;
  }
  checkAndThrow(hasStreamPlayer, "No stream activated, cannot create provider");

  auto timeSyncMapper = std::make_shared<TimeSyncMapper>(reader_, timesyncPlayers_);

  auto interface = std::make_shared<RecordReaderInterface>(
      reader_,
      imagePlayers_,
      motionPlayers_,
      gpsPlayers_,
      wpsPlayers_,
      audioPlayers_,
      barometerPlayers_,
      batteryStatusPlayers_,
      bluetoothPlayers_,
      magnetometerPlayers_,
      ppgPlayers_,
      alsPlayers_,
      temperaturePlayers_,
      vioPlayers_,
      vioHighFreqPlayers_,
      eyeGazePlayers_,
      handPosePlayers_,
      timeSyncMapper);

  auto configMap = std::make_shared<StreamIdConfigurationMapper>(
      reader_,
      imagePlayers_,
      motionPlayers_,
      gpsPlayers_,
      wpsPlayers_,
      audioPlayers_,
      barometerPlayers_,
      batteryStatusPlayers_,
      bluetoothPlayers_,
      magnetometerPlayers_,
      ppgPlayers_,
      alsPlayers_,
      temperaturePlayers_,
      eyeGazePlayers_,
      handPosePlayers_,
      vioPlayers_,
      vioHighFreqPlayers_);

  return std::make_shared<VrsDataProvider>(
      interface, configMap, timeSyncMapper, streamIdLabelMapper_, maybeDeviceCalib_);
}
} // namespace

std::shared_ptr<VrsDataProvider> createVrsDataProvider(const std::string& vrsFilename) {
  // Add support for decoding H.265
  vrs::utils::DecoderFactory::get().registerDecoderMaker(vrs::vxprs::xprsDecoderMaker);

  auto reader = std::make_shared<vrs::MultiRecordFileReader>();
  if (reader->open({vrsFilename})) {
    XR_LOGE("Cannot open vrsFile {}.", vrsFilename);
    return {};
  }
  VrsDataProviderFactory factory(reader);
  return factory.createProvider();
}

} // namespace projectaria::tools::data_provider
