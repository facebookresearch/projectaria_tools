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
#include <data_provider/SensorData.h>

namespace projectaria::tools::data_provider {

SensorData::SensorData(
    const vrs::StreamId& streamId,
    const SensorDataVariant& dataVariant,
    const SensorDataType& sensorDataType,
    const int64_t recordInfoTimeNs,
    const std::map<TimeSyncMode, int64_t>& timeSyncTimeNs)
    : streamId_(streamId),
      dataVariant_(dataVariant),
      sensorDataType_(sensorDataType),
      recordInfoTimeNs_(recordInfoTimeNs),
      timeSyncTimeNs_(timeSyncTimeNs) {
  if (dataVariant.index() == 0) { // monostate
    sensorDataType_ = SensorDataType::NotValid;
  }
}

vrs::StreamId SensorData::streamId() const {
  return streamId_;
};

SensorDataType SensorData::sensorDataType() const {
  return sensorDataType_;
};

ImageDataAndRecord SensorData::imageDataAndRecord() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Image, "Sensor data type is not image");
  return std::get<ImageDataAndRecord>(dataVariant_);
}
MotionData SensorData::imuData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Imu, "Sensor data type is not IMU");
  return std::get<MotionData>(dataVariant_);
}
GpsData SensorData::gpsData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Gps, "Sensor data type is not GPS");
  return std::get<GpsData>(dataVariant_);
}
WifiBeaconData SensorData::wpsData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Wps, "Sensor data type is not WPS");
  return std::get<WifiBeaconData>(dataVariant_);
}
AudioDataAndRecord SensorData::audioDataAndRecord() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Audio, "Sensor data type is not audio");
  return std::get<AudioDataAndRecord>(dataVariant_);
}
BarometerData SensorData::barometerData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Barometer, "Sensor data type is not barometer");
  return std::get<BarometerData>(dataVariant_);
}
BluetoothBeaconData SensorData::bluetoothData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Bluetooth, "Sensor data type is not bluetooth");
  return std::get<BluetoothBeaconData>(dataVariant_);
}
MotionData SensorData::magnetometerData() const {
  checkAndThrow(
      sensorDataType_ == SensorDataType::Magnetometer, "Sensor data type is not Magnetometer");
  return std::get<MotionData>(dataVariant_);
}
PpgData SensorData::ppgData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Ppg, "Sensor data type is not PPG");
  return std::get<PpgData>(dataVariant_);
}

AlsData SensorData::alsData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Als, "Sensor data type is not ALS");
  return std::get<AlsData>(dataVariant_);
}

TemperatureData SensorData::temperatureData() const {
  checkAndThrow(
      sensorDataType_ == SensorDataType::Temperature, "Sensor data type is not Temperature");
  return std::get<TemperatureData>(dataVariant_);
}

BatteryStatusData SensorData::batteryStatusData() const {
  checkAndThrow(
      sensorDataType_ == SensorDataType::BatteryStatus, "Sensor data type is not BatteryStatus");
  return std::get<BatteryStatusData>(dataVariant_);
}

FrontendOutput SensorData::vioData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Vio, "Sensor data type is not Vio");
  return std::get<FrontendOutput>(dataVariant_);
}

OnDeviceVioHighFreqData SensorData::vioHighFreqData() const {
  checkAndThrow(
      sensorDataType_ == SensorDataType::VioHighFreq, "Sensor data type is not VioHighFreq");
  return std::get<OnDeviceVioHighFreqData>(dataVariant_);
}

OnDeviceEyeGazeData SensorData::eyeGazeData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::EyeGaze, "Sensor data type is not EyeGaze");
  return std::get<OnDeviceEyeGazeData>(dataVariant_);
}

OnDeviceHandPoseData SensorData::handPoseData() const {
  checkAndThrow(sensorDataType_ == SensorDataType::HandPose, "Sensor data type is not HandPose");
  return std::get<OnDeviceHandPoseData>(dataVariant_);
}

int64_t SensorData::getDeviceTime() const {
  switch (sensorDataType_) {
    case SensorDataType::Image:
      return imageDataAndRecord().second.captureTimestampNs;
    case SensorDataType::Imu:
      return imuData().captureTimestampNs;
    case SensorDataType::Audio: {
      const auto& timestampsNs = audioDataAndRecord().second.captureTimestampsNs;
      return timestampsNs.empty() ? -1 : timestampsNs.back();
    }
    case SensorDataType::Barometer:
      return barometerData().captureTimestampNs;
    case SensorDataType::Gps:
      return gpsData().captureTimestampNs;
    case SensorDataType::Wps:
      return -1; // wpsData().boardTimestampNs is measured as time since epoch.
    case SensorDataType::Magnetometer:
      return magnetometerData().captureTimestampNs;
    case SensorDataType::Bluetooth:
      return -1; // bluetoothData().boardTimestampNs is measured as time since epoch.
    case SensorDataType::Ppg:
      return ppgData().captureTimestampNs;
    case SensorDataType::Als:
      return alsData().captureTimestampNs;
    case SensorDataType::Temperature:
      return temperatureData().captureTimestampNs;
    case SensorDataType::BatteryStatus:
      return batteryStatusData().captureTimestampNs;

    // on device MP data
    case SensorDataType::Vio:
      return vioData().captureTimestampNs;
    case SensorDataType::VioHighFreq:
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
                 vioHighFreqData().trackingTimestamp)
          .count();
    case SensorDataType::EyeGaze:
      return std::chrono::duration_cast<std::chrono::nanoseconds>(eyeGazeData().trackingTimestamp)
          .count();
    case SensorDataType::HandPose:
      return std::chrono::duration_cast<std::chrono::nanoseconds>(handPoseData().trackingTimestamp)
          .count();
    case SensorDataType::NotValid:
      return -1;
  }
  return -1;
}

int64_t SensorData::getHostTime() const {
  switch (sensorDataType_) {
    case SensorDataType::Image:
      return imageDataAndRecord().second.arrivalTimestampNs;
    case SensorDataType::Imu:
      return imuData().arrivalTimestampNs;
    case SensorDataType::Wps:
      return wpsData().systemTimestampNs;
    case SensorDataType::Magnetometer:
      return magnetometerData().arrivalTimestampNs;
    case SensorDataType::Bluetooth:
      return bluetoothData().systemTimestampNs;
    // For the following sensor data types, host time is not available
    case SensorDataType::Audio:
    case SensorDataType::Barometer:
    case SensorDataType::BatteryStatus:
    case SensorDataType::Gps:
    case SensorDataType::Ppg:
    case SensorDataType::Als:
    case SensorDataType::Temperature:
    case SensorDataType::Vio:
    case SensorDataType::VioHighFreq:
    case SensorDataType::EyeGaze:
    case SensorDataType::HandPose:
    case SensorDataType::NotValid:
      return -1;
  }
  return -1;
}

int64_t SensorData::getTimeNs(TimeDomain timeDomain) const {
  switch (timeDomain) {
    case TimeDomain::RecordTime:
      return recordInfoTimeNs_;
    case TimeDomain::DeviceTime:
      return getDeviceTime();
    case TimeDomain::HostTime:
      return getHostTime();
    case TimeDomain::TimeCode:
      if (timeSyncTimeNs_.count(TimeSyncMode::TIMECODE) > 0) {
        return timeSyncTimeNs_.at(TimeSyncMode::TIMECODE);
      }
      return -1; // maintain backward compatibility for -1 value when timecode not available
    case TimeDomain::TicSync:
      if (timeSyncTimeNs_.count(TimeSyncMode::TIC_SYNC) > 0) {
        return timeSyncTimeNs_.at(TimeSyncMode::TIC_SYNC);
      }
      return -1;
    case TimeDomain::SubGhz:
      if (timeSyncTimeNs_.count(TimeSyncMode::SUBGHZ) > 0) {
        return timeSyncTimeNs_.at(TimeSyncMode::SUBGHZ);
      }
      return -1;
    case TimeDomain::Utc:
      if (timeSyncTimeNs_.count(TimeSyncMode::UTC) > 0) {
        return timeSyncTimeNs_.at(TimeSyncMode::UTC);
      }
      return -1;
    case TimeDomain::Count:
      checkAndThrow(false, "Invalid time domain: TimeDomain::Count");
      break;
  }
  return -1;
}

} // namespace projectaria::tools::data_provider
