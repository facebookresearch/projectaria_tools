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
    const int64_t timeCodeTimeNs)
    : streamId_(streamId),
      dataVariant_(dataVariant),
      sensorDataType_(sensorDataType),
      recordInfoTimeNs_(recordInfoTimeNs),
      timeCodeTimeNs_(timeCodeTimeNs) {
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
    case SensorDataType::Audio:
      return -1;
    case SensorDataType::Barometer:
      return -1;
    case SensorDataType::Gps:
      return -1;
    case SensorDataType::Wps:
      return wpsData().systemTimestampNs;
    case SensorDataType::Magnetometer:
      return magnetometerData().arrivalTimestampNs;
    case SensorDataType::Bluetooth:
      return bluetoothData().systemTimestampNs;
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
      return timeCodeTimeNs_;
  }
  return -1;
}

} // namespace projectaria::tools::data_provider
