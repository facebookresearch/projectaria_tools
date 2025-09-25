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
#include <data_provider/SensorConfiguration.h>

namespace projectaria::tools::data_provider {

SensorConfiguration::SensorConfiguration(
    const SensorConfigurationVariant& sensorConfigurationVariant,
    const SensorDataType& sensorDataType)
    : sensorConfigurationVariant_(sensorConfigurationVariant), sensorDataType_(sensorDataType) {}

SensorDataType SensorConfiguration::sensorDataType() const {
  return sensorDataType_;
};

ImageConfigRecord SensorConfiguration::imageConfiguration() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Image, "Sensor data type is not image");
  return std::get<ImageConfigRecord>(sensorConfigurationVariant_);
}
MotionConfigRecord SensorConfiguration::imuConfiguration() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Imu, "Sensor data type is not IMU");
  return std::get<MotionConfigRecord>(sensorConfigurationVariant_);
}
GpsConfigRecord SensorConfiguration::gpsConfiguration() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Gps, "Sensor data type is not GPS");
  return std::get<GpsConfigRecord>(sensorConfigurationVariant_);
}
WifiBeaconConfigRecord SensorConfiguration::wpsConfiguration() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Wps, "Sensor data type is not WPS");
  return std::get<WifiBeaconConfigRecord>(sensorConfigurationVariant_);
}
AudioConfig SensorConfiguration::audioConfiguration() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Audio, "Sensor data type is not audio");
  return std::get<AudioConfig>(sensorConfigurationVariant_);
}
BarometerConfigRecord SensorConfiguration::barometerConfiguration() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Barometer, "Sensor data type is not barometer");
  return std::get<BarometerConfigRecord>(sensorConfigurationVariant_);
}
BluetoothBeaconConfigRecord SensorConfiguration::bluetoothConfiguration() const {
  checkAndThrow(sensorDataType_ == SensorDataType::Bluetooth, "Sensor data type is not bluetooth");
  return std::get<BluetoothBeaconConfigRecord>(sensorConfigurationVariant_);
}
MotionConfigRecord SensorConfiguration::magnetometerConfiguration() const {
  checkAndThrow(
      sensorDataType_ == SensorDataType::Magnetometer, "Sensor data type is not Magnetometer");
  return std::get<MotionConfigRecord>(sensorConfigurationVariant_);
}

double SensorConfiguration::getNominalRateHz() const {
  switch (sensorDataType_) {
    case SensorDataType::Image:
      return imageConfiguration().nominalRateHz;
    case SensorDataType::Imu:
      return imuConfiguration().nominalRateHz;
    case SensorDataType::Audio:
      return audioConfiguration().sampleRate;
    case SensorDataType::Barometer:
      return barometerConfiguration().sampleRate;
    case SensorDataType::Gps:
      return gpsConfiguration().sampleRateHz;
    case SensorDataType::Wps:
      return -1; // WPS doesn't have the field
    case SensorDataType::Magnetometer:
      return magnetometerConfiguration().nominalRateHz;
    case SensorDataType::Bluetooth:
      return bluetoothConfiguration().sampleRateHz;
    case SensorDataType::NotValid:
    default:
      return -1;
  }
}

} // namespace projectaria::tools::data_provider
