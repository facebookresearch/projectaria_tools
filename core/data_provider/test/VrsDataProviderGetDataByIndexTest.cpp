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

#include <data_provider/VrsDataProvider.h>

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

using namespace projectaria::tools::data_provider;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"

static const std::string ariaTestDataPath = XSTRING(TEST_FOLDER);

void compare(const ImageDataAndRecord& imageData1, const ImageDataAndRecord& imageData2) {
  EXPECT_EQ(imageData1.first.pixelFrame->getSpec(), imageData2.first.pixelFrame->getSpec());
  EXPECT_EQ(imageData1.second.arrivalTimestampNs, imageData2.second.arrivalTimestampNs);
  EXPECT_EQ(imageData1.second.captureTimestampNs, imageData2.second.captureTimestampNs);
  EXPECT_EQ(imageData1.second.exposureDuration, imageData2.second.exposureDuration);
  EXPECT_EQ(imageData1.second.frameNumber, imageData2.second.frameNumber);
  EXPECT_EQ(imageData1.second.gain, imageData2.second.gain);
  EXPECT_EQ(imageData1.second.groupId, imageData2.second.groupId);
  EXPECT_EQ(imageData1.second.groupMask, imageData2.second.groupMask);
  EXPECT_THAT(
      imageData1.second.temperature, testing::NanSensitiveDoubleEq(imageData2.second.temperature));
}

void compare(const MotionData& imuData1, const MotionData& imuData2) {
  EXPECT_EQ(imuData1.accelMSec2, imuData2.accelMSec2);
  EXPECT_EQ(imuData1.gyroRadSec, imuData2.gyroRadSec);
  EXPECT_EQ(imuData1.accelMSec2, imuData2.accelMSec2);
  EXPECT_EQ(imuData1.gyroRadSec, imuData2.gyroRadSec);
  EXPECT_EQ(imuData1.magValid, imuData2.magValid);
  EXPECT_EQ(imuData1.magTesla, imuData2.magTesla);
  EXPECT_EQ(imuData1.captureTimestampNs, imuData2.captureTimestampNs);
  EXPECT_EQ(imuData1.arrivalTimestampNs, imuData2.arrivalTimestampNs);
}

void compare(const GpsData& gpsData1, const GpsData& gpsData2) {
  EXPECT_EQ(gpsData1.captureTimestampNs, gpsData2.captureTimestampNs);
  EXPECT_EQ(gpsData1.utcTimeMs, gpsData2.utcTimeMs);
  EXPECT_EQ(gpsData1.provider, gpsData2.provider);
  EXPECT_EQ(gpsData1.latitude, gpsData2.latitude);
  EXPECT_EQ(gpsData1.longitude, gpsData2.longitude);
  EXPECT_EQ(gpsData1.altitude, gpsData2.altitude);
  EXPECT_EQ(gpsData1.accuracy, gpsData2.accuracy);
  EXPECT_EQ(gpsData1.speed, gpsData2.speed);
}

void compare(const WifiBeaconData& wpsData1, const WifiBeaconData& wpsData2) {
  EXPECT_EQ(wpsData1.systemTimestampNs, wpsData2.systemTimestampNs);
  EXPECT_EQ(wpsData1.boardTimestampNs, wpsData2.boardTimestampNs);
  EXPECT_EQ(wpsData1.boardScanRequestStartTimestampNs, wpsData2.boardScanRequestStartTimestampNs);
  EXPECT_EQ(
      wpsData1.boardScanRequestCompleteTimestampNs, wpsData2.boardScanRequestCompleteTimestampNs);
  EXPECT_EQ(wpsData1.ssid, wpsData2.ssid);
  EXPECT_EQ(wpsData1.bssidMac, wpsData2.bssidMac);
  EXPECT_EQ(wpsData1.rssi, wpsData2.rssi);
  EXPECT_EQ(wpsData1.freqMhz, wpsData2.freqMhz);
}

void compare(const AudioDataAndRecord& audioData1, const AudioDataAndRecord& audioData2) {
  EXPECT_EQ(audioData1.first.data, audioData2.first.data);
  EXPECT_EQ(audioData1.second.captureTimestampsNs, audioData2.second.captureTimestampsNs);
  EXPECT_EQ(audioData1.second.audioMuted, audioData2.second.audioMuted);
}

void compare(const BluetoothBeaconData& bluetoothData1, const BluetoothBeaconData& bluetoothData2) {
  EXPECT_EQ(bluetoothData1.systemTimestampNs, bluetoothData2.systemTimestampNs);
  EXPECT_EQ(bluetoothData1.boardTimestampNs, bluetoothData2.boardTimestampNs);
  EXPECT_EQ(
      bluetoothData1.boardScanRequestStartTimestampNs,
      bluetoothData2.boardScanRequestStartTimestampNs);
  EXPECT_EQ(
      bluetoothData1.boardScanRequestCompleteTimestampNs,
      bluetoothData2.boardScanRequestCompleteTimestampNs);
  EXPECT_EQ(bluetoothData1.uniqueId, bluetoothData2.uniqueId);
  EXPECT_EQ(bluetoothData1.txPower, bluetoothData2.txPower);
  EXPECT_EQ(bluetoothData1.rssi, bluetoothData2.rssi);
  EXPECT_EQ(bluetoothData1.freqMhz, bluetoothData2.freqMhz);
}

void compare(const BarometerData& barometerData1, const BarometerData& barometerData2) {
  EXPECT_EQ(barometerData1.altitude, barometerData2.altitude);
  EXPECT_EQ(barometerData1.captureTimestampNs, barometerData2.captureTimestampNs);
  EXPECT_EQ(barometerData1.pressure, barometerData2.pressure);
  EXPECT_EQ(barometerData1.temperature, barometerData2.temperature);
}

void checkGetDataByIndex(
    const std::shared_ptr<VrsDataProvider>& provider,
    const vrs::StreamId& streamId) {
  size_t numData = provider->getNumData(streamId);
  int lastDeviceTime = -1;

  for (int f = 0; f < numData; ++f) {
    const auto sensorData = provider->getSensorDataByIndex(streamId, f);
    EXPECT_EQ(sensorData.sensorDataType(), provider->getSensorDataType(streamId));

    int64_t currentDeviceTime = sensorData.getTimeNs(TimeDomain::DeviceTime);
    EXPECT_LE(lastDeviceTime, currentDeviceTime);
    lastDeviceTime = currentDeviceTime;

    switch (sensorData.sensorDataType()) {
      case SensorDataType::Image: {
        const auto imageData = provider->getImageDataByIndex(streamId, f);
        compare(imageData, sensorData.imageDataAndRecord());
        break;
      }
      case SensorDataType::Imu: {
        const auto imuData = provider->getImuDataByIndex(streamId, f);
        compare(imuData, sensorData.imuData());
        break;
      }
      case SensorDataType::Gps: {
        const auto gpsData = provider->getGpsDataByIndex(streamId, f);
        compare(gpsData, sensorData.gpsData());
        break;
      }
      case SensorDataType::Wps: {
        const auto wpsData = provider->getWpsDataByIndex(streamId, f);
        compare(wpsData, sensorData.wpsData());
        break;
      }
      case SensorDataType::Audio: {
        const auto audioData = provider->getAudioDataByIndex(streamId, f);
        break;
      }
      case SensorDataType::Bluetooth: {
        const auto bluetoothData = provider->getBluetoothDataByIndex(streamId, f);
        compare(bluetoothData, sensorData.bluetoothData());
        break;
      }
      case SensorDataType::Barometer: {
        const auto barometerData = provider->getBarometerDataByIndex(streamId, f);
        compare(barometerData, sensorData.barometerData());
        break;
      }
      case SensorDataType::Magnetometer: {
        const auto magnetometerData = provider->getMagnetometerDataByIndex(streamId, f);
        compare(magnetometerData, sensorData.magnetometerData());
        break;
      }
      default:
        break;
    }
  }
}

TEST(VrsDataProvider, getDataByIndex) {
  auto provider = createVrsDataProvider(ariaTestDataPath);

  const auto streamIds = provider->getAllStreams();
  for (const auto streamId : streamIds) {
    checkGetDataByIndex(provider, streamId);
  }
}

TEST(VrsDataProvider, multiThreadGetDataByIndex) {
  auto provider = createVrsDataProvider(ariaTestDataPath);

  const auto streamIds = provider->getAllStreams();
  static constexpr int numProviderRequests = 10;
  std::vector<std::vector<std::thread>> threads(numProviderRequests);
  for (size_t i = 0; i < numProviderRequests; ++i) {
    for (const auto streamId : streamIds) {
      threads[i].emplace_back(
          std::thread([&provider, streamId]() { checkGetDataByIndex(provider, streamId); }));
    }

    for (auto& thread : threads[i]) {
      thread.join();
    }
  }
}
