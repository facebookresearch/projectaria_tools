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
#include <data_provider/test/CompareDataHelper.h>
#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::mps;
using namespace projectaria::tools::data_provider::test;
#define STRING(x) #x
#define GEN1_STRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"
#define GEN2_STRING(x) std::string(STRING(x)) + "aria_gen2_unit_test_sequence.vrs"
#define GEN2_STRING_BLU(x) \
  std::string(STRING(x)) + "aria_gen2_unit_test_sequence_with_bluetooth.vrs"

static const std::string ariaGen1TestDataPath = GEN1_STRING(TEST_FOLDER);
static const std::string ariaGen2TestDataPath = GEN2_STRING(TEST_FOLDER_GEN2);
static const std::string ariaGen2TestDataPathWithBluetooth = GEN2_STRING_BLU(TEST_FOLDER_GEN2);

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
        // compare(gpsData, sensorData.gpsData()); // TODO: enable this after GPS data is available.
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
      case SensorDataType::Ppg: {
        const auto ppgData = provider->getPpgDataByIndex(streamId, f);
        compare(ppgData, sensorData.ppgData());
        break;
      }
      case SensorDataType::Als: {
        const auto alsData = provider->getAlsDataByIndex(streamId, f);
        compare(alsData, sensorData.alsData());
        break;
      }
      case SensorDataType::Temperature: {
        const auto temperatureData = provider->getTemperatureDataByIndex(streamId, f);
        compare(temperatureData, sensorData.temperatureData());
        break;
      }
      case SensorDataType::Vio: {
        const auto vioData = provider->getVioDataByIndex(streamId, f);
        compare(vioData, sensorData.vioData());
        break;
      }
      case SensorDataType::VioHighFreq: {
        const auto vioHighFreqData = provider->getVioHighFreqDataByIndex(streamId, f);
        compare(vioHighFreqData, sensorData.vioHighFreqData());
        break;
      }
      case SensorDataType::EyeGaze: {
        const auto eyeGazeData = provider->getEyeGazeDataByIndex(streamId, f);
        compare(eyeGazeData, sensorData.eyeGazeData());
        break;
      }
      case SensorDataType::HandPose: {
        const auto handPoseData = provider->getHandPoseDataByIndex(streamId, f);
        compare(handPoseData, sensorData.handPoseData());
        break;
      }
      default:
        break;
    }
  }
}

// TODO: Add with BLU sequence.
TEST(VrsDataProvider, checkStreamDataCount) {
  for (const auto& dataPath :
       {ariaGen1TestDataPath, ariaGen2TestDataPath, ariaGen2TestDataPathWithBluetooth}) {
    auto provider = createVrsDataProvider(dataPath);

    const auto streamIds = provider->getAllStreams();
    for (const auto streamId : streamIds) {
      // Skipping Bluetooth and GPS stream because the unit test does not contain these data
      if (dataPath == ariaGen2TestDataPath &&
          (streamId.getTypeId() == vrs::RecordableTypeId::BluetoothBeaconRecordableClass ||
           streamId.getTypeId() == vrs::RecordableTypeId::GpsRecordableClass)) {
        continue;
      }
      // Skipping Wifi stream for Gen2 sequence with Bluetooth, which does not contain Wifi and GPS
      // data.
      else if (
          dataPath == ariaGen2TestDataPathWithBluetooth &&
          (streamId.getTypeId() == vrs::RecordableTypeId::WifiBeaconRecordableClass ||
           streamId.getTypeId() == vrs::RecordableTypeId::GpsRecordableClass)) {
        continue;
      }

      // Skipping ALS stream for Gen2 sequence with Bluetooth, which does not contain ALS data.
      else if (
          dataPath == ariaGen2TestDataPathWithBluetooth &&
          (streamId.getTypeId() == vrs::RecordableTypeId::AmbientLightRecordableClass)) {
        continue;
      }

      // Skipping Temperature stream for Gen2 sequence, which does not contain Temperature data.
      else if (
          dataPath == ariaGen2TestDataPath &&
          (streamId.getTypeId() == vrs::RecordableTypeId::TemperatureRecordableClass)) {
        continue;
      }
      auto numData = provider->getNumData(streamId);
      EXPECT_GT(numData, 0);
    }
  }
}

TEST(VrsDataProvider, getDataByIndex) {
  for (const auto& dataPath :
       {ariaGen1TestDataPath, ariaGen2TestDataPath, ariaGen2TestDataPathWithBluetooth}) {
    auto provider = createVrsDataProvider(dataPath);

    const auto streamIds = provider->getAllStreams();
    for (const auto streamId : streamIds) {
      checkGetDataByIndex(provider, streamId);
    }
  }
}

TEST(VrsDataProvider, multiThreadGetDataByIndex) {
  auto provider = createVrsDataProvider(ariaGen1TestDataPath);

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
