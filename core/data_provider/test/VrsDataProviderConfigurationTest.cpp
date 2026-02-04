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

#include <gtest/gtest.h>
#include <thread>

using namespace projectaria::tools::data_provider;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"

static const std::string ariaTestDataPath = XSTRING(TEST_FOLDER);

void checkSensorConfig(
    const std::shared_ptr<VrsDataProvider>& provider,
    const vrs::StreamId& streamId) {
  switch (provider->getSensorDataType(streamId)) {
    case SensorDataType::Image: {
      const auto imageConfig = provider->getImageConfiguration(streamId);
      EXPECT_GE(imageConfig.cameraId, 0);
      break;
    }
    case SensorDataType::Imu: {
      const auto imuConfig = provider->getImuConfiguration(streamId);
      EXPECT_GE(imuConfig.streamIndex, 0);
      break;
    }
    case SensorDataType::Gps: {
      const auto gpsConfig = provider->getGpsConfiguration(streamId);
      EXPECT_GE(gpsConfig.streamId, 0);
      break;
    }
    case SensorDataType::Wps: {
      const auto wpsConfig = provider->getWpsConfiguration(streamId);
      EXPECT_GE(wpsConfig.streamId, 0);
      break;
    }
    case SensorDataType::Audio: {
      const auto audioConfig = provider->getAudioConfiguration(streamId);
      EXPECT_GE(audioConfig.streamId, 0);
      break;
    }
    case SensorDataType::Bluetooth: {
      const auto bluetoothConfig = provider->getBluetoothConfiguration(streamId);
      EXPECT_GE(bluetoothConfig.streamId, 0);
      break;
    }
    case SensorDataType::Barometer: {
      const auto barometerConfig = provider->getBarometerConfiguration(streamId);
      EXPECT_GE(barometerConfig.streamId, 0);
      break;
    }
    case SensorDataType::Magnetometer: {
      const auto magnetometerConfig = provider->getMagnetometerConfiguration(streamId);
      EXPECT_GE(magnetometerConfig.streamIndex, 0);
      break;
    }
    default:
      break;
  }
}

TEST(VrsDataProvider, getConfigurations) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  const auto streamIds = provider->getAllStreams();
  for (const auto streamId : streamIds) {
    checkSensorConfig(provider, streamId);
  }
}

TEST(VrsDataProvider, multiThreadGetConfigurations) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  const auto streamIds = provider->getAllStreams();
  std::vector<std::thread> threads;
  for (const auto streamId : streamIds) {
    threads.emplace_back([&provider, streamId]() { checkSensorConfig(provider, streamId); });
  }

  for (auto& thread : threads) {
    thread.join();
  }
}
