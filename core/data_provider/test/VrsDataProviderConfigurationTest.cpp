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
#define GEN1_STRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"
#define GEN2_STRING(x) std::string(STRING(x)) + "aria_gen2_unit_test_sequence.vrs"

static const std::string ariaGen1TestDataPath = GEN1_STRING(TEST_FOLDER);
static const std::string ariaGen2TestDataPath = GEN2_STRING(TEST_FOLDER_GEN2);

void checkSensorConfig(
    const std::shared_ptr<VrsDataProvider>& provider,
    const vrs::StreamId& streamId) {
  // First, check the API of getConfiguration to ensure it returns the correct type
  const auto sensorConfig = provider->getConfiguration(streamId);
  EXPECT_EQ(sensorConfig.sensorDataType(), provider->getSensorDataType(streamId));

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
    case SensorDataType::Ppg: {
      const auto ppgConfig = provider->getPpgConfiguration(streamId);
      EXPECT_GE(ppgConfig.streamId, 0);
      break;
    }
    case SensorDataType::EyeGaze: {
      const auto eyeGazeConfig = provider->getEyeGazeConfiguration(streamId);
      EXPECT_GE(eyeGazeConfig.streamId, 0);
      break;
    }
    case SensorDataType::HandPose: {
      const auto handPoseConfig = provider->getHandPoseConfiguration(streamId);
      EXPECT_GE(handPoseConfig.streamId, 0);
      break;
    }
    case SensorDataType::Vio: {
      const auto vioConfig = provider->getVioConfiguration(streamId);
      EXPECT_GE(vioConfig.streamId, 0);
      break;
    }
    case SensorDataType::VioHighFreq: {
      const auto vioHighFreqConfig = provider->getVioHighFreqConfiguration(streamId);
      EXPECT_GE(vioHighFreqConfig.streamId, 0);
      break;
    }
    default:
      break;
  }
}

TEST(VrsDataProvider, getConfigurations) {
  for (const auto& dataPath : {ariaGen1TestDataPath, ariaGen2TestDataPath}) {
    auto provider = createVrsDataProvider(dataPath);
    const auto streamIds = provider->getAllStreams();
    for (const auto streamId : streamIds) {
      checkSensorConfig(provider, streamId);
    }
  }
}

TEST(VrsDataProvider, multiThreadGetConfigurations) {
  for (const auto& dataPath : {ariaGen1TestDataPath, ariaGen2TestDataPath}) {
    auto provider = createVrsDataProvider(dataPath);
    const auto streamIds = provider->getAllStreams();
    std::vector<std::thread> threads;
    threads.reserve(streamIds.size());
    for (const auto streamId : streamIds) {
      threads.emplace_back([&provider, streamId]() { checkSensorConfig(provider, streamId); });
    }

    for (auto& thread : threads) {
      thread.join();
    }
  }
}
