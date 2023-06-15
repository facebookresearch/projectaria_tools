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

using namespace projectaria::tools::data_provider;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"

static const std::string ariaTestDataPath = XSTRING(TEST_FOLDER);

TEST(VrsDataProvider, getCalibrations) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  auto maybeCalib = provider->getDeviceCalibration();
  EXPECT_TRUE(maybeCalib);

  auto calib = *maybeCalib;

  for (const auto& label : calib.getCameraLabels()) {
    auto camCalib = calib.getCameraCalib(label);
    EXPECT_TRUE(camCalib);
  }

  for (const auto& label : calib.getImuLabels()) {
    auto imuCalib = calib.getImuCalib(label);
    EXPECT_TRUE(imuCalib);
  }

  for (const auto& label : calib.getMagnetometerLabels()) {
    auto magCalib = calib.getMagnetometerCalib(label);
    EXPECT_TRUE(magCalib);
  }

  for (const auto& label : calib.getBarometerLabels()) {
    auto baroCalib = calib.getBarometerCalib(label);
    EXPECT_TRUE(baroCalib);
  }

  auto failedEtCalib = calib.getCameraCalib("camera-et");
  EXPECT_FALSE(failedEtCalib);

  auto etCalib = calib.getAriaEtCameraCalib();
  EXPECT_TRUE(etCalib);

  auto micCalib = calib.getAriaMicrophoneCalib();
  EXPECT_TRUE(micCalib);

  for (const auto& label : calib.getAllLabels()) {
    auto sensorCalib = calib.getSensorCalib(label);
    EXPECT_TRUE(sensorCalib);
  }

  for (const auto& streamId : provider->getAllStreams()) {
    if (!hasCalibration(provider->getSensorDataType(streamId))) {
      continue;
    }
    auto maybeLabel = provider->getLabelFromStreamId(streamId);
    auto maybeSensorCalib = provider->getSensorCalibration(streamId);
    EXPECT_EQ(maybeLabel.has_value(), maybeSensorCalib.has_value());
  }
}

TEST(VrsDataProvider, rescaledCalibration) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  auto maybeCalib = provider->getDeviceCalibration();
  EXPECT_TRUE(maybeCalib);

  static const vrs::StreamId kRgbCameraStreamId{vrs::RecordableTypeId::RgbCameraRecordableClass, 1};
  const auto rgbStreamIdLabel = provider->getLabelFromStreamId(kRgbCameraStreamId).value();
  const auto rgbCameraCalib = maybeCalib.value().getCameraCalib(rgbStreamIdLabel).value();
  // Stream image resolution and camera resolution should match
  EXPECT_EQ(
      rgbCameraCalib.getImageSize().x(),
      provider->getConfiguration(kRgbCameraStreamId).imageConfiguration().imageWidth);
  EXPECT_EQ(
      rgbCameraCalib.getImageSize().y(),
      provider->getConfiguration(kRgbCameraStreamId).imageConfiguration().imageHeight);

  // Note sensor calibration can be different (since not updated to VRS data content)
  // i.e
  // auto rgbSensorCalib = provider->getSensorCalibration(kRgbCameraStreamId)->cameraCalibration();
  // rgbSensorCalib.getImageSize() could be different than stream image resolution
}

TEST(VrsDataProvider, streamIdToLabelMapping) {
  auto provider = createVrsDataProvider(ariaTestDataPath);
  const auto streamIds = provider->getAllStreams();
  for (const auto& streamId : streamIds) {
    auto maybeLabel = provider->getLabelFromStreamId(streamId);
    EXPECT_TRUE(maybeLabel);
    auto maybeStreamId = provider->getStreamIdFromLabel(*maybeLabel);
    EXPECT_TRUE(maybeStreamId);
    EXPECT_EQ(*maybeStreamId, streamId);
  }
}
