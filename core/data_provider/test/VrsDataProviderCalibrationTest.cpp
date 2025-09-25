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
  auto rgbSensorCalib = provider->getSensorCalibration(kRgbCameraStreamId)->cameraCalibration();

  // Stream image resolution and camera resolution should match
  EXPECT_EQ(
      rgbCameraCalib.getImageSize().x(),
      provider->getConfiguration(kRgbCameraStreamId).imageConfiguration().imageWidth);
  EXPECT_EQ(
      rgbCameraCalib.getImageSize().y(),
      provider->getConfiguration(kRgbCameraStreamId).imageConfiguration().imageHeight);

  // sensor calib should match device calib
  EXPECT_EQ(rgbCameraCalib.getImageSize().x(), rgbSensorCalib.getImageSize().x());
  EXPECT_EQ(rgbCameraCalib.getImageSize().y(), rgbSensorCalib.getImageSize().y());

  // Check eye tracking camera scale
  static const vrs::StreamId kEyeCameraStreamId{vrs::RecordableTypeId::EyeCameraRecordableClass, 1};
  const auto eyeCameraLabel = provider->getLabelFromStreamId(kEyeCameraStreamId).value();
  // eyeCameraLabel = "camera-et", which should not contain calibrations
  // only "camera-et-left" and "camera-et-right" can obtain calibrations
  EXPECT_FALSE(maybeCalib.value().getCameraCalib(eyeCameraLabel).has_value());
  auto eyeCameraCalib = maybeCalib.value().getAriaEtCameraCalib().value();
  auto etSensorCalib = provider->getSensorCalibration(kEyeCameraStreamId)->ariaEtCalibration();

  // Stream image resolution and camera resolution should match
  EXPECT_EQ(
      eyeCameraCalib[0].getImageSize().x(),
      provider->getConfiguration(kEyeCameraStreamId).imageConfiguration().imageWidth / 2);
  EXPECT_EQ(
      eyeCameraCalib[0].getImageSize().y(),
      provider->getConfiguration(kEyeCameraStreamId).imageConfiguration().imageHeight);
  EXPECT_EQ(
      eyeCameraCalib[1].getImageSize().x(),
      provider->getConfiguration(kEyeCameraStreamId).imageConfiguration().imageWidth / 2);
  EXPECT_EQ(
      eyeCameraCalib[1].getImageSize().y(),
      provider->getConfiguration(kEyeCameraStreamId).imageConfiguration().imageHeight);

  // sensor calib should match device calib
  EXPECT_EQ(etSensorCalib[0].getImageSize().x(), eyeCameraCalib[0].getImageSize().x());
  EXPECT_EQ(etSensorCalib[0].getImageSize().y(), eyeCameraCalib[0].getImageSize().y());
  EXPECT_EQ(etSensorCalib[1].getImageSize().x(), eyeCameraCalib[1].getImageSize().x());
  EXPECT_EQ(etSensorCalib[1].getImageSize().y(), eyeCameraCalib[1].getImageSize().y());
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
