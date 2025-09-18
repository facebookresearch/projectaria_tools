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
using namespace projectaria::tools::calibration;

#define STRING(x) #x
#define GEN1_STRING(x) std::string(STRING(x)) + "aria_unit_test_sequence_calib.vrs"
#define GEN2_STRING(x) std::string(STRING(x)) + "aria_gen2_unit_test_sequence.vrs"

static const std::string ariaGen1TestDataPath = GEN1_STRING(TEST_FOLDER);
static const std::string ariaGen2TestDataPath = GEN2_STRING(TEST_FOLDER_GEN2);

// A test util function to check DeviceCalibration content for a single VRS file, supports both Gen1
// and Gen2
void checkCalibrationsForSingleVrs(
    const std::string& dataPath,
    const DeviceVersion& expectedDeviceVersion) {
  fmt::print(
      "Testing device calibration for Aria Device version {}\n", getName(expectedDeviceVersion));

  auto provider = createVrsDataProvider(dataPath);
  auto maybeCalib = provider->getDeviceCalibration();
  EXPECT_TRUE(maybeCalib);

  auto calib = *maybeCalib;

  // Check for device version
  EXPECT_EQ(calib.getDeviceVersion(), expectedDeviceVersion);

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

// Test util functions to check camera rescaling. Gen1 and Gen2 has different ET checks.
void checkEyetrackingRescaleForGen1(
    const std::shared_ptr<VrsDataProvider> provider,
    const DeviceCalibration& deviceCalib) {
  static const vrs::StreamId kEyeCameraStreamId{vrs::RecordableTypeId::EyeCameraRecordableClass, 1};
  const auto eyeCameraLabel = provider->getLabelFromStreamId(kEyeCameraStreamId).value();
  // eyeCameraLabel = "camera-et", which should not contain calibrations
  // only "camera-et-left" and "camera-et-right" can obtain calibrations
  EXPECT_FALSE(deviceCalib.getCameraCalib(eyeCameraLabel).has_value());
  auto eyeCameraCalib = deviceCalib.getAriaEtCameraCalib().value();
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
void checkEyetrackingRescaleForGen2(
    const std::shared_ptr<VrsDataProvider> provider,
    const DeviceCalibration& deviceCalib) {
  static const vrs::StreamId kLeftEyeCameraStreamId{
      vrs::RecordableTypeId::EyeCameraRecordableClass, 1};
  static const vrs::StreamId kRightEyeCameraStreamId{
      vrs::RecordableTypeId::EyeCameraRecordableClass, 2};
  const auto leftCameraLabel = provider->getLabelFromStreamId(kLeftEyeCameraStreamId).value();
  const auto rightCameraLabel = provider->getLabelFromStreamId(kRightEyeCameraStreamId).value();

  // For Gen2, camera-et-left and camera-et-right should both exist
  const auto maybeLeftEtCalib = deviceCalib.getCameraCalib(leftCameraLabel);
  const auto maybeRightEtCalib = deviceCalib.getCameraCalib(rightCameraLabel);
  EXPECT_TRUE(maybeLeftEtCalib.has_value());
  EXPECT_TRUE(maybeRightEtCalib.has_value());
  auto eyeCameraCalib = deviceCalib.getAriaEtCameraCalib().value();

  // Check calibration consistency for joint et calib
  EXPECT_NEAR(
      eyeCameraCalib[0].projectionParams()[0],
      maybeLeftEtCalib.value().projectionParams()[0],
      1e-3);
  EXPECT_NEAR(
      eyeCameraCalib[1].projectionParams()[0],
      maybeRightEtCalib.value().projectionParams()[0],
      1e-3);

  // TODO: Re-enable this when Gen2 unit test sequence contains valid ET stream
  // Stream image resolution and camera resolution should match
  /*
  EXPECT_EQ(
      eyeCameraCalib[0].getImageSize().x(),
      provider->getConfiguration(kLeftEyeCameraStreamId).imageConfiguration().imageWidth / 2);
  EXPECT_EQ(
      eyeCameraCalib[0].getImageSize().y(),
      provider->getConfiguration(kLeftEyeCameraStreamId).imageConfiguration().imageHeight);
  EXPECT_EQ(
      eyeCameraCalib[1].getImageSize().x(),
      provider->getConfiguration(kRightEyeCameraStreamId).imageConfiguration().imageWidth / 2);
  EXPECT_EQ(
      eyeCameraCalib[1].getImageSize().y(),
      provider->getConfiguration(kRightEyeCameraStreamId).imageConfiguration().imageHeight);
  */
}
void checkRescaleForSingleVrs(const std::string& dataPath, const DeviceVersion& deviceVersion) {
  fmt::print("Testing calibration rescalingg for Aria Device version {}\n", getName(deviceVersion));

  auto provider = createVrsDataProvider(dataPath);
  auto maybeCalib = provider->getDeviceCalibration();
  EXPECT_TRUE(maybeCalib);

  /********** Check for RGB camera ************/
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

  /********** Check for ET camera ************/
  switch (deviceVersion) {
    case DeviceVersion::Gen1: {
      checkEyetrackingRescaleForGen1(provider, maybeCalib.value());
      break;
    }
    case DeviceVersion::Gen2: {
      checkEyetrackingRescaleForGen2(provider, maybeCalib.value());
      break;
    }
    default:
      break;
  }
}

// Test util function to check stream id label mappings
void checkStreamIdLabelMapping(const std::string& dataPath) {
  auto provider = createVrsDataProvider(dataPath);
  const auto streamIds = provider->getAllStreams();
  for (const auto& streamId : streamIds) {
    auto maybeLabel = provider->getLabelFromStreamId(streamId);
    EXPECT_TRUE(maybeLabel);
    auto maybeStreamId = provider->getStreamIdFromLabel(*maybeLabel);
    EXPECT_TRUE(maybeStreamId);
    EXPECT_EQ(*maybeStreamId, streamId);
  }
}

TEST(VrsDataProvider, getCalibrations) {
  // Test Gen1
  checkCalibrationsForSingleVrs(ariaGen1TestDataPath, DeviceVersion::Gen1);

  // Test Gen2
  checkCalibrationsForSingleVrs(ariaGen2TestDataPath, DeviceVersion::Gen2);
}

TEST(VrsDataProvider, rescaledCalibration) {
  // Test Gen1
  checkRescaleForSingleVrs(ariaGen1TestDataPath, DeviceVersion::Gen1);

  // Test Gen2
  checkRescaleForSingleVrs(ariaGen2TestDataPath, DeviceVersion::Gen2);
}

TEST(VrsDataProvider, streamIdToLabelMapping) {
  // Test Gen1
  checkStreamIdLabelMapping(ariaGen1TestDataPath);

  // Test Gen2
  checkStreamIdLabelMapping(ariaGen2TestDataPath);
}
