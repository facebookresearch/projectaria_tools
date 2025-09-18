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

#pragma once

#include <data_provider/VrsDataProvider.h>
#include <data_provider/test/CompareDataUtils.h>
#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>
#include <mps/HandTracking.h>
#include <mps/Trajectory.h>

namespace projectaria::tools::data_provider::test {
using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::mps;
inline void compare(
    const ImageDataAndRecord& imageDataAndRecord1,
    const ImageDataAndRecord& imageDataAndRecord2) {
  EXPECT_EQ(
      imageDataAndRecord1.first.pixelFrame->getSpec(),
      imageDataAndRecord2.first.pixelFrame->getSpec());
  EXPECT_EQ(imageDataAndRecord1.second.cameraId, imageDataAndRecord2.second.cameraId);
  EXPECT_EQ(
      imageDataAndRecord1.second.arrivalTimestampNs, imageDataAndRecord2.second.arrivalTimestampNs);
  EXPECT_EQ(
      imageDataAndRecord1.second.captureTimestampNs, imageDataAndRecord2.second.captureTimestampNs);
  EXPECT_EQ(
      imageDataAndRecord1.second.exposureDuration, imageDataAndRecord2.second.exposureDuration);
  EXPECT_EQ(imageDataAndRecord1.second.frameNumber, imageDataAndRecord2.second.frameNumber);
  EXPECT_EQ(imageDataAndRecord1.second.gain, imageDataAndRecord2.second.gain);
  EXPECT_EQ(imageDataAndRecord1.second.temperature, imageDataAndRecord2.second.temperature);
  EXPECT_EQ(imageDataAndRecord1.second.groupId, imageDataAndRecord2.second.groupId);
  EXPECT_EQ(imageDataAndRecord1.second.groupMask, imageDataAndRecord2.second.groupMask);
  EXPECT_THAT(
      imageDataAndRecord1.second.temperature,
      testing::NanSensitiveDoubleEq(imageDataAndRecord2.second.temperature));
}

inline void compare(const MotionData& motionData1, const MotionData& motionData2) {
  EXPECT_EQ(motionData1.accelValid, motionData2.accelValid);
  EXPECT_EQ(motionData1.gyroValid, motionData2.gyroValid);
  EXPECT_EQ(motionData1.magValid, motionData2.magValid);
  EXPECT_TRUE(
      (std::isnan(motionData1.temperature) && std::isnan(motionData2.temperature)) ||
      motionData1.temperature == motionData2.temperature);
  EXPECT_EQ(motionData1.captureTimestampNs, motionData2.captureTimestampNs);
  EXPECT_EQ(motionData1.arrivalTimestampNs, motionData2.arrivalTimestampNs);
  EXPECT_EQ(motionData1.accelMSec2, motionData2.accelMSec2);
  EXPECT_EQ(motionData1.gyroRadSec, motionData2.gyroRadSec);
  EXPECT_EQ(motionData1.magTesla, motionData2.magTesla);
}

inline void compare(const GpsData& gpsData1, const GpsData& gpsData2) {
  EXPECT_EQ(gpsData1.captureTimestampNs, gpsData2.captureTimestampNs);
  EXPECT_EQ(gpsData1.utcTimeMs, gpsData2.utcTimeMs);
  EXPECT_EQ(gpsData1.provider, gpsData2.provider);
  EXPECT_EQ(gpsData1.latitude, gpsData2.latitude);
  EXPECT_EQ(gpsData1.longitude, gpsData2.longitude);
  EXPECT_EQ(gpsData1.altitude, gpsData2.altitude);
  EXPECT_EQ(gpsData1.accuracy, gpsData2.accuracy);
  EXPECT_EQ(gpsData1.speed, gpsData2.speed);
  EXPECT_EQ(gpsData1.verticalAccuracy, gpsData2.verticalAccuracy);

  // Compare raw data
  EXPECT_EQ(gpsData1.rawData.size(), gpsData2.rawData.size());
  for (size_t i = 0; i < gpsData1.rawData.size(); ++i) {
    EXPECT_EQ(gpsData1.rawData[i], gpsData2.rawData[i]);
  }
}

inline void compare(const GpsConfigRecord& gpsConfig1, const GpsConfigRecord& gpsConfig2) {
  EXPECT_EQ(gpsConfig1.streamId, gpsConfig2.streamId);
  EXPECT_EQ(gpsConfig1.sampleRateHz, gpsConfig2.sampleRateHz);
}

inline void compare(const WifiBeaconData& wpsData1, const WifiBeaconData& wpsData2) {
  EXPECT_EQ(wpsData1.systemTimestampNs, wpsData2.systemTimestampNs);
  EXPECT_EQ(wpsData1.boardTimestampNs, wpsData2.boardTimestampNs);
  EXPECT_EQ(wpsData1.boardScanRequestStartTimestampNs, wpsData2.boardScanRequestStartTimestampNs);
  EXPECT_EQ(
      wpsData1.boardScanRequestCompleteTimestampNs, wpsData2.boardScanRequestCompleteTimestampNs);
  EXPECT_EQ(wpsData1.ssid, wpsData2.ssid);
  EXPECT_EQ(wpsData1.bssidMac, wpsData2.bssidMac);
  EXPECT_EQ(wpsData1.rssi, wpsData2.rssi);
  EXPECT_EQ(wpsData1.freqMhz, wpsData2.freqMhz);

  // Compare rssiPerAntenna vector
  EXPECT_EQ(wpsData1.rssiPerAntenna.size(), wpsData2.rssiPerAntenna.size());
  for (size_t i = 0; i < wpsData1.rssiPerAntenna.size(); ++i) {
    EXPECT_EQ(wpsData1.rssiPerAntenna[i], wpsData2.rssiPerAntenna[i]);
  }
}

inline void compare(const AudioDataAndRecord& audioData1, const AudioDataAndRecord& audioData2) {
  EXPECT_EQ(audioData1.first.data, audioData2.first.data);
  EXPECT_EQ(audioData1.second.captureTimestampsNs, audioData2.second.captureTimestampsNs);
  EXPECT_EQ(audioData1.second.audioMuted, audioData2.second.audioMuted);
}

inline void compare(const AudioConfig& audioConfig1, const AudioConfig& audioConfig2) {
  EXPECT_EQ(audioConfig1.streamId, audioConfig2.streamId);
  EXPECT_EQ(audioConfig1.numChannels, audioConfig2.numChannels);
  EXPECT_EQ(audioConfig1.sampleRate, audioConfig2.sampleRate);
  EXPECT_EQ(audioConfig1.sampleFormat, audioConfig2.sampleFormat);
}

inline void compare(
    const BluetoothBeaconData& bluetoothData1,
    const BluetoothBeaconData& bluetoothData2) {
  EXPECT_EQ(bluetoothData1.systemTimestampNs, bluetoothData2.systemTimestampNs);
  EXPECT_EQ(bluetoothData1.boardTimestampNs, bluetoothData2.boardTimestampNs);
  EXPECT_EQ(
      bluetoothData1.boardScanRequestStartTimestampNs,
      bluetoothData2.boardScanRequestStartTimestampNs);
  EXPECT_EQ(
      bluetoothData1.boardScanRequestCompleteTimestampNs,
      bluetoothData2.boardScanRequestCompleteTimestampNs);
  EXPECT_EQ(bluetoothData1.uniqueId, bluetoothData2.uniqueId);
  if (!(std::isnan(bluetoothData1.txPower) && std::isnan(bluetoothData2.txPower))) {
    EXPECT_EQ(bluetoothData1.txPower, bluetoothData2.txPower);
  } // TODO: remove this after txPower and freqMhz is available.
  EXPECT_EQ(bluetoothData1.rssi, bluetoothData2.rssi);
  if (!(std::isnan(bluetoothData1.freqMhz) && std::isnan(bluetoothData2.freqMhz))) {
    EXPECT_EQ(bluetoothData1.freqMhz, bluetoothData2.freqMhz);
  }
}

inline void compare(const BarometerData& barometerData1, const BarometerData& barometerData2) {
  EXPECT_EQ(barometerData1.captureTimestampNs, barometerData2.captureTimestampNs);
  EXPECT_EQ(barometerData1.pressure, barometerData2.pressure);
  EXPECT_EQ(barometerData1.temperature, barometerData2.temperature);
}

inline void compare(
    const BarometerConfigRecord& barometerConfig1,
    const BarometerConfigRecord& barometerConfig2) {
  EXPECT_EQ(barometerConfig1.streamId, barometerConfig2.streamId);
  EXPECT_EQ(barometerConfig1.sensorModelName, barometerConfig2.sensorModelName);
  EXPECT_DOUBLE_EQ(barometerConfig1.sampleRate, barometerConfig2.sampleRate);
}

inline void compare(const BatteryStatusData& batteryData1, const BatteryStatusData& batteryData2) {
  EXPECT_EQ(batteryData1.captureTimestampNs, batteryData2.captureTimestampNs);
  EXPECT_EQ(batteryData1.chargingStatus, batteryData2.chargingStatus);
  EXPECT_EQ(batteryData1.batteryLevel, batteryData2.batteryLevel);
  EXPECT_FLOAT_EQ(batteryData1.temperatureC, batteryData2.temperatureC);
  EXPECT_FLOAT_EQ(batteryData1.voltageVolt, batteryData2.voltageVolt);
  EXPECT_FLOAT_EQ(batteryData1.voltageAvgVolt, batteryData2.voltageAvgVolt);
  EXPECT_FLOAT_EQ(batteryData1.currentAmp, batteryData2.currentAmp);
  EXPECT_FLOAT_EQ(batteryData1.currentAvgAmp, batteryData2.currentAvgAmp);
  EXPECT_FLOAT_EQ(batteryData1.powerWatt, batteryData2.powerWatt);
  EXPECT_FLOAT_EQ(batteryData1.powerAvgWatt, batteryData2.powerAvgWatt);
}

inline void compare(
    const BatteryStatusConfiguration& batteryConfig1,
    const BatteryStatusConfiguration& batteryConfig2) {
  EXPECT_EQ(batteryConfig1.streamId, batteryConfig2.streamId);
  EXPECT_EQ(batteryConfig1.sensorModel, batteryConfig2.sensorModel);
  EXPECT_EQ(batteryConfig1.deviceId, batteryConfig2.deviceId);
  EXPECT_DOUBLE_EQ(batteryConfig1.nominalRateHz, batteryConfig2.nominalRateHz);
  EXPECT_EQ(batteryConfig1.description, batteryConfig2.description);
}

inline void compare(
    const BluetoothBeaconConfigRecord& bluetoothConfig1,
    const BluetoothBeaconConfigRecord& bluetoothConfig2) {
  EXPECT_EQ(bluetoothConfig1.streamId, bluetoothConfig2.streamId);
  EXPECT_DOUBLE_EQ(bluetoothConfig1.sampleRateHz, bluetoothConfig2.sampleRateHz);
}

inline void compare(const PpgConfiguration& ppgConfig1, const PpgConfiguration& ppgConfig2) {
  EXPECT_EQ(ppgConfig1.streamId, ppgConfig2.streamId);
  EXPECT_EQ(ppgConfig1.sensorModel, ppgConfig2.sensorModel);
  EXPECT_EQ(ppgConfig1.deviceId, ppgConfig2.deviceId);
  EXPECT_DOUBLE_EQ(ppgConfig1.nominalRateHz, ppgConfig2.nominalRateHz);
  EXPECT_EQ(ppgConfig1.description, ppgConfig2.description);
}

inline void compare(const PpgData& ppgData1, const PpgData& ppgData2) {
  EXPECT_EQ(ppgData1.captureTimestampNs, ppgData2.captureTimestampNs);
  EXPECT_EQ(ppgData1.value, ppgData2.value);
  EXPECT_EQ(ppgData1.ledCurrentMa, ppgData2.ledCurrentMa);
  EXPECT_EQ(ppgData1.integrationTimeUs, ppgData2.integrationTimeUs);
}

inline void compare(const AlsConfiguration& alsConfig1, const AlsConfiguration& alsConfig2) {
  EXPECT_EQ(alsConfig1.streamId, alsConfig2.streamId);
  EXPECT_EQ(alsConfig1.deviceId, alsConfig2.deviceId);
  EXPECT_DOUBLE_EQ(alsConfig1.nominalRateHz, alsConfig2.nominalRateHz);
  EXPECT_EQ(alsConfig1.sensorModel, alsConfig2.sensorModel);
}

inline void compare(const AlsData& alsData1, const AlsData& alsData2) {
  EXPECT_EQ(alsData1.captureTimestampNs, alsData2.captureTimestampNs);
  EXPECT_FLOAT_EQ(alsData1.redChannelNormalized, alsData2.redChannelNormalized);
  EXPECT_FLOAT_EQ(alsData1.greenChannelNormalized, alsData2.greenChannelNormalized);
  EXPECT_FLOAT_EQ(alsData1.blueChannelNormalized, alsData2.blueChannelNormalized);
  EXPECT_FLOAT_EQ(alsData1.uvChannelNormalized, alsData2.uvChannelNormalized);
  EXPECT_FLOAT_EQ(alsData1.irChannelNormalized, alsData2.irChannelNormalized);
  EXPECT_FLOAT_EQ(alsData1.clearChannelNormalized, alsData2.clearChannelNormalized);
  EXPECT_FLOAT_EQ(alsData1.uvFluxWattPerSquareMeter, alsData2.uvFluxWattPerSquareMeter);
  EXPECT_FLOAT_EQ(alsData1.irFluxWattPerSquareMeter, alsData2.irFluxWattPerSquareMeter);
  EXPECT_FLOAT_EQ(alsData1.clearFluxWattPerSquareMeter, alsData2.clearFluxWattPerSquareMeter);
  EXPECT_EQ(alsData1.gainRed, alsData2.gainRed);
  EXPECT_EQ(alsData1.gainGreen, alsData2.gainGreen);
  EXPECT_EQ(alsData1.gainBlue, alsData2.gainBlue);
  EXPECT_EQ(alsData1.gainUv, alsData2.gainUv);
  EXPECT_EQ(alsData1.gainIr, alsData2.gainIr);
  EXPECT_EQ(alsData1.gainClear, alsData2.gainClear);
  EXPECT_EQ(alsData1.exposureTimeUs, alsData2.exposureTimeUs);
  EXPECT_FLOAT_EQ(alsData1.cct, alsData2.cct);
  EXPECT_FLOAT_EQ(alsData1.lux, alsData2.lux);
}

inline void compare(const TemperatureData& tempData1, const TemperatureData& tempData2) {
  EXPECT_EQ(tempData1.captureTimestampNs, tempData2.captureTimestampNs);
  EXPECT_EQ(tempData1.temperatureCelsius, tempData2.temperatureCelsius);
  EXPECT_EQ(tempData1.sensorName, tempData2.sensorName);
}

inline void compare(
    const VioHighFreqConfiguration& config1,
    const VioHighFreqConfiguration& config2) {
  EXPECT_EQ(config1.streamId, config2.streamId);
  EXPECT_DOUBLE_EQ(config1.nominalRateHz, config2.nominalRateHz);
  EXPECT_EQ(config1.messageVersion, config2.messageVersion);
}

inline void compare(const VioConfiguration& config1, const VioConfiguration& config2) {
  EXPECT_EQ(config1.streamId, config2.streamId);
  EXPECT_DOUBLE_EQ(config1.nominalRateHz, config2.nominalRateHz);
  EXPECT_EQ(config1.messageVersion, config2.messageVersion);
}

inline void compare(
    const OnDeviceVioHighFreqData& pose1,
    const OnDeviceVioHighFreqData& pose2,
    float tolerance = 1e-5f) {
  // Compare timestamps
  EXPECT_EQ(pose1.trackingTimestamp.count(), pose2.trackingTimestamp.count());
  EXPECT_EQ(pose1.utcTimestamp.count(), pose2.utcTimestamp.count());

  // Compare session UID
  EXPECT_EQ(pose1.sessionUid, pose2.sessionUid);

  // Compare quality score
  EXPECT_FLOAT_EQ(pose1.qualityScore, pose2.qualityScore);

  // Compare pose
  EXPECT_TRUE(pose1.T_odometry_device.translation().isApprox(
      pose2.T_odometry_device.translation(), tolerance));

  EXPECT_TRUE(compareQuaternion<double>(
      pose1.T_odometry_device.unit_quaternion(),
      pose2.T_odometry_device.unit_quaternion(),
      tolerance));

  // Compare velocities
  EXPECT_TRUE(
      pose1.deviceLinearVelocity_odometry.isApprox(pose2.deviceLinearVelocity_odometry, tolerance));
  EXPECT_TRUE(pose1.angularVelocity_device.isApprox(pose2.angularVelocity_device, tolerance));

  // Compare gravity vector
  EXPECT_TRUE(
      comparePtr3<double>(pose1.gravity_odometry.data(), pose2.gravity_odometry.data(), tolerance));
}

inline void compare(
    const projectaria::tools::data_provider::FrontendOutput& output1,
    const projectaria::tools::data_provider::FrontendOutput& output2) {
  // Compare frontend session UUID
  EXPECT_EQ(output1.frontendSessionUid, output2.frontendSessionUid);

  // Compare frame ID
  EXPECT_EQ(output1.frameID, output2.frameID);

  // Compare timestamps
  EXPECT_EQ(output1.captureTimestampNs, output2.captureTimestampNs);
  // skip comparing unixTimestampNs as it is not available in the oatmeal data type. Therefore the
  // info is lost during the conversion
  // EXPECT_EQ(output1.unixTimestampNs, output2.unixTimestampNs);

  // Compare status
  EXPECT_EQ(output1.status, output2.status);
  EXPECT_EQ(output1.poseQuality, output2.poseQuality);
  EXPECT_EQ(output1.visualTrackingQuality, output2.visualTrackingQuality);

  // Compare online calibration state
  compareOnlineCalibState(output1.onlineCalib, output2.onlineCalib);

  // Compare camera serials: This part is currently skipped because camera serials are not available
  // in the OSS type, leading to information loss during conversion.
  // EXPECT_EQ(output1.cameraSerials.size(), output2.cameraSerials.size());
  // for (size_t i = 0; i < output1.cameraSerials.size(); ++i) {
  //   EXPECT_EQ(output1.cameraSerials[i], output2.cameraSerials[i]);
  // }

  // Compare gravity vector
  EXPECT_EQ(output1.gravityInOdometry, output2.gravityInOdometry);

  // Compare pose: T_Odometry_BodyImu
  EXPECT_EQ(output1.T_Odometry_BodyImu.translation(), output2.T_Odometry_BodyImu.translation());
  EXPECT_EQ(
      output1.T_Odometry_BodyImu.unit_quaternion().coeffs(),
      output2.T_Odometry_BodyImu.unit_quaternion().coeffs());

  // Compare pose: T_BodyImu_Device
  EXPECT_EQ(output1.T_BodyImu_Device.translation(), output2.T_BodyImu_Device.translation());
  EXPECT_EQ(
      output1.T_BodyImu_Device.unit_quaternion().coeffs(),
      output2.T_BodyImu_Device.unit_quaternion().coeffs());

  // Compare velocities
  EXPECT_EQ(output1.linearVelocityInOdometry, output2.linearVelocityInOdometry);
  EXPECT_EQ(output1.angularVelocityInBodyImu, output2.angularVelocityInBodyImu);
}

inline void compare(const HandPoseConfiguration& config1, const HandPoseConfiguration& config2) {
  EXPECT_EQ(config1.streamId, config2.streamId);
  EXPECT_DOUBLE_EQ(config1.nominalRateHz, config2.nominalRateHz);
  EXPECT_EQ(config1.isWristPalmOnly, config2.isWristPalmOnly);
  EXPECT_EQ(config1.userProfile, config2.userProfile);
}

inline void compare(
    const HandTrackingResult::OneSide& hand1,
    const HandTrackingResult::OneSide& hand2,
    double tolerance = 1e-5) {
  // Compare confidence
  EXPECT_EQ(hand1.confidence, hand2.confidence);

  // Compare wrist pose
  EXPECT_TRUE(
      hand1.T_Device_Wrist.translation().isApprox(hand2.T_Device_Wrist.translation(), tolerance));
  EXPECT_TRUE(compareQuaternion<double>(
      hand1.T_Device_Wrist.unit_quaternion(), hand2.T_Device_Wrist.unit_quaternion(), tolerance));

  // Compare landmarks
  for (int i = 0; i < kNumHandLandmarks; ++i) {
    EXPECT_EQ(hand1.landmarkPositions_device[i], hand2.landmarkPositions_device[i]);
  }

  // Compare wrist and palm normals if available
  if (hand1.wristAndPalmNormal_device.has_value() && hand2.wristAndPalmNormal_device.has_value()) {
    EXPECT_TRUE(hand1.wristAndPalmNormal_device->wristNormal_device.isApprox(
        hand2.wristAndPalmNormal_device->wristNormal_device, tolerance));
    EXPECT_TRUE(hand1.wristAndPalmNormal_device->palmNormal_device.isApprox(
        hand2.wristAndPalmNormal_device->palmNormal_device, tolerance));
  } else {
    EXPECT_EQ(
        hand1.wristAndPalmNormal_device.has_value(), hand2.wristAndPalmNormal_device.has_value());
  }
}

inline void compare(
    const HandTrackingResult& result1,
    const HandTrackingResult& result2,
    double tolerance = 1e-5) {
  // Compare timestamp
  EXPECT_EQ(result1.trackingTimestamp.count(), result2.trackingTimestamp.count());

  // Compare left hand if available
  if (result1.leftHand.has_value() && result2.leftHand.has_value()) {
    compare(result1.leftHand.value(), result2.leftHand.value(), tolerance);
  } else {
    EXPECT_EQ(result1.leftHand.has_value(), result2.leftHand.has_value());
  }

  // Compare right hand if available
  if (result1.rightHand.has_value() && result2.rightHand.has_value()) {
    compare(result1.rightHand.value(), result2.rightHand.value(), tolerance);
  } else {
    EXPECT_EQ(result1.rightHand.has_value(), result2.rightHand.has_value());
  }
}

inline void compare(
    const EyeGazeConfiguration& eyeGazeConfig1,
    const EyeGazeConfiguration& eyeGazeConfig2) {
  EXPECT_EQ(eyeGazeConfig1.streamId, eyeGazeConfig2.streamId);
  EXPECT_DOUBLE_EQ(eyeGazeConfig1.nominalRateHz, eyeGazeConfig2.nominalRateHz);
  EXPECT_EQ(eyeGazeConfig1.userCalibrated, eyeGazeConfig2.userCalibrated);
  EXPECT_FLOAT_EQ(eyeGazeConfig1.userCalibrationError, eyeGazeConfig2.userCalibrationError);
}

inline void compare(
    const MotionConfigRecord& motionConfig1,
    const MotionConfigRecord& motionConfig2) {
  // Skip streamIndex as it is not stored in the internal type, losing information during
  // conversion. EXPECT_EQ(motionConfig1.streamIndex, motionConfig2.streamIndex);
  EXPECT_EQ(motionConfig1.deviceType, motionConfig2.deviceType);
  EXPECT_EQ(motionConfig1.deviceVersion, motionConfig2.deviceVersion);
  EXPECT_EQ(motionConfig1.deviceSerial, motionConfig2.deviceSerial);
  EXPECT_EQ(motionConfig1.deviceId, motionConfig2.deviceId);
  EXPECT_EQ(motionConfig1.sensorModel, motionConfig2.sensorModel);
  EXPECT_DOUBLE_EQ(motionConfig1.nominalRateHz, motionConfig2.nominalRateHz);
  EXPECT_EQ(motionConfig1.hasAccelerometer, motionConfig2.hasAccelerometer);
  EXPECT_EQ(motionConfig1.hasGyroscope, motionConfig2.hasGyroscope);
  EXPECT_EQ(motionConfig1.hasMagnetometer, motionConfig2.hasMagnetometer);
  EXPECT_EQ(motionConfig1.factoryCalibration, motionConfig2.factoryCalibration);
  EXPECT_EQ(motionConfig1.onlineCalibration, motionConfig2.onlineCalibration);
  EXPECT_EQ(motionConfig1.description, motionConfig2.description);
}

inline void compare(
    const OnDeviceEyeGazeData& eyeGaze1,
    const OnDeviceEyeGazeData& eyeGaze2,
    float tolerance = 1e-5f) {
  EXPECT_EQ(eyeGaze1.trackingTimestamp, eyeGaze2.trackingTimestamp);
  EXPECT_NEAR(eyeGaze1.yaw, eyeGaze2.yaw, tolerance);
  EXPECT_NEAR(eyeGaze1.pitch, eyeGaze2.pitch, tolerance);
  EXPECT_NEAR(eyeGaze1.depth, eyeGaze2.depth, tolerance);
  EXPECT_NEAR(eyeGaze1.yaw_low, eyeGaze2.yaw_low, tolerance);
  EXPECT_NEAR(eyeGaze1.yaw_high, eyeGaze2.yaw_high, tolerance);
  EXPECT_NEAR(eyeGaze1.pitch_low, eyeGaze2.pitch_low, tolerance);
  EXPECT_NEAR(eyeGaze1.pitch_high, eyeGaze2.pitch_high, tolerance);

  EXPECT_TRUE(
      eyeGaze1.spatial_gaze_point_in_cpf.isApprox(eyeGaze2.spatial_gaze_point_in_cpf, tolerance));
  EXPECT_TRUE(eyeGaze1.combined_gaze_origin_in_cpf.isApprox(
      eyeGaze2.combined_gaze_origin_in_cpf, tolerance));
  EXPECT_EQ(eyeGaze1.spatial_gaze_point_valid, eyeGaze2.spatial_gaze_point_valid);
  EXPECT_EQ(eyeGaze1.combined_gaze_valid, eyeGaze2.combined_gaze_valid);

  // Compare left and right eyes
  const auto& vergence1 = eyeGaze1.vergence;
  const auto& vergence2 = eyeGaze2.vergence;
  EXPECT_FLOAT_EQ(vergence1.left_yaw, vergence2.left_yaw);
  EXPECT_FLOAT_EQ(vergence1.right_yaw, vergence2.right_yaw);
  EXPECT_FLOAT_EQ(vergence1.left_yaw_low, vergence2.left_yaw_low);
  EXPECT_FLOAT_EQ(vergence1.right_yaw_low, vergence2.right_yaw_low);
  EXPECT_FLOAT_EQ(vergence1.left_yaw_high, vergence2.left_yaw_high);
  EXPECT_FLOAT_EQ(vergence1.right_yaw_high, vergence2.right_yaw_high);
  Eigen::Vector3f leftEyeTranslation1(
      vergence1.tx_left_eye, vergence1.ty_left_eye, vergence1.tz_left_eye);
  Eigen::Vector3f rightEyeTranslation1(
      vergence1.tx_right_eye, vergence1.ty_right_eye, vergence1.tz_right_eye);
  Eigen::Vector3f leftEyeTranslation2(
      vergence2.tx_left_eye, vergence2.ty_left_eye, vergence2.tz_left_eye);
  Eigen::Vector3f rightEyeTranslation2(
      vergence2.tx_right_eye, vergence2.ty_right_eye, vergence2.tz_right_eye);
  EXPECT_TRUE(leftEyeTranslation1.isApprox(leftEyeTranslation2, tolerance));
  EXPECT_TRUE(rightEyeTranslation1.isApprox(rightEyeTranslation2, tolerance));
  EXPECT_NEAR(vergence1.left_pitch, vergence2.left_pitch, tolerance);
  EXPECT_NEAR(vergence1.right_pitch, vergence2.right_pitch, tolerance);
  EXPECT_EQ(vergence1.left_blink, vergence2.left_blink);
  EXPECT_EQ(vergence1.right_blink, vergence2.right_blink);

  EXPECT_TRUE(vergence1.left_entrance_pupil_position_meter.isApprox(
      vergence2.left_entrance_pupil_position_meter, tolerance));
  EXPECT_TRUE(vergence1.right_entrance_pupil_position_meter.isApprox(
      vergence2.right_entrance_pupil_position_meter, tolerance));
  EXPECT_FLOAT_EQ(vergence1.left_pupil_diameter_meter, vergence2.left_pupil_diameter_meter);
  EXPECT_FLOAT_EQ(vergence1.right_pupil_diameter_meter, vergence2.right_pupil_diameter_meter);
  EXPECT_EQ(vergence1.left_gaze_valid, vergence2.left_gaze_valid);
  EXPECT_EQ(vergence1.right_gaze_valid, vergence2.right_gaze_valid);
  EXPECT_EQ(vergence1.left_blink_valid, vergence2.left_blink_valid);
  EXPECT_EQ(vergence1.right_blink_valid, vergence2.right_blink_valid);
  EXPECT_EQ(
      vergence1.left_entrance_pupil_position_valid, vergence2.left_entrance_pupil_position_valid);
  EXPECT_EQ(
      vergence1.right_entrance_pupil_position_valid, vergence2.right_entrance_pupil_position_valid);
  EXPECT_EQ(vergence1.left_pupil_diameter_valid, vergence2.left_pupil_diameter_valid);
  EXPECT_EQ(vergence1.right_pupil_diameter_valid, vergence2.right_pupil_diameter_valid);
}

inline void compare(const TimeSyncData& timeSyncData1, const TimeSyncData& timeSyncData2) {
  EXPECT_EQ(timeSyncData1.monotonicTimestampNs, timeSyncData2.monotonicTimestampNs);
  EXPECT_EQ(timeSyncData1.realTimestampNs, timeSyncData2.realTimestampNs);
}

inline void compare(
    const AriaTimeSyncConfigRecord& timeSyncConfig1,
    const AriaTimeSyncConfigRecord& timeSyncConfig2) {
  EXPECT_EQ(timeSyncConfig1.streamId, timeSyncConfig2.streamId);
  EXPECT_DOUBLE_EQ(timeSyncConfig1.sampleRateHz, timeSyncConfig2.sampleRateHz);
  EXPECT_EQ(timeSyncConfig1.mode, timeSyncConfig2.mode);
}

} // namespace projectaria::tools::data_provider::test
