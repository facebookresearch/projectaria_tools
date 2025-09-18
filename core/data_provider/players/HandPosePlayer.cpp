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

#include <data_provider/players/HandPosePlayer.h>

#include <fmt/format.h>
#include <cmath>

#include <data_provider/players/DataPlayerUtils.h>

namespace projectaria::tools::data_provider {

using namespace projectaria::tools::mps;

namespace {

constexpr uint8_t kHtLandmarksCount = mps::kNumHandLandmarks;

std::optional<HandTrackingResult::OneSide> populateOneSideHand(
    const datalayout::SingleHandPoseLayoutStruct& singleHandPoseLayout,
    HANDEDNESS handness) {
  HandTrackingResult::OneSide oneSide;

  // Parse validity flag
  bool handValid = singleHandPoseLayout.handValid.get();
  if (!handValid) {
    return {};
  }

  // Parse T_Device_Wrist pose
  vrs::Point4Df wristRotationXyzw;
  if (!singleHandPoseLayout.wristRotationXyzw.get(wristRotationXyzw)) {
    fmt::print("Missing wristRotationXyzw in hand pose data layout!");
    return {};
  }
  vrs::Point3Df wristTranslationMeterXyz;
  if (!singleHandPoseLayout.wristTranslationMeterXyz.get(wristTranslationMeterXyz)) {
    fmt::print("Missing wristTranslationMeterXyz in hand pose data layout!");
    return {};
  }
  oneSide.T_Device_Wrist =
      populateToSE3<float, double>(wristRotationXyzw, wristTranslationMeterXyz);

  // Parse hand confidences
  oneSide.confidence = singleHandPoseLayout.handConfidence.get();

  // Parse Landmarks
  std::vector<vrs::Point3Df> vrsLandmarks3d;
  if (!singleHandPoseLayout.handLandmarks3d_DeviceMeterXyz.get(vrsLandmarks3d)) {
    fmt::print("Missing handLandmarks3d_DeviceMeterXyz in hand pose data layout!");
    return {};
  }
  for (int landmarkIdx = 0; landmarkIdx < kHtLandmarksCount; ++landmarkIdx) {
    oneSide.landmarkPositions_device[landmarkIdx] =
        mapToEigenVector<float, double, 3>(vrsLandmarks3d.at(landmarkIdx));
  }

  // Set wrist normal as -Y of T_Device_Wrist
  HandTrackingResult::OneSide::WristAndPalmNormals wristAndPalmNormal;
  wristAndPalmNormal.wristNormal_device = -1.0 * oneSide.T_Device_Wrist.rotationMatrix().col(1);
  wristAndPalmNormal.palmNormal_device =
      estimatePalmNormal(oneSide.landmarkPositions_device, handness);
  oneSide.wristAndPalmNormal_device = wristAndPalmNormal;

  return oneSide;
}
} // namespace

bool HandPosePlayer::onDataLayoutRead(
    const vrs::CurrentRecord& header,
    size_t blockIndex,
    vrs::DataLayout& layout) {
  if (header.recordType == vrs::Record::Type::CONFIGURATION) {
    const datalayout::HandPoseConfigurationLayout& configLayout =
        getExpectedLayout<datalayout::HandPoseConfigurationLayout>(layout, blockIndex);
    if (!configLayout.streamId.get(configRecord_.streamId)) {
      fmt::print("Missing streamId in hand pose config layout!");
      return false;
    }
    if (!configLayout.nominalRateHz.get(configRecord_.nominalRateHz)) {
      fmt::print("Missing nominalRateHz in hand pose config layout!");
      return false;
    }
    configRecord_.isWristPalmOnly = configLayout.isWristPalmOnly.get();
    configRecord_.userProfile = configLayout.userProfile.get();
    return true;
  } else if (header.recordType == vrs::Record::Type::DATA) {
    const datalayout::HandPoseDataLayout& dataLayout =
        getExpectedLayout<datalayout::HandPoseDataLayout>(layout, blockIndex);

    // Parsing timestamp
    int64_t timestampNs;
    if (dataLayout.captureTimestampNs.get(timestampNs)) {
      dataRecord_.trackingTimestamp = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::nanoseconds(timestampNs));
    } else {
      fmt::print("Missing captureTimestamp in hand pose data layout!");
      return false;
    }

    // Populate left and right hand pose
    dataRecord_.leftHand = populateOneSideHand(dataLayout.leftHand, HANDEDNESS::LEFT);
    dataRecord_.rightHand = populateOneSideHand(dataLayout.rightHand, HANDEDNESS::RIGHT);

    callback_(dataRecord_);

    return true;
  }
  return false;
}

} // namespace projectaria::tools::data_provider
