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

#include <data_provider/players/VioPlayer.h>

#include <fmt/format.h>

#include <data_provider/json_io/FrontendTypesJson.h>
#include <data_provider/players/DataPlayerUtils.h>
#include <vrs/RecordFileReader.h>
#include <vrs/RecordFormat.h>
#include <vrs/RecordReaders.h>

namespace projectaria::tools::data_provider {

bool VioPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& header,
    size_t blockIndex,
    vrs::DataLayout& layout) {
  if (header.recordType == vrs::Record::Type::CONFIGURATION) {
    const datalayout::VioResultConfigurationLayout& configLayout =
        getExpectedLayout<datalayout::VioResultConfigurationLayout>(layout, blockIndex);
    if (!configLayout.streamId.get(configRecord_.streamId)) {
      fmt::print("Missing streamId in VIO result config layout!");
      return false;
    }
    if (!configLayout.nominalRateHz.get(configRecord_.nominalRateHz)) {
      fmt::print("Missing nominalRateHz in VIO result config layout!");
      return false;
    }

    configRecord_.messageVersion = configLayout.messageVersion.get();

    return true;
  } else if (header.recordType == vrs::Record::Type::DATA) {
    const datalayout::VioResultDataLayout& dataLayout =
        getExpectedLayout<datalayout::VioResultDataLayout>(layout, blockIndex);
    return onVioResultDataLayoutRead(dataLayout);
  }
  return false;
}

bool VioPlayer::onVioResultDataLayoutRead(const datalayout::VioResultDataLayout& dataLayout) {
  if (!dataLayout.captureTimestampNs.get(dataRecord_.captureTimestampNs)) {
    fmt::print("Missing captureTimestampNs in VIO result data layout!");
    return false;
  }

  // Parse frontendSessionUid
  std::string uuidString;
  if (!dataLayout.frontendSessionUuid.get(uuidString)) {
    fmt::print("Missing frontendSessionUuid in VIO result data layout!");
    return false;
  }
  const auto uidAndFlag = FrontendSessionUid::createFromString(uuidString);
  if (!uidAndFlag.second) {
    fmt::print("Parsing FrontendSessionUid from string has failed!");
    return false;
  }
  dataRecord_.frontendSessionUid = uidAndFlag.first;

  // Parse frame id
  int32_t frameId = 0;
  if (!dataLayout.frameId.get(frameId) || frameId < 0) {
    fmt::print(
        "Missing frameId in VIO result data layout, or parsed frame ID has negative value of {}!",
        frameId);
    return false;
  }
  dataRecord_.frameID = static_cast<FrameSetId>(frameId);

  // Parse VIO status
  int8_t vioStatus;
  if (!dataLayout.status.get(vioStatus)) {
    fmt::print("Missing vioStatus in VIO result data layout!");
    return false;
  }
  dataRecord_.status = static_cast<VioStatus>(vioStatus);

  // Parse tracker health status
  // TODO: change this when VRS records is written as enum
  bool poseQualityIsGood = dataLayout.isTrackerHealthy.get();
  dataRecord_.poseQuality = poseQualityIsGood ? TrackingQuality::GOOD : TrackingQuality::BAD;
  bool visualTrackingQualityIsGood = dataLayout.isMapTrackingHealthy.get();
  dataRecord_.visualTrackingQuality =
      visualTrackingQualityIsGood ? VisualTrackingQuality::GOOD : VisualTrackingQuality::BAD;

  // Parse gravity vec
  vrs::Point3Dd gravityVecInOdometry;
  if (!dataLayout.gravityVecInOdometry.get(gravityVecInOdometry)) {
    fmt::print("Missing gravityVec_Odometry in VIO result data layout!");
    return false;
  }
  dataRecord_.gravityInOdometry = mapToEigenVector<double, float, 3>(gravityVecInOdometry);

  // Parse T_BodyImu_Device
  vrs::Point3Dd txyz_BodyImu_Device;
  if (!dataLayout.txyz_BodyImu_Device.get(txyz_BodyImu_Device)) {
    fmt::print("Missing txyz_BodyImu_Device in VIO result data layout!");
    return false;
  }
  vrs::Point4Dd qxyzw_BodyImu_Device;
  if (!dataLayout.qxyzw_BodyImu_Device.get(qxyzw_BodyImu_Device)) {
    fmt::print("Missing qxyzw_BodyImu_Device in VIO result data layout!");
    return false;
  }
  dataRecord_.T_BodyImu_Device =
      populateToSE3<double, float>(qxyzw_BodyImu_Device, txyz_BodyImu_Device);

  // T_Odometry_BodyImu, stored as translation and quaternion separately
  vrs::Point3Dd txyz_Odometry_BodyImu;
  if (!dataLayout.txyz_Odometry_BodyImu.get(txyz_Odometry_BodyImu)) {
    fmt::print("Missing txyz_Odometry_BodyImu in VIO result data layout!");
    return false;
  }
  vrs::Point4Dd qxyzw_Odometry_BodyImu;
  if (!dataLayout.qxyzw_Odometry_BodyImu.get(qxyzw_Odometry_BodyImu)) {
    fmt::print("Missing qxyzw_Odometry_BodyImu in VIO result data layout!");
    return false;
  }
  dataRecord_.T_Odometry_BodyImu =
      populateToSE3<double, float>(qxyzw_Odometry_BodyImu, txyz_Odometry_BodyImu);

  // Parse velocity
  vrs::Point3Dd linearVelocityInOdometry;
  if (!dataLayout.linearVelocityInOdometry.get(linearVelocityInOdometry)) {
    fmt::print("Missing linearVelocity_Odometry in VIO result data layout!");
    return false;
  }
  dataRecord_.linearVelocityInOdometry =
      mapToEigenVector<double, float, 3>(linearVelocityInOdometry);

  vrs::Point3Dd angularVelocityInBodyImu;
  if (!dataLayout.angularVelocityInBodyImu.get(angularVelocityInBodyImu)) {
    fmt::print("Missing angularVelocity_BodyImu in VIO result data layout!");
    return false;
  }
  dataRecord_.angularVelocityInBodyImu =
      mapToEigenVector<double, float, 3>(angularVelocityInBodyImu);

  // Read online calibration
  std::string onlineCalibString;
  if (!dataLayout.onlineCalibState.get(onlineCalibString)) {
    fmt::print(
        "Missing online calibration in VIO result data layout, this data field will be empty.\n");
  } else {
    const auto maybeOnlineCalib =
        projectaria::tools::json::onlineCalibStateFromJsonStr(onlineCalibString);
    if (!maybeOnlineCalib.has_value()) {
      fmt::print("Cannot correctly parse online calibration string in VIO result data layout!");
      return false;
    }
    dataRecord_.onlineCalib = maybeOnlineCalib.value();
  }

  if (callback_) {
    callback_(dataRecord_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
