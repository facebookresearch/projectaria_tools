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

#include <data_provider/players/VioHighFrequencyPlayer.h>

#include <fmt/format.h>

#include <data_provider/players/DataPlayerUtils.h>
#include <vrs/RecordFileReader.h>
#include <vrs/RecordFormat.h>
#include <vrs/RecordReaders.h>

namespace projectaria::tools::data_provider {

bool VioHighFrequencyPlayer::onDataLayoutRead(
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
    const datalayout::VioHighFrequencyResultDataLayout& dataLayout =
        getExpectedLayout<datalayout::VioHighFrequencyResultDataLayout>(layout, blockIndex);
    return onVioHighFrequencyResultDataLayoutRead(dataLayout);
  }
  return false;
}

bool VioHighFrequencyPlayer::onVioHighFrequencyResultDataLayoutRead(
    const datalayout::VioHighFrequencyResultDataLayout& dataLayout) {
  // Parsing timestamp
  int64_t timestampNs;
  if (dataLayout.captureTimestampNs.get(timestampNs)) {
    dataRecord_.trackingTimestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::nanoseconds(timestampNs));
  } else {
    fmt::print("Missing captureTimestamp in VIO result data layout!");
    return false;
  }

  if (!dataLayout.frontendSessionUuid.get(dataRecord_.sessionUid)) {
    fmt::print("Missing captureTimestampNs in VIO result data layout!");
    return false;
  }

  vrs::Point3Dd gravityVec_Odometry;
  if (!dataLayout.gravityVec_Odometry.get(gravityVec_Odometry)) {
    fmt::print("Missing gravityVec_Odometry in VIO result data layout!");
    return false;
  }
  dataRecord_.gravity_odometry = mapToEigenVector(gravityVec_Odometry);

  // txyz_Odometry_Device, stored as translation and quaternion separately
  vrs::Point3Dd txyz_Odometry_Device;
  if (!dataLayout.txyz_Odometry_Device.get(txyz_Odometry_Device)) {
    fmt::print("Missing txyz_Odometry_Device in VIO result data layout!");
    return false;
  }
  vrs::Point4Dd qxyzw_Odometry_Device;
  if (!dataLayout.qxyzw_Odometry_Device.get(qxyzw_Odometry_Device)) {
    fmt::print("Missing qxyzw_Odometry_Device in VIO result data layout!");
    return false;
  }
  dataRecord_.T_odometry_device =
      populateToSE3<double>(qxyzw_Odometry_Device, txyz_Odometry_Device);

  vrs::Point3Dd linearVelocity_Odometry;
  if (!dataLayout.linearVelocity_Odometry.get(linearVelocity_Odometry)) {
    fmt::print("Missing linearVelocity_Odometry in VIO result data layout!");
    return false;
  }
  dataRecord_.deviceLinearVelocity_odometry = mapToEigenVector(linearVelocity_Odometry);

  vrs::Point3Dd angularVelocity_Device;
  if (!dataLayout.angularVelocity_Device.get(angularVelocity_Device)) {
    fmt::print("Missing angularVelocity_Device in VIO result data layout!");
    return false;
  }
  dataRecord_.angularVelocity_device = mapToEigenVector(angularVelocity_Device);

  dataRecord_.qualityScore = dataLayout.qualityScore.get();

  if (highFrequencyCallback_) {
    highFrequencyCallback_(dataRecord_, configRecord_);
  }

  return true;
}

} // namespace projectaria::tools::data_provider
