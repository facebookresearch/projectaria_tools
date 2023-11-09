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

#include <cmath>

#include "PosePlayer.h"

namespace projectaria::tools::data_provider {

bool PosePlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config = getExpectedLayout<datalayout::PoseConfigRecordMetadata>(dl, blockIndex);
    configRecord_.streamId = config.streamId.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::PoseDataRecordMetadata>(dl, blockIndex);
    dataRecord_.captureTimestampNs = data.captureTimestampNs.get();
    vrs::Point3Df translation;
    data.T_World_ImuLeft_translation.get(translation);
    dataRecord_.T_World_ImuLeft_translation[0] = translation[0];
    dataRecord_.T_World_ImuLeft_translation[1] = translation[1];
    dataRecord_.T_World_ImuLeft_translation[2] = translation[2];
    vrs::Point4Df quaternion;
    data.T_World_ImuLeft_quaternion.get(quaternion);
    dataRecord_.T_World_ImuLeft_quaternion[0] = quaternion[0];
    dataRecord_.T_World_ImuLeft_quaternion[1] = quaternion[1];
    dataRecord_.T_World_ImuLeft_quaternion[2] = quaternion[2];
    dataRecord_.T_World_ImuLeft_quaternion[3] = quaternion[3];
    nextTimestampSec_ = std::nextafter(r.timestamp, std::numeric_limits<double>::max());
    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
