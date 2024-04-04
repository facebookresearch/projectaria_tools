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

#include "MotionSensorPlayer.h"

#include <vrs/ErrorCode.h>

namespace projectaria::tools::data_provider {

bool MotionSensorPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config = getExpectedLayout<datalayout::MotionSensorConfigRecordMetadata>(dl, blockIndex);
    configRecord_.streamIndex = config.streamIndex.get();
    configRecord_.deviceType = config.deviceType.get();
    configRecord_.deviceVersion = config.deviceVersion.get();
    configRecord_.deviceSerial = config.deviceSerial.get();
    configRecord_.deviceId = config.deviceId.get();
    configRecord_.sensorModel = config.sensorModel.get();
    configRecord_.nominalRateHz = config.nominalRateHz.get();
    configRecord_.hasAccelerometer = config.hasAccelerometer.get();
    configRecord_.hasGyroscope = config.hasGyroscope.get();
    configRecord_.hasMagnetometer = config.hasMagnetometer.get();
    configRecord_.factoryCalibration = config.factoryCalibration.get();
    configRecord_.onlineCalibration = config.onlineCalibration.get();
    configRecord_.description = config.description.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::MotionSensorDataRecordMetadata>(dl, blockIndex);
    dataRecord_.accelValid = data.accelValid.get();
    dataRecord_.gyroValid = data.gyroValid.get();
    dataRecord_.magValid = data.magValid.get();
    dataRecord_.temperature = data.temperature.get();
    dataRecord_.captureTimestampNs = data.captureTimestampNs.get();
    dataRecord_.arrivalTimestampNs = data.arrivalTimestampNs.get();
    data.accelMSec2.get(dataRecord_.accelMSec2.data(), 3);
    data.gyroRadSec.get(dataRecord_.gyroRadSec.data(), 3);
    data.magTesla.get(dataRecord_.magTesla.data(), 3);
    nextTimestampSec_ = std::nextafter(r.timestamp, std::numeric_limits<double>::max());
    callback_(dataRecord_, configRecord_, verbose_);
  }
  return true;
}

} // namespace projectaria::tools::data_provider
