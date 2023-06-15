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

#include <vrs/StreamId.h>

// List all StreamId related to Aria Data Layout
namespace projectaria::tools::data_provider {
static const vrs::StreamId kEyeCameraStreamId{vrs::RecordableTypeId::EyeCameraRecordableClass, 1};
static const vrs::StreamId kRgbCameraStreamId{vrs::RecordableTypeId::RgbCameraRecordableClass, 1};
static const vrs::StreamId kSlamLeftCameraStreamId{vrs::RecordableTypeId::SlamCameraData, 1};
static const vrs::StreamId kSlamRightCameraStreamId{vrs::RecordableTypeId::SlamCameraData, 2};
static const vrs::StreamId kImuRightStreamId{vrs::RecordableTypeId::SlamImuData, 1};
static const vrs::StreamId kImuLeftStreamId{vrs::RecordableTypeId::SlamImuData, 2};
static const vrs::StreamId kMagnetometerStreamId{vrs::RecordableTypeId::SlamMagnetometerData, 1};
static const vrs::StreamId kBarometerStreamId{vrs::RecordableTypeId::BarometerRecordableClass, 1};
static const vrs::StreamId kAudioStreamId{vrs::RecordableTypeId::StereoAudioRecordableClass, 1};
static const vrs::StreamId kWifiStreamId{vrs::RecordableTypeId::WifiBeaconRecordableClass, 1};
static const vrs::StreamId kBluetoothStreamId{
    vrs::RecordableTypeId::BluetoothBeaconRecordableClass,
    1};
static const vrs::StreamId kGpsStreamId{vrs::RecordableTypeId::GpsRecordableClass, 1};
static const vrs::StreamId kTimeSyncStreamId{vrs::RecordableTypeId::TimeRecordableClass, 1};
} // namespace projectaria::tools::data_provider
