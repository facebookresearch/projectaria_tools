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
#include <vrs/DataLayout.h>
#include <vrs/DataPieces.h>
#include <cstdint>

// ALS light flicker detection Config and Data Layouts in VRS file.
// The field names below must match the names written by the on-device recorder.

namespace datalayout {

class AlsFlickerDetectionConfigurationLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;
  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};
  vrs::DataPieceValue<std::uint64_t> deviceId{"device_id"};
  vrs::DataPieceValue<double> nominalRate{"nominal_rate"};
  vrs::DataPieceString sensorModel{"sensor_model"};

  vrs::AutoDataLayoutEnd endLayout;
};

class AlsFlickerDetectionDataLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};

  // Three frequency/magnitude pairs for the top detected flicker frequencies.
  vrs::DataPieceValue<float> flickerFrequencyHz0{"flicker_frequency_hz_0"};
  vrs::DataPieceValue<float> flickerMagnitude0{"flicker_magnitude_0"};
  vrs::DataPieceValue<float> flickerFrequencyHz1{"flicker_frequency_hz_1"};
  vrs::DataPieceValue<float> flickerMagnitude1{"flicker_magnitude_1"};
  vrs::DataPieceValue<float> flickerFrequencyHz2{"flicker_frequency_hz_2"};
  vrs::DataPieceValue<float> flickerMagnitude2{"flicker_magnitude_2"};

  vrs::AutoDataLayoutEnd endLayout;
};
} // namespace datalayout
