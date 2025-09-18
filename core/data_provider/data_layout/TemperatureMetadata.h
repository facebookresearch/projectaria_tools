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

namespace datalayout {

class TemperatureConfigurationLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;
  vrs::DataPieceValue<uint32_t> streamId{"stream_id"};
  vrs::DataPieceValue<uint64_t> deviceId{"device_id"};
  vrs::DataPieceValue<double> nominalRate{"nominal_rate"};
  vrs::DataPieceString sensorModel{"sensor_model"};

  vrs::AutoDataLayoutEnd endLayout;
};

class TemperatureDataLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<int64_t> captureTimestampNs{"capture_timestamp_ns"};
  vrs::DataPieceValue<float> temperatureCelsius{"temperature_celsius"};
  vrs::DataPieceString sensorName{"sensor_name"};

  vrs::AutoDataLayoutEnd endLayout;
};

} // namespace datalayout
