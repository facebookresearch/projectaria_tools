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

// ALS Ambient Light Sensor Config and Data Layouts in VRS file

namespace datalayout {

class AlsConfigurationLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;
  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};
  vrs::DataPieceValue<std::uint64_t> deviceId{"device_id"};
  vrs::DataPieceValue<double> nominalRate{"nominal_rate"};
  vrs::DataPieceString sensorModel{"sensor_model"};

  vrs::AutoDataLayoutEnd endLayout;
};

class AlsDataLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};
  vrs::DataPieceValue<float> redChannelNormalized{"red_channel_normalized"};
  vrs::DataPieceValue<float> greenChannelNormalized{"green_channel_normalized"};
  vrs::DataPieceValue<float> blueChannelNormalized{"blue_channel_normalized"};
  vrs::DataPieceValue<float> uvChannelNormalized{"uv_channel_normalized"};
  vrs::DataPieceValue<float> irChannelNormalized{"ir_channel_normalized"};
  vrs::DataPieceValue<float> clearChannelNormalized{"clear_channel_normalized"};
  vrs::DataPieceValue<float> uvFluxWattPerSquareMeter{"uv_flux_watt_per_square_meter"};
  vrs::DataPieceValue<float> irFluxWattPerSquareMeter{"ir_flux_watt_per_square_meter"};
  vrs::DataPieceValue<float> clearFluxWattPerSquareMeter{"clear_flux_watt_per_square_meter"};
  vrs::DataPieceValue<std::int32_t> gainRed{"gain_red"};
  vrs::DataPieceValue<std::int32_t> gainGreen{"gain_green"};
  vrs::DataPieceValue<std::int32_t> gainBlue{"gain_blue"};
  vrs::DataPieceValue<std::int32_t> gainUv{"gain_uv"};
  vrs::DataPieceValue<std::int32_t> gainIr{"gain_ir"};
  vrs::DataPieceValue<std::int32_t> gainClear{"gain_clear"};
  vrs::DataPieceValue<std::int32_t> exposureTimeUs{"exposure_time_us"};
  vrs::DataPieceValue<float> cct{"cct"};
  vrs::DataPieceValue<float> lux{"lux"};

  vrs::AutoDataLayoutEnd endLayout;
};
} // namespace datalayout
