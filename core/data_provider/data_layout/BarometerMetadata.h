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

#include <cstdint>

#include <vrs/DataLayout.h>
#include <vrs/DataPieces.h>

// Note: The VRS stream type for barometer data is
// vrs::RecordableTypeId::BarometerRecordableClass.

namespace datalayout {

struct BarometerConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};
  vrs::DataPieceString sensorModelName{"sensor_model_name"};

  // Sample rate for temperature and pressure data (in unit of Hz)
  vrs::DataPieceValue<double> sampleRate{"sample_rate"};

  vrs::AutoDataLayoutEnd end;
};

struct BarometerDataMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  // Timestamp of data capture in board clock, in unit of nanoseconds.
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};

  // Temperature in Celsius degree.
  vrs::DataPieceValue<double> temperature{"temperature"};

  // Pressure in Pascal.
  vrs::DataPieceValue<double> pressure{"pressure"};

  // Relative altitude in meters.
  vrs::DataPieceValue<double> altitude{"altitude"};

  vrs::AutoDataLayoutEnd end;
};

} // namespace datalayout
