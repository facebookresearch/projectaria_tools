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

// Note: The VRS stream type for battery status data is
// vrs::RecordableTypeId::BatteryStatusRecordableClass.

namespace datalayout {

struct BatteryStatusConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};
  vrs::DataPieceString sensorModel{"sensor_model"};
  vrs::DataPieceValue<std::uint64_t> deviceId{"device_id"};
  vrs::DataPieceValue<double> nominalRateHz{"nominal_rate"};
  vrs::DataPieceString description{"description"};

  vrs::AutoDataLayoutEnd end;
};

struct BatteryStatusDataMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  // Timestamp of data capture in board clock, in unit of nanoseconds.
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};

  // Battery charging status (0=Discharging, 1=Charging, 2=Full, 3=Unknown)
  vrs::DataPieceValue<std::uint32_t> chargingStatus{"charging_status"};

  // Battery level in percentage. Value range [0, 100]
  vrs::DataPieceValue<std::uint8_t> batteryLevel{"battery_level"};

  // Temperature in Celsius
  vrs::DataPieceValue<float> temperatureC{"temperature_c"};

  // Current voltage in volts
  vrs::DataPieceValue<float> voltageNowVolt{"voltage_now_volt"};

  // Average voltage in volts
  vrs::DataPieceValue<float> voltageAvgVolt{"voltage_avg_volt"};

  // Current current in amps
  vrs::DataPieceValue<float> currentNowAmp{"current_now_amp"};

  // Average current in amps
  vrs::DataPieceValue<float> currentAvgAmp{"current_avg_amp"};

  // Current power in watts
  vrs::DataPieceValue<float> powerNowWatt{"power_now_watt"};

  // Average power in watts
  vrs::DataPieceValue<float> powerAvgWatt{"power_avg_watt"};

  vrs::AutoDataLayoutEnd end;
};

} // namespace datalayout
