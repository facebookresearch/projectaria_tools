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

// Note: The VRS stream type for Bluetooth beacon data is
// vrs::RecordableTypeId::BluetoothBeaconRecordableClass.

namespace datalayout {

struct BluetoothBeaconConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};

  // Sample rate [Hz]
  vrs::DataPieceValue<double> sampleRateHz{"sample_rate_hz"};

  vrs::AutoDataLayoutEnd end;
};

struct BluetoothBeaconDataMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;

  // Timestamp of the data sample in real time (UNIX epoch).
  vrs::DataPieceValue<std::int64_t> systemTimestampNs{"system_timestamp_ns"};

  // Timestamp of the data sample in board-clock domain.
  vrs::DataPieceValue<std::int64_t> boardTimestampNs{"board_timestamp_ns"};

  // Timestamp (in board clock) when scan request was issued.
  // This field is used to group samples in the same way as phones do it.
  vrs::DataPieceValue<std::int64_t> boardScanRequestStartTimestampNs{
      "board_request_start_timestamp_ns"};

  // Timestamp (in board clock) when scan request is completed.
  // This field is used to group samples and calculate single scan duration.
  vrs::DataPieceValue<std::int64_t> boardScanRequestCompleteTimestampNs{
      "board_request_complete_timestamp_ns"};

  // A string representing the unique identifier of the Bluetooth beacon physical device.
  vrs::DataPieceString uniqueId{"unique_id"};

  // Transmission power, in units of dBm.
  vrs::DataPieceValue<float> txPower{"tx_power"};

  // Received signal strength indication, in units of dBm.
  vrs::DataPieceValue<float> rssi{"rssi"};

  // Frequency of the signal, in units of MHz.
  vrs::DataPieceValue<float> freqMhz{"freq_mhz"};

  vrs::AutoDataLayoutEnd end;
};

} // namespace datalayout
