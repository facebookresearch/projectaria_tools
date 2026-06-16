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

namespace datalayout {

// Subset of oatmeal::GenericSensorConfigurationLayout for the EMG IMU batch stream: the recorder's
// factoryCalibration{"calibration"} field is intentionally omitted because PAT does not expose EMG
// calibration. The DataPiece labels below must match the recorder byte-for-byte (see
// arvr/projects/oatmeal/vrs/sensors/GenericLayout.h); a recorder-side label rename would make
// .get() silently return type defaults. These layouts ship in the OSS tree and cannot include the
// internal recorder headers, so the labels are duplicated here by necessity.
class EmgConfigurationLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;
  vrs::DataPieceValue<uint32_t> streamId{"stream_id"};
  vrs::DataPieceString sensorModel{"sensor_model"};
  vrs::DataPieceValue<std::uint64_t> deviceId{"device_id"};
  vrs::DataPieceValue<double> nominalRateHz{"nominal_rate"};
  vrs::DataPieceString description{"description"};

  vrs::AutoDataLayoutEnd endLayout;
};

// Mirrors oatmeal::EmgImuBatchDataLayout. The DataPiece labels below must match the recorder
// byte-for-byte (see arvr/projects/oatmeal/vrs/sensors/EmgImuBatchLayout.h); a recorder-side label
// rename would make .get() silently return type defaults. These layouts ship in the OSS tree and
// cannot include the internal recorder headers, so the labels are duplicated here by necessity.
class EmgDataLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;

  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};
  vrs::DataPieceValue<std::uint32_t> batchSequenceNumber{"batch_sequence_number"};
  vrs::DataPieceValue<std::uint32_t> emgSampleCount{"emg_sample_count"};
  vrs::DataPieceValue<std::uint32_t> accelSampleCount{"accel_sample_count"};
  vrs::DataPieceValue<std::uint32_t> gyroSampleCount{"gyro_sample_count"};
  vrs::DataPieceVector<std::uint32_t> emgSequenceNumbers{"emg_sequence_numbers"};
  vrs::DataPieceVector<std::uint64_t> emgTimestampsNs{"emg_timestamps_ns"};
  vrs::DataPieceVector<std::string> emgChannels{"emg_channels"};
  vrs::DataPieceVector<std::uint32_t> emgEncodings{"emg_encodings"};
  vrs::DataPieceVector<std::uint32_t> accelSequenceNumbers{"accel_sequence_numbers"};
  vrs::DataPieceVector<std::uint64_t> accelTimestampsNs{"accel_timestamps_ns"};
  vrs::DataPieceVector<std::string> accelChannels{"accel_channels"};
  vrs::DataPieceVector<std::uint32_t> gyroSequenceNumbers{"gyro_sequence_numbers"};
  vrs::DataPieceVector<std::uint64_t> gyroTimestampsNs{"gyro_timestamps_ns"};
  vrs::DataPieceVector<std::string> gyroChannels{"gyro_channels"};
  vrs::DataPieceValue<std::uint32_t> channelCount{"channel_count"};
  vrs::DataPieceValue<std::uint32_t> bitsPerAdcReading{"bits_per_adc_reading"};
  vrs::DataPieceValue<std::uint32_t> samplesPerBatch{"samples_per_batch"};

  vrs::AutoDataLayoutEnd endLayout;
};
} // namespace datalayout
