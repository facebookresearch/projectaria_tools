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
#include <vrs/DataLayoutConventions.h>
#include <vrs/DataPieces.h>

// Note: The VRS stream types for image data are vrs::RecordableTypeId::SlamCameraData (for SLAM
// cameras), vrs::RecordableTypeId::RgbCameraRecordableClass (for RGB camera), and
// vrs::RecordableTypeId::EyeCameraRecordableClass (for eye-tracking cameras).

namespace datalayout {

using vrs::datalayout_conventions::ImageSpecType;

struct ImageSensorConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;

  // Type, version, serial of *entire* HMD device
  vrs::DataPieceString deviceType{"device_type"};
  vrs::DataPieceString deviceVersion{"device_version"};
  vrs::DataPieceString deviceSerial{"device_serial"};

  // Index of the camera in the calibration JSON
  vrs::DataPieceValue<std::uint32_t> cameraId{"camera_id"};

  // Type & serial number of the camera module.
  vrs::DataPieceString sensorModel{"sensor_model"};
  vrs::DataPieceString sensorSerial{"sensor_serial"};

  // Nominal frame rate [Hz], NaN if not available
  vrs::DataPieceValue<double> nominalRateHz{"nominal_rate"};

  // Image description. Part of general VRS data layout conventions.
  vrs::DataPieceValue<ImageSpecType> imageWidth{vrs::datalayout_conventions::kImageWidth};
  vrs::DataPieceValue<ImageSpecType> imageHeight{vrs::datalayout_conventions::kImageHeight};
  vrs::DataPieceValue<ImageSpecType> imageStride{vrs::datalayout_conventions::kImageStride};
  vrs::DataPieceValue<ImageSpecType> pixelFormat{vrs::datalayout_conventions::kImagePixelFormat};

  // Exposure duration min, max [s]. NaN if not available.
  vrs::DataPieceValue<double> exposureDurationMin{"exposure_duration.min"};
  vrs::DataPieceValue<double> exposureDurationMax{"exposure_duration.max"};

  // Linear gain min, max [-, no unit]. NaN if not available.
  vrs::DataPieceValue<double> gainMin{"gain.min"};
  vrs::DataPieceValue<double> gainMax{"gain.max"};

  // Gamma factor [-, no unit]
  vrs::DataPieceValue<double> gammaFactor{"gamma_factor"};

  // Calibration (as JSON) of the *all* device sensors
  vrs::DataPieceString factoryCalibration{"factory_calibration"};
  vrs::DataPieceString onlineCalibration{"online_calibration"};

  vrs::DataPieceString description{"description"};

  vrs::AutoDataLayoutEnd end;
};

// The field imageBufferSize is not part of the data layout because it is
// redundant since it can be obtained via a call to vrs::ContentBlock::getBlockSize().
struct ImageSensorDataRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;

  // The first two entries are used to identify frame groups:
  // group_id is an increasing index for the frame set.
  // group_mask encodes the available frames (e.g. 0b1111=15 if 4 out of 4 cameras are
  // available, or 0b1101=13 if the second camera frame was dropped for this frame set).
  vrs::DataPieceValue<std::uint64_t> groupId{"group_id"};
  vrs::DataPieceValue<std::uint64_t> groupMask{"group_mask"};

  /// This sample's frame number. Each frame should be +1 of the previous frame.
  /// This is a per-stream quantity, i.e. frames of a frameset may have different frame numbers.
  /// Note: This can reset to zero sometimes, when the underlying HW detects an error, for example.
  vrs::DataPieceValue<std::uint64_t> frameNumber{"frame_number"};

  // Exposure duration in [s].
  vrs::DataPieceValue<double> exposureDuration{"exposure_duration_s"};
  vrs::DataPieceValue<double> gain{"gain"};

  // Mid-exposure timestamp in the hardware clock domain
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};
  // Arrival timestamp in the host clock domain
  vrs::DataPieceValue<std::int64_t> arrivalTimestampNs{"arrival_timestamp_ns"};

  // Set to NaN if not available.
  vrs::DataPieceValue<double> temperature{"temperature_deg_c"};

  vrs::AutoDataLayoutEnd end;
};

struct ImageSensorStateRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;
  vrs::AutoDataLayoutEnd end;
};

} // namespace datalayout
