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

// Fields present only when the images are encoded as video.
struct VideoConfigurationFields {
  vrs::DataPieceString videoCodecName{vrs::datalayout_conventions::kImageCodecName};
};

struct ImageSensorConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;

  explicit ImageSensorConfigRecordMetadata(bool allocateVideoFields = false)
      : videoConfigurationFields(allocateVideoFields) {}

  // Type, version, serial of *entire* HMD device
  vrs::DataPieceString deviceType{"device_type"};
  vrs::DataPieceString deviceVersion{"device_version"};
  vrs::DataPieceString deviceSerial{"device_serial"};

  // Index of the camera in the calibration JSON
  vrs::DataPieceValue<std::uint32_t> cameraId{"camera_id"};
  vrs::DataPieceValue<std::uint32_t> streamType{"stream_type"};
  vrs::DataPieceValue<std::uint32_t> streamIndex{"stream_index"};

  // Type & serial number of the camera module.
  vrs::DataPieceString sensorModel{"sensor_model"};
  vrs::DataPieceString sensorSerial{"sensor_serial"};

  // Nominal frame rate [Hz], NaN if not available
  vrs::DataPieceValue<double> nominalRateHz{"nominal_rate"};

  // Image description. Part of general VRS data layout conventions.
  vrs::DataPieceValue<ImageSpecType> imageWidth{vrs::datalayout_conventions::kImageWidth};
  vrs::DataPieceValue<ImageSpecType> imageHeight{vrs::datalayout_conventions::kImageHeight};
  vrs::DataPieceValue<ImageSpecType> imageStride{vrs::datalayout_conventions::kImageStride};
  vrs::DataPieceValue<ImageSpecType> imageStride2{vrs::datalayout_conventions::kImageStride2};
  vrs::DataPieceValue<ImageSpecType> pixelFormat{vrs::datalayout_conventions::kImagePixelFormat};
  // Rows between the start of the first plane and the start of the second/third plane.
  vrs::DataPieceValue<std::uint32_t> imagePlane2OffsetRows{"image_plane_2_offset_rows"};
  vrs::DataPieceValue<std::uint32_t> imagePlane3OffsetRows{"image_plane_3_offset_rows"};
  vrs::DataPieceValue<std::uint32_t> imageOrientation{"image_orientation"};
  vrs::DataPieceValue<std::uint32_t> shutterDirection{"shutter_direction"};

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
  vrs::DataPieceString cameraMuxModeName{"camera_mux_mode_name"};

  const vrs::OptionalDataPieces<VideoConfigurationFields> videoConfigurationFields;

  vrs::AutoDataLayoutEnd end;
};

// The field imageBufferSize is not part of the data layout because it is
// redundant since it can be obtained via a call to vrs::ContentBlock::getBlockSize().
// Fields present only when the images are encoded as video: the timestamp of the frame's key
// frame, and the frame's index since it.
struct VideoDataFields {
  vrs::DataPieceValue<double> keyFrameTimestamp{
      vrs::datalayout_conventions::kImageKeyFrameTimeStamp};
  vrs::DataPieceValue<ImageSpecType> keyFrameIndex{
      vrs::datalayout_conventions::kImageKeyFrameIndex};
};

struct ImageSensorDataRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;

  explicit ImageSensorDataRecordMetadata(bool allocateVideoFields = false)
      : videoDataFields(allocateVideoFields) {}

  // The first two entries are used to identify frame groups:
  // group_id is an increasing index for the frame set.
  // group_mask encodes the available frames (e.g. 0b1111=15 if 4 out of 4 cameras are
  // available, or 0b1101=13 if the second camera frame was dropped for this frame set).
  vrs::DataPieceValue<std::uint64_t> groupId{"group_id"};
  vrs::DataPieceValue<std::uint64_t> groupMask{"group_mask"};
  vrs::DataPieceValue<std::uint64_t> streamIndexMask{"stream_index_mask"};

  /// This sample's frame number. Each frame should be +1 of the previous frame.
  /// This is a per-stream quantity, i.e. frames of a frameset may have different frame numbers.
  /// Note: This can reset to zero sometimes, when the underlying HW detects an error, for example.
  vrs::DataPieceValue<std::uint64_t> frameNumber{"frame_number"};
  vrs::DataPieceValue<std::uint32_t> frameTag{"frame_tag"};

  // Exposure duration in [s].
  vrs::DataPieceValue<double> exposureDuration{"exposure_duration_s"};
  vrs::DataPieceValue<double> gain{"gain"};
  vrs::DataPieceValue<double> readoutDuration{"readout_duration_s"};

  // Mid-exposure timestamp in the hardware clock domain
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};
  vrs::DataPieceValue<std::int64_t> captureTimestampInProcessingClockDomainNs{
      "capture_timestamp_in_processing_clock_domain_ns"};
  // Arrival timestamp in the host clock domain
  vrs::DataPieceValue<std::int64_t> arrivalTimestampNs{"arrival_timestamp_ns"};
  vrs::DataPieceValue<std::int64_t> processingStartTimestampNs{"processing_start_timestamp_ns"};

  // Set to NaN if not available.
  vrs::DataPieceValue<double> temperature{"temperature_deg_c"};

  vrs::DataPieceVector<std::uint8_t> imageMetadata{"image_metadata"};

  const vrs::OptionalDataPieces<VideoDataFields> videoDataFields;

  // -1 if not available.
  vrs::DataPieceValue<double> focusDistanceMm{"focus_distance_mm", -1.0};

  vrs::AutoDataLayoutEnd end;
};

struct ImageSensorStateRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;
  vrs::DataPieceValue<std::uint32_t> decimation{"decimation"};
  vrs::AutoDataLayoutEnd end;
};

} // namespace datalayout
