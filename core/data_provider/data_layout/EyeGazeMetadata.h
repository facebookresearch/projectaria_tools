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
#include <vrs/DataLayoutConventions.h>

namespace projectaria::tools::datalayout {

// 2 bits per slot on the packed uint32. Numeric values are the on-disk
// wire format and must not be renumbered.
enum class FieldProvenance : uint8_t {
  SUPPORTED = 0, // direct sensor/model output
  CALCULATED = 1, // derived on-device from other valid fields
  HARDCODED = 2, // constant from device calibration; valid==true but does not track state
  NOT_PRODUCED = 3, // source does not populate the field; valid==false
};

// Slot indices (2 bits each) inside EyeGazeConfigurationLayout::fieldProvenanceSingle.
enum class SingleFieldId : uint32_t {
  GazeOrigin = 0,
  GazeDirection = 1,
  EntrancePupilPosition = 2,
  PupilDiameter = 3,
  Blink = 4,
};

// Slot indices (2 bits each) inside EyeGazeConfigurationLayout::fieldProvenanceCombined.
enum class CombinedFieldId : uint32_t {
  GazeOriginCombined = 0,
  GazeDirectionCombined = 1,
  ConvergenceDistance = 2,
  InterocularDistance = 3,
  FoveatedGaze = 4,
  SpatialGazePoint = 5,
};

template <typename FieldId>
constexpr FieldProvenance getFieldProvenance(uint32_t packed, FieldId id) {
  const auto shift = static_cast<uint32_t>(id) * 2u;
  return static_cast<FieldProvenance>((packed >> shift) & 0b11u);
}

template <typename FieldId>
constexpr uint32_t setFieldProvenance(uint32_t packed, FieldId id, FieldProvenance p) {
  const auto shift = static_cast<uint32_t>(id) * 2u;
  return (packed & ~(0b11u << shift)) | ((static_cast<uint32_t>(p) & 0b11u) << shift);
}

class EyeGazeConfigurationLayout : public vrs::AutoDataLayout {
 public:
  // v2 adds user_calibration_params_json + field_provenance_{single,combined};
  // VRS field mapping is name-based so older readers skip transparently.
  static constexpr uint32_t kVersion = 2;
  vrs::DataPieceValue<uint32_t> streamId{"stream_id"};
  // Algorithm version to distinguish different eye tracking implementations. Formatted in string
  // as "Major.minor",
  vrs::DataPieceString algorithmVersion{"algorithm_version"};
  vrs::DataPieceString algorithmName{"algorithm_name"};

  vrs::DataPieceValue<double> nominalRateHz{"nominal_rate_hz"};

  // Whether user eyetracking calibration is done before the recording
  vrs::DataPieceValue<vrs::Bool> userCalibrated{"user_calibrated"};
  // Indicates the accuracy of the user ET calibration. Lower is better
  vrs::DataPieceValue<float> userCalibrationError{"user_calibration_error"};

  // v2: verbatim calibration_params.json for the ML eye-tracking source.
  // Empty for the geometric source or when userCalibrated == false.
  vrs::DataPieceString userCalibrationParamsJson{"user_calibration_params_json"};
  // v2: per-field provenance for per-eye fields on SingleEyeGazeLayoutStruct.
  // Packed 2 bits/slot, indexed by SingleFieldId. Applies to both eyes.
  vrs::DataPieceValue<uint32_t> fieldProvenanceSingle{"field_provenance_single"};
  // v2: per-field provenance for combined/whole-frame fields on EyeGazeLayout.
  // Packed 2 bits/slot, indexed by CombinedFieldId.
  vrs::DataPieceValue<uint32_t> fieldProvenanceCombined{"field_provenance_combined"};

  vrs::AutoDataLayoutEnd endLayout;
};

struct SingleEyeGazeLayoutStruct : public vrs::DataLayoutStruct {
  DATA_LAYOUT_STRUCT(SingleEyeGazeLayoutStruct)

  vrs::DataPieceValue<vrs::Bool> gazeOriginValid{"gaze_origin_valid"};
  // Gaze origin in the device coordinate frame
  vrs::DataPieceValue<vrs::Point3Df> gazeOriginInDeviceMeterXyz{"gaze_origin_in_device_meter_xyz"};
  vrs::DataPieceValue<vrs::Bool> gazeDirectionValid{"gaze_direction_valid"};
  // Gaze direction in the device coordinate frame
  vrs::DataPieceValue<vrs::Point3Df> gazeDirectionInDeviceNormalizedXyz{
      "gaze_direction_in_device_normalized_xyz"};
  vrs::DataPieceValue<vrs::Bool> entrancePupilPositionValid{"entrance_pupil_position_valid"};
  vrs::DataPieceValue<vrs::Point3Df> entrancePupilPositionInDeviceMeterXyz{
      "entrance_pupil_position_in_device_meter_xyz"};
  vrs::DataPieceValue<vrs::Bool> pupilDiameterValid{"pupil_diameter_valid"};
  vrs::DataPieceValue<float> pupilDiameterMeter{"pupil_diameter_meter"};
  vrs::DataPieceValue<vrs::Bool> blinkValid{"blink_valid"};
  vrs::DataPieceValue<vrs::Bool> blink{"blink"};
};

class EyeGazeLayout : public vrs::AutoDataLayout {
 public:
  static constexpr uint32_t kVersion = 1;
  vrs::DataPieceValue<std::int64_t> trackerTimestampNs{"tracker_timestamp_ns"};
  vrs::DataPieceValue<std::int64_t> systemTimestampNs{"system_timestamp_ns"};

  SingleEyeGazeLayoutStruct leftEye{"left_eye"};
  SingleEyeGazeLayoutStruct rightEye{"right_eye"};

  // Combined gaze origin (from both left/right eye) in device coordinate frame
  vrs::DataPieceValue<vrs::Bool> gazeOriginCombinedInDeviceValid{
      "gaze_origin_combined_in_device_valid"};
  vrs::DataPieceValue<vrs::Point3Df> gazeOriginCombinedInDeviceMeterXyz{
      "gaze_origin_combined_in_device_meter_xyz"};
  // Combined gaze direction (from both left/right eye) in device coordinate frame
  vrs::DataPieceValue<vrs::Bool> gazeDirectionCombinedInDeviceValid{
      "gaze_direction_combined_in_device_valid"};
  vrs::DataPieceValue<vrs::Point3Df> gazeDirectionCombinedInDeviceNormalizedXyz{
      "gaze_direction_combined_in_device_normalized_xyz"};

  // Convergence distance used to compute the gaze vergence point (in Device coordinate frame)
  // using: gazeOriginCombinedInDeviceMeterXyz + convergenceDistanceMeter *
  // gazeDirectionCombinedInDeviceNormalizedXyz
  vrs::DataPieceValue<vrs::Bool> convergenceDistanceValid{"convergence_distance_valid"};
  vrs::DataPieceValue<float> convergenceDistanceMeter{"convergence_distance_meter"};
  // Distance between left/right eye pupils
  vrs::DataPieceValue<vrs::Bool> interocularDistanceValid{"interocular_distance_valid"};
  vrs::DataPieceValue<float> interocularDistanceMeter{"interocular_distance_meter"};

  vrs::DataPieceValue<vrs::Bool> foveatedGazeEnabled{"foveated_gaze_enabled"};
  vrs::DataPieceValue<uint8_t> foveatedGazeTrackingStateType{"foveated_gaze_tracking_state_type"};
  vrs::DataPieceValue<vrs::Point3Df> foveatedGazeDirectionCombinedInDeviceNormalizedXyz{
      "foveated_gaze_direction_combined_in_device_normalized_xyz"};

  vrs::DataPieceValue<vrs::Bool> spatialGazePointValid{"spatial_gaze_point_valid"};
  vrs::DataPieceValue<vrs::Point3Df> spatialGazePointInDeviceMeterXyz{
      "spatial_gaze_point_in_device_meter_xyz"};

  vrs::AutoDataLayoutEnd endLayout;
};

} // namespace projectaria::tools::datalayout
