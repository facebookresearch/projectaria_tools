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

#include <optional>

#include <calibration/NeuralBandEmgCalibration.h>
#include <calibration/NeuralBandImuCalibration.h>

namespace projectaria::tools::calibration {

/**
 * @brief Container for the two calibrations that live on a single NeuralBand
 * batch stream. Enables the `SensorCalibration` variant to carry both under
 * one stream — mirroring how `AriaEtCalibration` bundles the two ET-camera
 * calibrations of a single stitched stream.
 *
 * Either `emg_calib` or `imu_calib` may be absent; both being absent means
 * the recording carries no NeuralBand calibration.
 */
struct NeuralBandBatchCalibration {
  std::optional<NeuralBandEmgCalibration> emg_calib;
  std::optional<NeuralBandImuCalibration> imu_calib;
};

} // namespace projectaria::tools::calibration
