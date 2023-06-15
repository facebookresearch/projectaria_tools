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
#include <calibration/CameraCalibration.h>

namespace projectaria::dataset::ase {
using CameraCalibration = projectaria::tools::calibration::CameraCalibration;

// returns camera calibration for ASE RGB images, extrinsic is set to SE3{}
// supported image sizes are 704x704 or 1408x1408
CameraCalibration getAseRgbCalibration(const size_t imageSize = 704);

} // namespace projectaria::dataset::ase
