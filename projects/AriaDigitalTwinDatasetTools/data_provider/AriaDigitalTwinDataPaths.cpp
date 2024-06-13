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

#include "AriaDigitalTwinDataPaths.h"

#define DEFAULT_LOG_CHANNEL "AriaDigitalTwinDataPaths"
#include <logging/Log.h>

#include "AriaDigitalTwinDataFileNames.h"

namespace projectaria::dataset::adt {

std::string AriaDigitalTwinDataPaths::toString() const {
  std::string result;
  result += "--sequenceName: " + sequenceName + "\n";
  result += "--ariaVrsFilePath: " + ariaVrsFilePath + "\n";
  result += "--ariaTrajectoryFilePath: " + ariaTrajectoryFilePath + "\n";
  result += "--objectTrajectoriesFilePath: " + objectTrajectoriesFilePath + "\n";
  result += "--objectBoundingBox3dFilePath: " + objectBoundingBox3dFilePath + "\n";
  result += "--segmentationsFilePath: " + segmentationsFilePath + "\n";
  result += "--depthImagesFilePath: " + depthImagesFilePath + "\n";
  result += "--syntheticVideoFilePath: " + syntheticVrsFilePath + "\n";
  result += "--metaDataFilePath: " + metaDataFilePath + "\n";
  result += "--instancesFilePath: " + instancesFilePath + "\n";
  result += "--boundingBoxes2dFilePath: " + boundingBoxes2dFilePath + "\n";
  result += "--eyeGazesFilePath: " + eyeGazesFilePath + "\n";
  result += "--skeletonsFilePaths: \n";
  for (const auto& [skeletonId, skeletonFilePath] : skeletonsFilePaths) {
    result += "SkeletonId " + std::to_string(skeletonId) + ": " + skeletonFilePath + "\n";
  }
  return result;
}

} // namespace projectaria::dataset::adt
