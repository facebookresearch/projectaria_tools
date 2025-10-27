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

#include "AriaDigitalTwinUtils.h"

#define DEFAULT_LOG_CHANNEL "AriaDigitalTwinDataProvider"
#include <logging/Log.h>

#include <cstdint>

namespace projectaria::dataset::adt {

std::set<int64_t>::const_iterator queryTimestampsSet(
    const std::set<int64_t>& tsSet,
    int64_t ts,
    const TimeQueryOptions& timeQueryOptions) {
  // first check if exact match exists
  auto itExact = tsSet.find(ts);
  if (itExact != tsSet.end()) {
    return itExact;
  }

  // check if outside bounds
  if (ts < *tsSet.begin()) {
    return timeQueryOptions == TimeQueryOptions::Before ? tsSet.end() : tsSet.begin();
  } else if (ts > *tsSet.rbegin()) {
    return timeQueryOptions == TimeQueryOptions::After ? tsSet.end() : std::prev(tsSet.end());
  }

  // else get before and after
  auto iterAfter = tsSet.lower_bound(ts);
  auto iterBefore = std::prev(iterAfter);

  switch (timeQueryOptions) {
    case TimeQueryOptions::Closest:
      if (std::abs(*iterBefore - ts) < std::abs(*iterAfter - ts)) {
        return iterBefore;
      } else {
        return iterAfter;
      }
      break;
    case TimeQueryOptions::After:
      return iterAfter;
      break;
    case TimeQueryOptions::Before:
      return iterBefore;
      break;
    default:
      throw std::runtime_error{"invalid timeQueryOptions"};
  }
}

std::vector<Eigen::Vector2d> bbox2dToImageCoordinates(const Eigen::Vector4f& boxRange) {
  return {
      {boxRange[0], boxRange[2]},
      {boxRange[1], boxRange[2]},
      {boxRange[1], boxRange[3]},
      {boxRange[0], boxRange[3]}};
}

std::vector<Eigen::Vector3d> bbox3dToCoordinates(const Vector6d& bbox) {
  return {
      // bottom
      {bbox[0], bbox[2], bbox[4]},
      {bbox[0], bbox[3], bbox[4]},
      {bbox[1], bbox[3], bbox[4]},
      {bbox[1], bbox[2], bbox[4]},
      // top
      {bbox[0], bbox[2], bbox[5]},
      {bbox[0], bbox[3], bbox[5]},
      {bbox[1], bbox[3], bbox[5]},
      {bbox[1], bbox[2], bbox[5]}};
  ;
}

std::vector<Eigen::Vector2d> bbox2dToImageLineCoordinates(const Eigen::Vector4f& boxRange) {
  return {
      {boxRange[0], boxRange[2]},
      {boxRange[1], boxRange[2]},
      {boxRange[1], boxRange[3]},
      {boxRange[0], boxRange[3]},
      {boxRange[0], boxRange[2]}};
}

std::vector<Eigen::Vector3d> bbox3dToLineCoordinates(const Vector6d& bbox) {
  return {
      // bottom
      {bbox[0], bbox[2], bbox[4]},
      {bbox[0], bbox[3], bbox[4]},
      {bbox[1], bbox[3], bbox[4]},
      {bbox[1], bbox[2], bbox[4]},
      {bbox[0], bbox[2], bbox[4]},
      // top
      {bbox[0], bbox[2], bbox[5]},
      {bbox[0], bbox[3], bbox[5]},
      {bbox[1], bbox[3], bbox[5]},
      {bbox[1], bbox[2], bbox[5]},
      {bbox[0], bbox[2], bbox[5]},
      // side
      {bbox[0], bbox[3], bbox[5]},
      {bbox[0], bbox[3], bbox[4]},
      {bbox[1], bbox[3], bbox[4]},
      {bbox[1], bbox[3], bbox[5]},
      {bbox[1], bbox[2], bbox[5]},
      {bbox[1], bbox[2], bbox[4]}};
}

void logErrorAndThrow(const std::string& message) {
  XR_LOGE("{}", message);
  throw std::runtime_error(message);
}

} // namespace projectaria::dataset::adt
