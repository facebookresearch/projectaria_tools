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

#include <chrono>
#include <iostream>
#include <set>

#include "AriaDigitalTwinDataTypes.h"

namespace projectaria::dataset::adt {

class HighResolutionTimer {
 public:
  using Clock = std::chrono::high_resolution_clock;

  HighResolutionTimer() {
    start_ = Clock::now();
  }

  void reset() {
    start_ = Clock::now();
  }

  int64_t elapsedInNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start_).count();
  }

  int64_t elapsedInUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_).count();
  }

  double elapsedInMs() {
    return static_cast<double>(elapsedInNs()) * 1e-6;
  }

  double elapsedInS() {
    return static_cast<double>(elapsedInNs()) * 1e-9;
  }

  double elapsedInHz() {
    return 1.0 / elapsedInS();
  }

 private:
  std::chrono::time_point<Clock> start_;
};

/**
 * @brief helper function for querying an ordered timestamps set
 * @param tsSet some set ordered by int64_t timestamp
 * @param ts query time where ts must have the same time units as tsSet
 * @param timeQueryOptions
 * (1) Closest: returns the closest timestamp. This will always return a value even if querying
 * before or after data range
 * (2) Before: returns the timestamp right before (or equal to) ts. If ts < start, then we return
 * invalid result
 * (3) After: returns the timestamp right after (or equal to) ts. If ts > end, we
 * return  an invalid result
 */
std::set<int64_t>::const_iterator queryTimestampsSet(
    const std::set<int64_t>& tsSet,
    int64_t ts,
    const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest);

/**
 * @brief helper function to convert a 2D bounding box [xmin, xmax, ymin, ymax] to the 4 image
 * coordinates [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
 */
std::vector<Eigen::Vector2d> bbox2dToImageCoordinates(const Eigen::Vector4f& boxRange);

/**
 * @brief helper function to convert a 2D bounding box [xmin, xmax, ymin, ymax] to the 5 coordinates
 * that draw the lines in the image completing the full bounding box [(x1, y1), (x2, y2), (x3, y3),
 * (x4, y4), (x1, y1)]
 */
std::vector<Eigen::Vector2d> bbox2dToImageLineCoordinates(const Eigen::Vector4f& boxRange);

/**
 * @brief helper function to convert a 3d bounding box [xmin, xmax, ymin, ymax, zmin, zmax] to the 8
 * corner coordinates in the object frame [b1, b2, b3, b4, t1, t2, t3, t4] where b is for bottom and
 * t is for top
 */
std::vector<Eigen::Vector3d> bbox3dToCoordinates(const Vector6d& bbox);

/**
 * @brief helper function to convert a 3d bounding box [xmin, xmax, ymin, ymax, zmin, zmax] to the
 * 16 coordinates that draw the lines completing the full bounding box [b1, b2, b3, b4, b1, t1, t2,
 * t3, t4, t1, t2, b2, b3, t3, t4, b4] where b is for bottom and t is for top
 */
std::vector<Eigen::Vector3d> bbox3dToLineCoordinates(const Vector6d& bbox);

void logErrorAndThrow(const std::string& message);
} // namespace projectaria::dataset::adt
