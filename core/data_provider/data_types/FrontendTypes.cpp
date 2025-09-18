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

#include <data_provider/data_types/FrontendTypes.h>

namespace projectaria::tools::data_provider {

std::string toString(const VioStatus status) {
  switch (status) {
    case VioStatus::VALID:
      return "VALID";
    case VioStatus::FILTER_NOT_INITIALIZED:
      return "FILTER_NOT_INITIALIZED";
    case VioStatus::INVALID:
      return "INVALID";
    default:
      throw std::runtime_error("Unknown VioStatus enum");
  }
}

std::string toString(const TrackingQuality trackingQuality) {
  switch (trackingQuality) {
    case TrackingQuality::UNKNOWN:
      return "UNKNOWN";
    case TrackingQuality::GOOD:
      return "GOOD";
    case TrackingQuality::BAD:
      return "BAD";
    case TrackingQuality::UNRECOVERABLE:
      return "UNRECOVERABLE";
    default:
      throw std::runtime_error("Unknown TrackingQuality enum");
  }
}

std::string toString(const VisualTrackingQuality trackingQuality) {
  switch (trackingQuality) {
    case VisualTrackingQuality::UNKNOWN:
      return "UNKNOWN";
    case VisualTrackingQuality::GOOD:
      return "GOOD";
    case VisualTrackingQuality::BAD:
      return "BAD";
    default:
      throw std::runtime_error("Unknown VisualTrackingQuality enum");
  }
}

} // namespace projectaria::tools::data_provider
