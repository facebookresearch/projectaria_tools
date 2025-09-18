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

#include <data_provider/data_layout/HandPoseMetadata.h>
#include <mps/HandTracking.h>
#include <vrs/RecordFormatStreamPlayer.h>

#include <utility>

namespace projectaria::tools::data_provider {

struct HandPoseConfiguration {
  uint32_t streamId{};
  double nominalRateHz{};
  // Whether the result is limited to wrist and palm joints.
  bool isWristPalmOnly{};
  std::string userProfile;
};

using HandPoseCallback = std::function<bool(const mps::HandTrackingResult& data)>;

class HandPosePlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit HandPosePlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  HandPosePlayer(const HandPosePlayer&) = delete;
  HandPosePlayer& operator=(const HandPosePlayer&) = delete;
  HandPosePlayer& operator=(HandPosePlayer&) = delete;
  HandPosePlayer(HandPosePlayer&&) = default;

  void setCallback(HandPoseCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const HandPoseConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const mps::HandTrackingResult& getDataRecord() const {
    return dataRecord_;
  }

  [[nodiscard]] const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  [[nodiscard]] double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

 private:
  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& layout)
      override;

  const vrs::StreamId streamId_;
  HandPoseCallback callback_ = [](const mps::HandTrackingResult&) { return true; };

  HandPoseConfiguration configRecord_;
  mps::HandTrackingResult dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = true;
};

} // namespace projectaria::tools::data_provider
