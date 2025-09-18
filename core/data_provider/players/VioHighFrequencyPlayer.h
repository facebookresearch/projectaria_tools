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

#include <data_layout/VioMetadata.h>
#include <mps/Trajectory.h>
#include <vrs/RecordFormatStreamPlayer.h>

#include <utility>

namespace projectaria::tools::data_provider {

struct VioHighFreqConfiguration {
  uint32_t streamId;
  double nominalRateHz;
  std::string messageVersion;
};

using VioHighFrequencyDataCallback = std::function<
    bool(const mps::OpenLoopTrajectoryPose& data, const VioHighFreqConfiguration& config)>;

class VioHighFrequencyPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit VioHighFrequencyPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  VioHighFrequencyPlayer(const VioHighFrequencyPlayer&) = delete;
  VioHighFrequencyPlayer& operator=(const VioHighFrequencyPlayer&) = delete;
  VioHighFrequencyPlayer& operator=(VioHighFrequencyPlayer&) = delete;
  VioHighFrequencyPlayer(VioHighFrequencyPlayer&&) = default;

  void setHighFrequencyCallback(VioHighFrequencyDataCallback callback) {
    highFrequencyCallback_ = std::move(callback);
  }

  [[nodiscard]] const VioHighFreqConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const mps::OpenLoopTrajectoryPose& getDataRecord() const {
    return dataRecord_;
  }

  [[nodiscard]] const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  [[nodiscard]] double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& layout)
      override;

 private:
  bool onVioHighFrequencyResultDataLayoutRead(
      const datalayout::VioHighFrequencyResultDataLayout& dataLayout);
  static bool onVioResultDataLayoutRead(const datalayout::VioResultDataLayout& dataLayout);

  const vrs::StreamId streamId_;
  VioHighFrequencyDataCallback highFrequencyCallback_ =
      [](const mps::OpenLoopTrajectoryPose&, const VioHighFreqConfiguration&) { return true; };

  VioHighFreqConfiguration configRecord_{};
  mps::OpenLoopTrajectoryPose dataRecord_;

  double nextTimestampSec_ = 0;
};

} // namespace projectaria::tools::data_provider
