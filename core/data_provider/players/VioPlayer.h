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
#include <data_provider/data_types/FrontendOutput.h>
#include <mps/Trajectory.h>
#include <vrs/RecordFormatStreamPlayer.h>

#include <optional>
#include <utility>

namespace projectaria::tools::data_provider {

struct VioConfiguration {
  uint32_t streamId;
  double nominalRateHz;
  std::string messageVersion;
};

using VioCallback = std::function<bool(const FrontendOutput& data)>;

class VioPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit VioPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  VioPlayer(const VioPlayer&) = delete;
  VioPlayer& operator=(const VioPlayer&) = delete;
  VioPlayer& operator=(VioPlayer&) = delete;
  VioPlayer(VioPlayer&&) = default;

  // The callback will be invoked at the end of each `onDataLayoutRead()` call, when a new VRS
  // Record is being read.
  void setCallback(VioCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const VioConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const FrontendOutput& getDataRecord() const {
    return dataRecord_;
  }

  [[nodiscard]] const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  [[nodiscard]] double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  void setT_BodyImu_Device(const Sophus::SE3f& T_BodyImu_Device) {
    T_BodyImu_Device_ = T_BodyImu_Device;
  }

  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& layout)
      override;

 private:
  bool onVioResultDataLayoutRead(const datalayout::VioResultDataLayout& dataLayout);

  const vrs::StreamId streamId_;
  VioCallback callback_ = [](const FrontendOutput&) { return true; };

  VioConfiguration configRecord_{};
  FrontendOutput dataRecord_;

  double nextTimestampSec_ = 0;
  Sophus::SE3f T_BodyImu_Device_;
};

} // namespace projectaria::tools::data_provider
