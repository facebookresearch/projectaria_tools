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

#include <cstdint>
#include <functional>
#include <string>
#include <utility>

#include <data_layout/ProximitySensorMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

namespace projectaria::tools::data_provider {

struct ProximitySensorConfiguration {
  uint32_t streamId{};
};

struct ProximitySensorData {
  int64_t captureTimestampNs{};
  // Passed through as recorded. The Oatmeal recorder writes "near", "far", "invalid", or
  // "unknown" for a state it doesn't recognize.
  std::string proximityValue;
};

using ProximitySensorCallback = std::function<bool(
    const ProximitySensorData& data,
    const ProximitySensorConfiguration& config,
    bool verbose)>;

class ProximitySensorPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit ProximitySensorPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  ProximitySensorPlayer(const ProximitySensorPlayer&) = delete;
  ProximitySensorPlayer& operator=(const ProximitySensorPlayer&) = delete;
  ProximitySensorPlayer(ProximitySensorPlayer&&) = default;
  ProximitySensorPlayer& operator=(ProximitySensorPlayer&&) = delete;
  ~ProximitySensorPlayer() override = default;

  void setCallback(ProximitySensorCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const ProximitySensorConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const ProximitySensorData& getDataRecord() const {
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
  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& dl)
      override;

  const vrs::StreamId streamId_;
  ProximitySensorCallback callback_ =
      [](const ProximitySensorData&, const ProximitySensorConfiguration&, bool) { return true; };

  ProximitySensorConfiguration configRecord_{};
  ProximitySensorData dataRecord_{};

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

} // namespace projectaria::tools::data_provider
