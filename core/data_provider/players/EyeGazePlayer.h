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

#include <data_provider/data_layout/EyeGazeMetadata.h>
#include <mps/EyeGaze.h>
#include <vrs/RecordFormatStreamPlayer.h>

#include <sophus/se3.hpp>
#include <optional>
#include <utility>

namespace projectaria::tools::data_provider {

struct EyeGazeConfiguration {
  uint32_t streamId;
  double nominalRateHz;
  bool userCalibrated;
  float userCalibrationError;
};

using EyeGazeCallback =
    std::function<bool(const mps::EyeGaze& data, const EyeGazeConfiguration& config, bool verbose)>;

class EyeGazePlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit EyeGazePlayer(
      vrs::StreamId streamId,
      const std::optional<Sophus::SE3f>& maybeT_Cpf_Device = std::nullopt)
      : streamId_(streamId), T_Cpf_Device_(maybeT_Cpf_Device) {}
  EyeGazePlayer(const EyeGazePlayer&) = delete;
  EyeGazePlayer& operator=(const EyeGazePlayer&) = delete;
  EyeGazePlayer& operator=(EyeGazePlayer&) = delete;
  EyeGazePlayer(EyeGazePlayer&&) = default;

  void setCallback(EyeGazeCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const EyeGazeConfiguration& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const mps::EyeGaze& getDataRecord() const {
    return dataRecord_;
  }

  [[nodiscard]] const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  [[nodiscard]] double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  void setT_Cpf_Device(const Sophus::SE3f& T_Cpf_Device) {
    T_Cpf_Device_ = T_Cpf_Device;
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

 private:
  bool onDataLayoutRead(
      const vrs::CurrentRecord& header,
      size_t blockIndex,
      vrs::DataLayout& layout) override;

  const vrs::StreamId streamId_;
  EyeGazeCallback callback_ = [](const mps::EyeGaze&, const EyeGazeConfiguration&, bool) {
    return true;
  };

  EyeGazeConfiguration configRecord_{};
  mps::EyeGaze dataRecord_;

  std::optional<Sophus::SE3f> T_Cpf_Device_ = std::nullopt;
  double nextTimestampSec_ = 0;
  bool verbose_ = true;
};

} // namespace projectaria::tools::data_provider
