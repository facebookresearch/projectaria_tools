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

#include <data_layout/GpsMetadata.h>
#include <vrs/RecordFormatStreamPlayer.h>

#include <utility>

namespace projectaria::tools::data_provider {

/**
 * @brief Gps sensor configuration type
 */
struct GpsConfigRecord {
  uint32_t streamId; ///< @brief ID of the VRS stream, 0 to N
  double sampleRateHz; ///< @brief the number of data collected per second
};

/**
 * Gps data type, note that GPS sensor data are already rectified
 */
struct GpsData {
  int64_t captureTimestampNs; ///< @brief capture time in device domain
  int64_t utcTimeMs; ///< @brief capture time in UTC domain
  std::string provider; ///< @brief GPS provider
  float latitude; ///< @brief latitude of the position in Degrees Minutes Seconds (DMS)
  float longitude; ///< @brief longitude of the position in Degrees Minutes Seconds (DMS)
  float altitude; ///< @brief altitude of the position
  float accuracy; ///< @brief horizontal accuracy of the position in meters
  float verticalAccuracy; ///< @brief vertical accuracy of the position in meters
  float speed;
  std::vector<std::string> rawData;
};

using GpsCallback =
    std::function<bool(const GpsData& data, const GpsConfigRecord& config, bool verbose)>;

class GpsPlayer : public vrs::RecordFormatStreamPlayer {
 public:
  explicit GpsPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  GpsPlayer(const GpsPlayer&) = delete;
  GpsPlayer& operator=(const GpsPlayer&) = delete;
  GpsPlayer& operator=(GpsPlayer&) = delete;
  GpsPlayer(GpsPlayer&&) = default;

  void setCallback(GpsCallback callback) {
    callback_ = std::move(callback);
  }

  [[nodiscard]] const GpsConfigRecord& getConfigRecord() const {
    return configRecord_;
  }

  [[nodiscard]] const GpsData& getDataRecord() const {
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
  GpsCallback callback_ = [](const GpsData&, const GpsConfigRecord&, bool) { return true; };

  GpsConfigRecord configRecord_;
  GpsData dataRecord_;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
};

} // namespace projectaria::tools::data_provider
