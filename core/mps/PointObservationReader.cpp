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

#include "PointObservation.h"

#include "CompressedIStream.h"

#ifndef CSV_IO_NO_THREAD
#define CSV_IO_NO_THREAD
#endif
#include "fast-cpp-csv-parser/csv.h"

#include <array>
#include <iostream>
#include <vector>

namespace projectaria::tools::mps {

constexpr std::array<const char*, 5> kPointObservationColumns = {
    "uid",
    "frame_tracking_timestamp_us",
    "camera_serial",
    "u",
    "v",
};

PointObservations readPointObservations(
    const std::string& path,
    const StreamCompressionMode compression) {
  PointObservations observations;
  try {
    CompressedIStream istream(path, compression);
    io::CSVReader<kPointObservationColumns.size()> csv(path.c_str(), istream);

    // Read in the CSV header
    // allow extra column for future-proof forward compatibility
    const auto readHeader = [&](auto&&... args) {
      csv.read_header(io::ignore_extra_column, args...);
    };
    std::apply(readHeader, kPointObservationColumns);

    uint64_t point_uid;
    std::int64_t frame_tracking_timestamp_us;
    std::string camera_serial;
    Eigen::Vector2f uv;

    while (csv.read_row(point_uid, frame_tracking_timestamp_us, camera_serial, uv.x(), uv.y())) {
      auto& observation = observations.emplace_back();

      observation.pointUid = point_uid;
      observation.frameCaptureTimestamp = std::chrono::microseconds(frame_tracking_timestamp_us);
      observation.cameraSerial = camera_serial;
      observation.uv = uv;
    }

    std::cout << "Loaded #observation records: " << observations.size() << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to parse semi dense observations file: " << e.what() << std::endl;
  }
  return observations;
}
} // namespace projectaria::tools::mps
