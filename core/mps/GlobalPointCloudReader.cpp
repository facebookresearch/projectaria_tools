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

#include "GlobalPointCloudReader.h"

#include "CompressedIStream.h"

#ifndef CSV_IO_NO_THREAD
#define CSV_IO_NO_THREAD
#endif
#include <fast-cpp-csv-parser/csv.h>

#include <array>
#include <filesystem>
#include <iostream>

namespace projectaria::tools::mps {

constexpr std::array<const char*, 7> kGlobalPointCloudColumns =
    {"uid", "graph_uid", "px_world", "py_world", "pz_world", "inv_dist_std", "dist_std"};

GlobalPointCloud readGlobalPointCloud(const std::string& path) {
  namespace fs = std::filesystem;
  if (fs::path(path).extension() == ".csv") {
    return readGlobalPointCloud(path, StreamCompressionMode::NONE);
  } else if (fs::path(path).extension() == ".gz") {
    return readGlobalPointCloud(path, StreamCompressionMode::GZIP);
  }
  return {};
}

GlobalPointCloud readGlobalPointCloud(
    const std::string& path,
    const StreamCompressionMode compression) {
  GlobalPointCloud cloud;
  try {
    CompressedIStream istream(path, compression);
    io::CSVReader<kGlobalPointCloudColumns.size()> csv(path.c_str(), istream);

    // Read in the CSV header
    // allow extra column for future-proof forward compatibility
    const auto readHeader = [&](auto&&... args) {
      csv.read_header(io::ignore_extra_column, args...);
    };
    std::apply(readHeader, kGlobalPointCloudColumns);

    GlobalPointPosition point;

    while (csv.read_row(
        point.uid,
        point.graphUid,
        point.position_world.x(),
        point.position_world.y(),
        point.position_world.z(),
        point.inverseDistanceStd,
        point.distanceStd)) {
      cloud.push_back(point);
    }
    std::cout << "Loaded #3dPoints: " << cloud.size() << std::endl;
  } catch (std::exception& e) {
    std::cerr << "Failed to parse global point cloud file: " << e.what() << std::endl;
  }
  return cloud;
}

} // namespace projectaria::tools::mps
