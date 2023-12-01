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

#include <fstream>
#include <string>
#include <vector>

#include "GlobalPointCloud.h"

namespace {

/// Export GlobalPointCloud 3D points to PLY format
template <class T>
inline bool exportToPly(
    const projectaria::tools::mps::GlobalPointCloud& points,
    const std::string& filename,
    const std::optional<T>& colors = std::nullopt);

template <>
inline bool exportToPly(
    const projectaria::tools::mps::GlobalPointCloud& points,
    const std::string& filename,
    const std::optional<std::vector<float>>& colors) {
  std::ofstream outfile(filename.c_str());
  if (!outfile) {
    return false;
  }

  outfile << "ply" << '\n'
          << "format ascii 1.0" << '\n'
          << "element vertex " << points.size() << '\n'
          << "property double x" << '\n'
          << "property double y" << '\n'
          << "property double z" << '\n'
          << "property uchar red" << '\n'
          << "property uchar green" << '\n'
          << "property uchar blue" << '\n'
          << "end_header"
          << "\n";

  outfile << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1);

  for (size_t i = 0; i < points.size(); ++i) {
    if (colors) {
      outfile << points[i].position_world.x() << ' ' << points[i].position_world.y() << ' '
              << points[i].position_world.z() << ' ' << static_cast<int>((*colors)[i]) << ' '
              << static_cast<int>((*colors)[i]) << ' ' << static_cast<int>((*colors)[i]) << "\n";
    } else {
      // We dont have any colors so we just set them all to white
      outfile << points[i].position_world.x() << ' ' << points[i].position_world.y() << ' '
              << points[i].position_world.z() << ' ' << "255 255 255\n";
    }
  }

  outfile.flush();
  const bool bOk = outfile.good();
  outfile.close();
  return bOk;
}

/// Export 3D point vector and color to PLY format
template <>
inline bool exportToPly(
    const projectaria::tools::mps::GlobalPointCloud& points,
    const std::string& filename,
    const std::optional<std::vector<Eigen::Vector3f>>& colors) {
  std::ofstream outfile(filename.c_str());
  if (!outfile) {
    return false;
  }

  outfile << "ply" << '\n'
          << "format ascii 1.0" << '\n'
          << "element vertex " << points.size() << '\n'
          << "property double x" << '\n'
          << "property double y" << '\n'
          << "property double z" << '\n'
          << "property uchar red" << '\n'
          << "property uchar green" << '\n'
          << "property uchar blue" << '\n'
          << "end_header"
          << "\n";

  outfile << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1);

  for (size_t i = 0; i < points.size(); ++i) {
    if (colors) {
      outfile << points[i].position_world.x() << ' ' << points[i].position_world.y() << ' '
              << points[i].position_world.z() << ' ' << static_cast<int>((*colors)[i].x()) << ' '
              << static_cast<int>((*colors)[i].y()) << ' ' << static_cast<int>((*colors)[i].z())
              << "\n";
    } else {
      // We dont have any colors so we just set them all to white
      outfile << points[i].position_world.x() << ' ' << points[i].position_world.y() << ' '
              << points[i].position_world.z() << ' ' << "255 255 255\n";
    }
  }

  outfile.flush();
  const bool bOk = outfile.good();
  outfile.close();
  return bOk;
}

} // namespace
