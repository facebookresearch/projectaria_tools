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

#include "AriaStreamIds.h"
#include "GlobalPointCloudFilter.h"
#include "GlobalPointCloudReader.h"
#include "PointObservationReader.h"
#include "TrajectoryReaders.h"

#include <data_provider/VrsDataProvider.h>

#include <CLI/CLI.hpp>

#include <cstdint>
#include <filesystem>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "GlobalPointCloudToPly.h"
#include "RGBPointCloudColorizer.h"
#include "SLAMPointCloudColorizer.h"

using namespace projectaria::tools;
using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::image;
using namespace projectaria::tools::mps;

int main(int argc, const char* argv[]) {
  std::string closedLoopTrajPath;
  std::string globalPointCloudPath;
  std::string globalPointObservationPath;
  std::string vrsPath;
  bool doNotFilter = false;
  bool useProjectionForGrayColoring = false;

  CLI::App app{"Point Cloud Colorization samples"};

  app.add_option(
         "--closed-loop-traj", closedLoopTrajPath, "Input closed loop trajectory file path.")
      ->check(CLI::ExistingPath)
      ->required();
  app.add_option(
         "--global-point-cloud", globalPointCloudPath, "Input global point cloud file path.")
      ->check(CLI::ExistingPath)
      ->required();
  app.add_option(
         "--global-point-cloud-observations",
         globalPointObservationPath,
         "Input global point cloud observations.")
      ->check(CLI::ExistingPath)
      ->required();
  app.add_option("--vrs", vrsPath, "Input vrs file.")->check(CLI::ExistingPath)->required();
  app.add_flag(
      "--use-projection-for-gray-coloring",
      useProjectionForGrayColoring,
      "By default GRAY coloring is using stored (u,v) feature coordinates since it is faster,\n"
      "this options allows you to use 3D point projection instead.");
  app.add_flag("--do-not-filter", doNotFilter, "Do not filter point cloud");

  CLI11_PARSE(app, argc, argv);

  // Load closed loop trajectories
  const auto trajectory = readClosedLoopTrajectory(closedLoopTrajPath);

  // Load point cloud (can be .csv or .gz)
  auto ptCloud = readGlobalPointCloud(globalPointCloudPath);

  auto pointObservations = readPointObservations(globalPointObservationPath);

  // Open the VRS File
  auto dataProvider = data_provider::createVrsDataProvider(vrsPath);

  if (!doNotFilter) {
    ptCloud = filterPointsFromConfidence(ptCloud);
  }

  // SLAM based image colorization
  {
    std::cout << "-- Perform SLAM based image point cloud colorization --" << std::endl;
    SLAMPointCloudColorizer pointCloudColorizer(
        trajectory, ptCloud, pointObservations, dataProvider, useProjectionForGrayColoring);
    pointCloudColorizer.Colorize();
    std::cout << "Saving PLY file." << std::endl;
    exportToPly(ptCloud, "gray.ply", pointCloudColorizer.getGray());
  }

  // RGB, SLAM based image colorization
  {
    std::cout << "-- Perform RGB based image point cloud colorization --" << std::endl;
    RGBPointCloudColorizer pointCloudColorizer(
        trajectory, ptCloud, pointObservations, dataProvider);
    pointCloudColorizer.Colorize();
    std::cout << "Saving PLY file." << std::endl;
    exportToPly(ptCloud, "rgb.ply", pointCloudColorizer.getRGB());
  }

  return EXIT_SUCCESS;
}
