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

#include <CLI/CLI.hpp>

#include "AriaDigitalTwinDataPathsProvider.h"
#include "AriaDigitalTwinDataTypes.h"
#include "AriaDigitalTwinViewer.h"

#define DEFAULT_LOG_CHANNEL "AriaDigitalTwinViewerMain"
#include <logging/Log.h>

using namespace projectaria::dataset::adt;

int main(int argc, const char* argv[]) {
  std::string sequencePath;
  std::string renderPath;
  bool skeletonFlag = false;

  CLI::App app{"Aria Digital Twin Viewer"};

  app.add_option(
         "--sequence-path",
         sequencePath,
         "Path to the recording sequence containing all digital twin datasets. "
         "This should contain per-device folders, which further contains the actual data files")
      ->required();
  app.add_option(
         "--skeleton-flag",
         skeletonFlag,
         "A boolean value to indicate whether to include skeleton ground-truth in 2D bounding box, "
         " depth map, segmentation with or without human occlusion. By default set to False.")
      ->default_val(false);
  app.add_option(
      "--render-path",
      renderPath,
      "If this path is set, then the viewer will run in headless mode and render all frames to the this path.");

  app.footer(fmt::format(
      "------ Aria Digital Twin Viewer Help ------\n"
      "Summary: \n"
      "This viewer loads and displays all Aria Digital Twin data from a single device "
      "in a data collection sequence. This includes displaying images with GT 2D/3D "
      "bounding boxes and eye gaze projected into the images. We also optionally display "
      "skeleton projected into the images, and display with segmentation and depth images "
      "if they exist in the input GT folder (i.e., if the user decided to download this "
      "data)\n\n"));

  CLI11_PARSE(app, argc, argv);

  AriaDigitalTwinDataPathsProvider dataPathsProvider(sequencePath);

  fmt::print("--------------------Sequence Information----------------------\n");
  fmt::print("Data sequence path: {}\n", sequencePath);
  fmt::print("Aria serial: {}\n", dataPathsProvider.getDeviceSerialNumber());
  fmt::print("scene: {}\n", dataPathsProvider.getSceneName());
  fmt::print("multi-person sequence? {}\n", dataPathsProvider.isMultiPerson());
  fmt::print("has skeleton gt? {} skeletons\n", dataPathsProvider.getNumSkeletons());
  fmt::print("--------------------------------------------------------------\n");

  fmt::print("Set SkeletonFlag to {}\n", skeletonFlag);

  std::optional<AriaDigitalTwinDataPaths> dataPaths = dataPathsProvider.getDataPaths(skeletonFlag);
  if (!dataPaths.has_value()) {
    std::cerr << "unable to get datapaths, check your inputs" << std::endl;
    return EXIT_FAILURE;
  }

  fmt::print("-------------------------Data Paths---------------------------\n");
  std::cout << dataPaths.value().toString();
  fmt::print("--------------------------------------------------------------\n");

  // get and open data provider
  AriaDigitalTwinViewer viewer(dataPaths.value(), renderPath);
  viewer.run();

  fmt::print("Visualizer finished cleanly\n");
  return EXIT_SUCCESS;
}
