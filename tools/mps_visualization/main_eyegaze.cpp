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
#include <string>

#include "AriaStreamIds.h"
#include "EyeGazeAriaPlayer.h"
#include "EyeGazeAriaViewer.h"
#include "EyeGazeAriaVisualizationData.h"

using namespace projectaria::tools;

namespace {
using namespace projectaria::tools::data_provider;

const std::vector<vrs::StreamId> kImageStreamIds = {
    kEyeCameraStreamId,
    kRgbCameraStreamId,
    kSlamLeftCameraStreamId,
    kSlamRightCameraStreamId};
} // namespace

int main(int argc, const char* argv[]) {
  CLI::App app{"MPS Eye Gaze Visualizer"};
  std::string vrsPath;
  std::string eyegazePath;
  app.add_option("--vrs", vrsPath, "Path to the source vrs file")
      ->check(CLI::ExistingPath)
      ->required();
  app.add_option("--eyegaze", eyegazePath, "Path to the eye gaze csv file")
      ->check(CLI::ExistingPath)
      ->required();

  CLI11_PARSE(app, argc, argv);

  std::shared_ptr<EyeGazeAriaPlayer> eyegazeAriaPlayer =
      createEyeGazeAriaPlayer(vrsPath, eyegazePath, kImageStreamIds);
  if (!eyegazeAriaPlayer) {
    return EXIT_FAILURE;
  }

  // create eye gaze aria viewer
  std::shared_ptr<EyeGazeAriaViewer> viewer =
      std::make_shared<EyeGazeAriaViewer>(eyegazeAriaPlayer, 1200, 600);
  viewer->init();

  std::thread playThread([&]() { eyegazeAriaPlayer->run(); });

  viewer->run();
  playThread.join();
  return 0;
}
