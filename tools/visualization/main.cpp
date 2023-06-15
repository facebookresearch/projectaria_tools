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
#include <thread>

#include "AriaPlayer.h"
#include "AriaStreamIds.h"
#include "AriaViewer.h"

using namespace projectaria::tools;

namespace {
using namespace projectaria::tools::data_provider;

const std::vector<vrs::StreamId> kImageStreamIds = {
    kSlamLeftCameraStreamId,
    kSlamRightCameraStreamId,
    kRgbCameraStreamId,
    kEyeCameraStreamId};
const std::vector<vrs::StreamId> kImuStreamIds = {kImuRightStreamId, kImuLeftStreamId};
const std::vector<vrs::StreamId> kDataStreams = {
    kMagnetometerStreamId,
    kBarometerStreamId,
    kAudioStreamId,
    // kWifiStreamId,
    // kBluetoothStreamId,
    // kGpsStreamId
};
} // namespace

int main(int argc, const char* argv[]) {
  CLI::App app{"Aria Sensor Data Visualizer"};
  std::string vrsPath;
  app.add_option("--vrs", vrsPath, "Path to the source vrs file")
      ->check(CLI::ExistingPath)
      ->required();

  CLI11_PARSE(app, argc, argv);

  auto ariaPlayer = createAriaPlayer(vrsPath, kImageStreamIds, kImuStreamIds, kDataStreams);
  if (!ariaPlayer) {
    return EXIT_FAILURE;
  }

  // start viewer with data provider
  std::shared_ptr<AriaViewer> viewer =
      std::make_shared<AriaViewer>(1280, 800, "AriaViewer", ariaPlayer);
  viewer->init();

  std::thread playThread([&]() { ariaPlayer->run(); });

  viewer->run();
  playThread.join();

  return 0;
}
