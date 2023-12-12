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

#include "VrsHealthCheck.h"

#include <string>

namespace projectaria::tools::vrs_check {

class Utils {
 public:
  // Log score to the console using an ANSI color scheme
  static void enableColoredText(bool doColor) {
    Utils::doColor_ = doColor;
  }
  static void logScore(const std::string& streamName, float score, float minScore);
  // Print a progress bar as an ASCII drawing
  static void printBar(const std::string& name, float progress);
  static int runVrsHealthCheck(int argc, char* argv[]);

  static int runVrsHealthCheck(
      const std::string& path,
      const std::string& jsonOutFilename = {},
      Settings settings = Settings{},
      const std::string& droppedOutFilename = {},
      bool printStats = false,
      bool disableLogging = false);

  static constexpr char kGreenStr[] = "\033[0;32m";
  static constexpr char kYellowStr[] = "\033[0;33m";
  static constexpr char kRedStr[] = "\033[0;31m";
  static constexpr char kResetStr[] = "\033[0m";
  static constexpr char kMoveUpStr[] = "\x1b[A";
  static constexpr char kClearStr[] = "\r\x1b[2K";

 private:
  static bool doColor_;
  static constexpr int kProgressBarMaxLength = 60;
  static constexpr int kMinWidth = 10;
  static constexpr int kProgressBarPadding = 30;
  static constexpr char kColumnsEnv[] = "COLUMNS";
};

} // namespace projectaria::tools::vrs_check
