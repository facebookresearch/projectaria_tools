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

#include "VrsHealthCheck.h"

#include <iomanip>
#include <iostream>

#if defined(_WIN32)
#include <io.h>
#define ISATTY _isatty
#define FILENO _fileno
#else
#include <unistd.h>
#define ISATTY isatty
#define FILENO fileno
#endif

#include <Utils.h>

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:Utils"
#include <CLI/CLI.hpp>
#include <logging/Log.h>
#include <logging/LogLevel.h>

#include "Logging.h"

namespace projectaria::tools::vrs_check {

bool Utils::doColor_ = true;

void Utils::logScore(const std::string& streamName, const float score, const float minScore) {
  std::cout << streamName << ": score ";
  std::cout << std::setprecision(3) << std::fixed;
  const char* beginColor = "";
  const char* endColor = "";
  if (Utils::doColor_) {
    // Show warning color if below halfway point between minScore and 100%
    if (score >= (minScore + 100.0) / 2) {
      beginColor = kGreenStr;
    } else if (score >= minScore) {
      beginColor = kYellowStr;
    } else {
      beginColor = kRedStr;
    }
    endColor = kResetStr;
  }
  std::cout << beginColor << score << "%" << endColor << std::endl;
}

void Utils::printBar(const std::string& streamName, float progress) {
  int progressBarLength = kProgressBarMaxLength;
  if (const char* envCols = std::getenv(kColumnsEnv)) {
    const int tempLength = std::stoi(envCols) - kProgressBarPadding - 5;
    if (tempLength > kMinWidth) {
      progressBarLength = tempLength;
    }
  }
  if (!std::isfinite(progress)) {
    progress = 1.0;
  }
  const int progInt = std::round(progressBarLength * progress);
  std::cout << streamName;
  if (kProgressBarPadding > streamName.length()) {
    for (int i = 0; i < kProgressBarPadding - streamName.length(); i++) {
      std::cout << " ";
    }
  }
  std::cout << " [" << kGreenStr;
  for (int i = 0; i < progressBarLength; i++) {
    if (i < progInt - 1 || progInt >= progressBarLength - 1) {
      std::cout << "=";
    } else if (i == progInt - 1) {
      std::cout << ">";
    } else {
      std::cout << " ";
    }
  }
  std::cout << kResetStr << std::fixed << std::setprecision(1);
  std::cout << "] " << 100 * progress << "%" << std::endl;
}

// Wrapper for pybind for CLI options to be parsed in python code
int Utils::runVrsHealthCheck(
    const std::string& path,
    const std::string& jsonOutFilename,
    Settings settings,
    const std::string& droppedOutFilename,
    bool printStats,
    bool disableLogging) {
  if (disableLogging) {
    arvr::logging::setGlobalLogLevel(arvr::logging::Level::Disabled);
    // Setting the stream in fail state will make it silently discard any output, until the failbit
    // is cleared.
    std::cout.setstate(std::ios::failbit);
  } else {
    arvr::logging::setGlobalLogLevel(arvr::logging::Level::Debug);
  }

  settings.isInteractive = ISATTY(FILENO(stdout)); // else it's a file or pipe

  // settings related to camera roi check
  settings.cameraCheckSettings = {
      {vrs::RecordableTypeId::SlamCameraData, CameraCheckSetting{"slam"}},
      {vrs::RecordableTypeId::EyeCameraRecordableClass, CameraCheckSetting{"eyetracking"}},
      {vrs::RecordableTypeId::RgbCameraRecordableClass, CameraCheckSetting{"rgb"}},
  };

  VrsHealthCheck healthCheck(settings);
  if (!healthCheck.setup(path)) {
    return EXIT_FAILURE;
  }
  if (!healthCheck.run()) {
    return EXIT_FAILURE;
  }
  if (printStats) {
    healthCheck.logStats();
  }
  if (!jsonOutFilename.empty()) {
    healthCheck.logStatsJson(jsonOutFilename);
  }
  if (!droppedOutFilename.empty()) {
    healthCheck.logDroppedFrames(droppedOutFilename);
  }

  return healthCheck.getResult() ? EXIT_SUCCESS : EXIT_FAILURE;
}

int Utils::runVrsHealthCheck(int argc, char* argv[]) {
  CLI::App app{"VrsHealthCheckTool"};
  Settings settings;
  std::string path;
  bool printStats = false;
  bool debug = false;
  bool verbose = false;
  std::string jsonOutFilename;
  std::string droppedOutFilename;

  std::string path_desc = "VRS file location (or directory)";
  app.add_option("--path", path, path_desc)->required();

  app.add_option("--min-imu-score", settings.minImuScore, "Minimum score for IMUs");
  app.add_option("--min-camera-score", settings.minCameraScore, "Minimum score for cameras");
  app.add_option("--min-baro-score", settings.minBaroScore, "Minimum score for barometers");
  app.add_option("--min-audio-score", settings.minAudioScore, "Minimum score for audio");
  app.add_option(
      "--min-time-domain-mapping-score",
      settings.minTimeDomainMappingScore,
      "Minimum score for time domain mapping");
  app.add_option("--imu-period", settings.defaultImuPeriodUs, "Default IMU period in microseconds");
  app.add_option("--max-imu-skip", settings.maxImuSkipUs, "Tolerated sequential IMU skip in us");
  app.add_option(
      "--max-frame-drop-us", settings.maxFrameDropUs, "Tolerated sequential frame drop in us");
  app.add_option("--min-gps-accuracy", settings.minGpsAccuracy, "Minimum GPS accuracy in meters");
  app.add_option(
      "--min-alignment-score", settings.minAlignmentScore, "Minimum sensor alignment score");
  app.add_flag("--ignore-gps", settings.ignoreGps, "Ignore GPS errors");
  app.add_option("--default-gps-rate-hz", settings.defaultGpsRateHz, "Default GPS rate in Hz");
  app.add_flag("--ignore-audio", settings.ignoreAudio, "Ignore audio errors");
  app.add_flag("--ignore-bluetooth", settings.ignoreBluetooth, "Ignore bluetooth errors");

  // settings related to camera roi check
  settings.cameraCheckSettings = {
      {vrs::RecordableTypeId::SlamCameraData, CameraCheckSetting{"slam"}},
      {vrs::RecordableTypeId::EyeCameraRecordableClass, CameraCheckSetting{"eyetracking"}},
      {vrs::RecordableTypeId::RgbCameraRecordableClass, CameraCheckSetting{"rgb"}},
  };
  // temporary data structure to store the roi ranges
  std::unordered_map<std::string, std::vector<int>> cameraRoiRanges = {
      {"slam", std::vector<int>()},
      {"eyetracking", std::vector<int>()},
      {"rgb", std::vector<int>()},
  };
  for (auto& cameraCheckSetting : settings.cameraCheckSettings) {
    const std::string& cameraTypeName = cameraCheckSetting.second.cameraTypeName;
    // frame count check parsin
    app.add_option(
        fmt::format("--{}-max-frame-count", cameraTypeName),
        cameraCheckSetting.second.maxFrameCounts,
        fmt::format("Max number of total frames for {}", cameraTypeName));
    app.add_option(
        fmt::format("--{}-min-frame-count", cameraTypeName),
        cameraCheckSetting.second.minFrameCounts,
        fmt::format("Min number of total frames for {}", cameraTypeName));

    // Roi parsing (Aria specific)
    app.add_option(
           fmt::format("--{}-roi-to-check", cameraTypeName),
           cameraRoiRanges.at(cameraTypeName),
           fmt::format(
               "(Aria-specific) Camera ROI to check for {}, format: [xMin, yMin, xMax, yMax]",
               cameraTypeName))
        ->expected(4);
    app.add_option(
        fmt::format("--{}-max-roi-bad-frame", cameraTypeName),
        cameraCheckSetting.second.maxRoiBadFrames,
        fmt::format("(Aria-specific) Max number of allowed bad frames for {}", cameraTypeName));
    app.add_option(
        fmt::format("--{}-roi-lower-thresh", cameraTypeName),
        cameraCheckSetting.second.roiLowerThresh,
        fmt::format(
            "(Aria-specific) Lower threshold for mean pixels within roi for {}", cameraTypeName));
    app.add_option(
        fmt::format("--{}-roi-upper-thresh", cameraTypeName),
        cameraCheckSetting.second.roiUpperThresh,
        fmt::format(
            "(Aria-specific) Upper threshold for mean pixels within roi for {}", cameraTypeName));
  } // end for cameraCheckSetting

  app.add_flag("--print-stats", printStats, "Print stats at the end");
  CLI::Option* const jsonOption =
      app.add_flag("--json-out{}", jsonOutFilename, "Export stats as JSON to console or file")
          ->expected(0, 1);
  CLI::Option* const droppedFrameOption = app.add_flag(
                                                 "--dropped-frames-out{}",
                                                 droppedOutFilename,
                                                 "Export detailed stats on dropped frames to file")
                                              ->expected(0, 1);

  // Mutually exclusive options
  auto* verboseOpt = app.add_flag("--verbose", verbose, "Print verbose logging");
  auto* debugOpt = app.add_flag("--debug", debug, "Print debug logging");
  debugOpt->excludes(verboseOpt);

  settings.isInteractive = ISATTY(FILENO(stdout)); // else it's a file or pipe

  try {
    app.parse(argc, argv);
  } catch (const CLI::CallForHelp&) {
    std::cout << app.help("", CLI::AppFormatMode::All);
    exit(-EINVAL);
  }

  if (verbose) {
    arvr::logging::setGlobalLogLevel(arvr::logging::Level::Info);
  } else if (debug) {
    arvr::logging::setGlobalLogLevel(arvr::logging::Level::Debug);
  }

  // Construct camera ROIs
  for (auto& cameraCheckSetting : settings.cameraCheckSettings) {
    const std::string& cameraTypeName = cameraCheckSetting.second.cameraTypeName;
    const std::vector<int>& roiRange = cameraRoiRanges.at(cameraTypeName);
    if (!roiRange.empty()) {
      cameraCheckSetting.second.roiToCheck = Eigen::AlignedBox2i(
          Eigen::Vector2i(roiRange[0], roiRange[1]), Eigen::Vector2i(roiRange[2], roiRange[3]));
    }
  } // end for camera roiSetting

  VrsHealthCheck healthCheck(settings);
  if (!healthCheck.setup(path)) {
    return EXIT_FAILURE;
  }
  if (!healthCheck.run()) {
    return EXIT_FAILURE;
  }
  if (printStats) {
    healthCheck.logStats();
  }
  if (*jsonOption) {
    healthCheck.logStatsJson(jsonOutFilename);
  }
  if (*droppedFrameOption) {
    healthCheck.logDroppedFrames(droppedOutFilename);
  }

  return healthCheck.getResult() ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace projectaria::tools::vrs_check
