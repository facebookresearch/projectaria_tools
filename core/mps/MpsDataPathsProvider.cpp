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

#include "MpsDataPathsProvider.h"

#include <filesystem>
#include <fstream>
#include <sstream>

#define DEFAULT_LOG_CHANNEL "MpsDataPathsProvider"
#include <logging/Log.h>

#include "MpsFileNames.h"

namespace fs = std::filesystem;

namespace {

void loadFilePathIfExists(
    const std::string& filepathIn,
    const std::string& filename,
    std::string& filepathOut) {
  const std::string filepath = (fs::path(filepathIn) / filename).string();
  if (!fs::exists(filepath)) {
    filepathOut.clear();
  } else {
    filepathOut = filepath;
  }
}

} // namespace

namespace projectaria::tools::mps {

MpsDataPathsProvider::MpsDataPathsProvider(const std::string& mpsRootPath)
    : mpsRootPath_(mpsRootPath) {
  loadDataPaths();
}

MpsDataPaths MpsDataPathsProvider::getDataPaths() const {
  return dataPaths_;
}

void MpsDataPathsProvider::loadDataPaths() {
  dataPaths_.root = mpsRootPath_;
  if (!fs::exists(dataPaths_.root)) {
    XR_LOGW("MPS root not found at {}, not loading MPS paths", dataPaths_.root);
    return;
  }

  std::string mpsSlamPath = (fs::path(mpsRootPath_) / fs::path(kMpsSlamFolder)).string();
  if (!fs::exists(mpsSlamPath)) {
    mpsSlamPath = (fs::path(mpsRootPath_) / fs::path(kMpsSlamFolderDeprecated)).string();
  }

  if (!fs::exists(mpsSlamPath)) {
    XR_LOGW(
        "MPS SLAM folder does not exist in MPS root folder with the name {} or {}, not loading SLAM data paths",
        kMpsSlamFolder,
        kMpsSlamFolderDeprecated);
  } else {
    loadFilePathIfExists(
        mpsSlamPath, kMpsSlamClosedLoopTrajectoryFile, dataPaths_.slam.closedLoopTrajectory);
    loadFilePathIfExists(
        mpsSlamPath, kMpsSlamOpenLoopTrajectoryFile, dataPaths_.slam.openLoopTrajectory);
    loadFilePathIfExists(
        mpsSlamPath, kMpsSlamSemidenseObservationsFile, dataPaths_.slam.semidenseObservations);
    loadFilePathIfExists(mpsSlamPath, kMpsSlamSemidensePointsFile, dataPaths_.slam.semidensePoints);
    if (dataPaths_.slam.semidensePoints.empty()) {
      loadFilePathIfExists(
          mpsSlamPath, kMpsSlamSemidensePointsFileDeprecated, dataPaths_.slam.semidensePoints);
    }
    loadFilePathIfExists(
        mpsSlamPath, MpsSlamOnlineCalibrationFile, dataPaths_.slam.onlineCalibration);
    loadFilePathIfExists(mpsSlamPath, kMpsSlamSummaryFile, dataPaths_.slam.summary);
  }

  std::string mpsEyegazePath = (fs::path(mpsRootPath_) / fs::path(kMpsEyegazeFolder)).string();
  loadFilePathIfExists(mpsEyegazePath, kMpsGeneralEyegazeFile, dataPaths_.eyegaze.generalEyegaze);
  if (dataPaths_.eyegaze.generalEyegaze.empty()) {
    loadFilePathIfExists(
        mpsEyegazePath, kMpsGeneralEyegazeFileDeprecated, dataPaths_.eyegaze.generalEyegaze);
  }
  loadFilePathIfExists(
      mpsEyegazePath, kMpsPersonalEyegazeFile, dataPaths_.eyegaze.personalizedEyegaze);
  if (dataPaths_.eyegaze.personalizedEyegaze.empty()) {
    loadFilePathIfExists(
        mpsEyegazePath, kMpsPersonalEyegazeFileDeprecated, dataPaths_.eyegaze.personalizedEyegaze);
  }

  loadFilePathIfExists(mpsEyegazePath, kMpsEyegazeSummaryFile, dataPaths_.eyegaze.summary);
}

} // namespace projectaria::tools::mps
