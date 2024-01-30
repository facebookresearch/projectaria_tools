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

#include "AriaEverydayActivitiesDataPathsProvider.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include <nlohmann/json.hpp>

#define DEFAULT_LOG_CHANNEL "AriaEverydayActivitiesDataPathsProvider"
#include <logging/Log.h>

#include "AriaEverydayActivitiesFileKeys.h"
#include "AriaEverydayActivitiesFileNames.h"

namespace fs = std::filesystem;

namespace projectaria::dataset::aea {

namespace {

// TODO: move these functions to a utils class in core
void validateKeysOrThrow(
    const nlohmann::json& json,
    const std::string& filename,
    const std::vector<std::string>& keys) {
  std::vector<std::string> missingKeys;
  for (const auto& key : keys) {
    if (!json.contains(key)) {
      missingKeys.push_back(key);
    }
  }

  if (!missingKeys.empty()) {
    std::string missingKeysStr;
    for (const auto& key : missingKeys) {
      missingKeysStr += key + " ";
    }
    std::string errMsg =
        fmt::format("Json {} is missing the following required keys: {}", filename, missingKeysStr);
    XR_LOGE("{}", errMsg);
    throw std::runtime_error{errMsg};
  }
}

void loadFilePathIfExists(
    const std::string& filepathIn,
    const std::string& filename,
    std::string& filepathOut) {
  const std::string filepath = (fs::path(filepathIn) / filename).string();
  if (!fs::exists(filepath)) {
    XR_LOGW("{} file not found at {}", filename, filepath);
    filepathOut.clear();
  } else {
    filepathOut = filepath;
  }
}

} // namespace

AriaEverydayActivitiesDataPathsProvider::AriaEverydayActivitiesDataPathsProvider(
    const std::string& sequencePath)
    : sequencePath_(sequencePath) {
  loadSequenceMetadata();
  validateDatasetVersion();
  loadDataPaths();
}

void AriaEverydayActivitiesDataPathsProvider::loadSequenceMetadata() {
  if (!fs::exists(sequencePath_)) {
    XR_LOGE("sequence path does not exist: {}", sequencePath_);
    throw std::runtime_error{"invalid sequence path was provided"};
    return;
  }

  fs::path fileMetadata = fs::path(sequencePath_) / fs::path(kMetadataFile);
  std::ifstream fileStream;
  if (fs::exists(fileMetadata)) {
    fileStream.open(fileMetadata);
  } else {
    XR_LOGW("no metadata file found at {}", fileMetadata.string());
  }

  if (!fileStream.is_open()) {
    XR_LOGE("Could not open {}, is this a valid sequence path?", kMetadataFile);
    throw std::runtime_error{"Could not open metadata file"};
  }

  nlohmann::json J;
  fileStream >> J;

  validateKeysOrThrow(
      J,
      kMetadataFile,
      {kLocationKey,
       kScriptKey,
       kSequenceKey,
       kRecordingKey,
       kConcurrentRecordingsKey,
       kDatasetVersionKey,
       kDatasetNameKey});

  location_ = J[kLocationKey];
  script_ = J[kScriptKey];
  sequence_ = J[kSequenceKey];
  recording_ = J[kRecordingKey];
  datasetVersion_ = J[kDatasetVersionKey];
  datasetName_ = J[kDatasetNameKey];
  concurrentRecordings_ = J[kConcurrentRecordingsKey];
  fileStream.close();
}

void AriaEverydayActivitiesDataPathsProvider::loadDataPaths() {
  loadFilePathIfExists(sequencePath_, kAriaVrsFile, dataPaths_.ariaVrs);
  loadFilePathIfExists(sequencePath_, kSpeechFile, dataPaths_.speech);
  loadFilePathIfExists(sequencePath_, kMetadataFile, dataPaths_.metadata);

  fs::path mpsPath = fs::path(sequencePath_) / fs::path(kMpsFolderPath);
  tools::mps::MpsDataPathsProvider mpsDataPathsProvider(mpsPath.string());
  dataPaths_.mps = mpsDataPathsProvider.getDataPaths();
}

void AriaEverydayActivitiesDataPathsProvider::validateDatasetVersion() const {
  if (kLatestDatasetVersions.find(datasetName_) == kLatestDatasetVersions.end()) {
    XR_LOGE("Invalid dataset name: {}", datasetName_);
    throw std::runtime_error{"invalid dataset name"};
  }

  std::string latestVersion = kLatestDatasetVersions.at(datasetName_);
  if (datasetVersion_.compare(latestVersion) == 0) {
    return;
  }

  // check versions
  if (datasetVersion_.compare(latestVersion) < 0) {
    XR_LOGW(
        "dataset version read ({}) is not up to date with latest ({}), we recommend you redownload your AEA dataset."
        " For a full version update history, please see the AEA wiki",
        datasetVersion_,
        latestVersion);
    return;
  }
  if (datasetVersion_.compare(latestVersion) > 0) {
    XR_LOGE(
        "data loader version ({}) is behind dataset version read ({}), please update projectaria_tools from github.",
        datasetVersion_,
        latestVersion);
    throw std::runtime_error{
        "data loader version is behind dataset version, projectaria_tools needs to be updated"};
  }
}

AriaEverydayActivitiesDataPaths AriaEverydayActivitiesDataPathsProvider::getDataPaths() const {
  return dataPaths_;
}

int AriaEverydayActivitiesDataPathsProvider::getLocationNumber() const {
  return location_;
}

int AriaEverydayActivitiesDataPathsProvider::getScriptNumber() const {
  return script_;
}

int AriaEverydayActivitiesDataPathsProvider::getSequenceNumber() const {
  return sequence_;
}

int AriaEverydayActivitiesDataPathsProvider::getRecordingNumber() const {
  return recording_;
}

std::vector<std::string> AriaEverydayActivitiesDataPathsProvider::getConcurrentRecordings() const {
  return concurrentRecordings_;
}

std::string AriaEverydayActivitiesDataPathsProvider::getDatasetName() const {
  return datasetName_;
}

std::string AriaEverydayActivitiesDataPathsProvider::getDatasetVersion() const {
  return datasetVersion_;
}

} // namespace projectaria::dataset::aea
