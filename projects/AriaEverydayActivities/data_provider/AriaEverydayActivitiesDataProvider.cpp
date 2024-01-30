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

#include "AriaEverydayActivitiesDataProvider.h"

#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#define DEFAULT_LOG_CHANNEL "AriaEverydayActivitiesDataProvider"
#include <logging/Log.h>

#include "AriaEverydayActivitiesFileKeys.h"

using namespace projectaria::tools::data_provider;
using namespace projectaria::tools::calibration;
namespace fs = std::filesystem;

namespace projectaria::dataset::aea {

namespace {
void validateKeysOrThrow(
    const nlohmann::json& json,
    const std::string& filename,
    const std::vector<std::string>& keys) {
  for (const auto& key : keys) {
    if (!json.contains(key)) {
      const std::string errMsg =
          fmt::format("key: '{}' not available in {} json file", key, filename);
      XR_LOGE("{}", errMsg);
      throw std::runtime_error{errMsg};
    }
  }
}
} // namespace

AriaEverydayActivitiesDataProvider::AriaEverydayActivitiesDataProvider(
    const AriaEverydayActivitiesDataPaths& dataPaths)
    : dataPaths_(dataPaths) {
  validateDatasetVersion();
  loadVrs();
  loadSpeech();
  loadMps();
}

AriaEverydayActivitiesDataProvider::AriaEverydayActivitiesDataProvider(
    const std::string& sequencePath) {
  AriaEverydayActivitiesDataPathsProvider pathsProvider(sequencePath);
  dataPaths_ = pathsProvider.getDataPaths();
  validateDatasetVersion();
  loadVrs();
  loadSpeech();
  loadMps();
}

void AriaEverydayActivitiesDataProvider::validateDatasetVersion() const {
  // Get dataset name and version from metadata file
  if (dataPaths_.metadata.empty()) {
    XR_LOGE("empty metadata file path, unable to validate dataset version!");
    return;
  }

  if (!fs::exists(dataPaths_.metadata)) {
    XR_LOGE("metadata file path does not exist at: {}", dataPaths_.metadata);
    XR_LOGE("cannot load metadata file, unable to validate dataset version!");
    return;
  }

  std::ifstream fileStream;
  fileStream.open(dataPaths_.metadata);
  if (!fileStream.is_open()) {
    XR_LOGE("could not open {}, is this a valid sequence path?", dataPaths_.metadata);
    throw std::runtime_error{"could not open metadata file"};
  }

  nlohmann::json jsonObject;
  fileStream >> jsonObject;

  validateKeysOrThrow(
      jsonObject,
      dataPaths_.metadata,
      {kDatasetNameKey,
       kDatasetVersionKey,
       kSequenceKey,
       kRecordingKey,
       kConcurrentRecordingsKey,
       kDatasetVersionKey,
       kDatasetNameKey});

  std::string datasetVersion = jsonObject[kDatasetVersionKey];
  std::string datasetName = jsonObject[kDatasetNameKey];

  if (kLatestDatasetVersions.find(datasetName) == kLatestDatasetVersions.end()) {
    XR_LOGE("Invalid dataset name: {}", datasetName);
    throw std::runtime_error{"invalid dataset name"};
  }

  std::string latestVersion = kLatestDatasetVersions.at(datasetName);
  if (datasetVersion.compare(latestVersion) == 0) {
    return;
  }

  // check versions
  if (datasetVersion.compare(latestVersion) < 0) {
    XR_LOGW(
        "dataset version read ({}) is not up to date with latest ({}), we recommend you redownload your ADT dataset."
        " For a full version update history, please see the AEA wiki",
        datasetVersion,
        latestVersion);
    return;
  }
  if (datasetVersion.compare(latestVersion) > 0) {
    XR_LOGE(
        "data loader version ({}) is behind dataset version read ({}), please update projectaria_tools from github.",
        datasetVersion,
        latestVersion);
    throw std::runtime_error{
        "data loader version is behind dataset version, projectaria_tools needs to be updated"};
  }
}

void AriaEverydayActivitiesDataProvider::loadVrs() {
  if (!dataPaths_.ariaVrs.empty()) {
    vrs = createVrsDataProvider(dataPaths_.ariaVrs);
  } else {
    XR_LOGI("skip loading VRS data because the data path is empty");
  }
}

void AriaEverydayActivitiesDataProvider::loadSpeech() {
  if (dataPaths_.speech.empty()) {
    XR_LOGI("skip loading speech data because the data path is empty");
    return;
  }
  speech = std::make_shared<SpeechDataProvider>(dataPaths_.speech);
}

void AriaEverydayActivitiesDataProvider::loadMps() {
  // load MPS only if root MPS folder exists
  if (!fs::exists(dataPaths_.mps.root)) {
    XR_LOGI("skip loading MPS data because the root path does not exist");
    return;
  }
  mps = std::make_shared<projectaria::tools::mps::MpsDataProvider>(dataPaths_.mps);
}

bool AriaEverydayActivitiesDataProvider::hasAriaData() const {
  return vrs != nullptr;
}

bool AriaEverydayActivitiesDataProvider::hasSpeechData() const {
  return speech ? !speech->empty() : false;
}

bool AriaEverydayActivitiesDataProvider::hasMpsData() const {
  return mps != nullptr;
}

} // namespace projectaria::dataset::aea
