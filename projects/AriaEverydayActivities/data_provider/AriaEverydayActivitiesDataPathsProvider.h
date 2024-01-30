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

#include <string>
#include <vector>

#include <mps/MpsDataPathsProvider.h>

namespace projectaria::dataset::aea {

/**
 * @brief A struct that includes the file paths of all AEA data files for one sequence
 */
struct AriaEverydayActivitiesDataPaths {
  std::string ariaVrs; /**< Aria vrs */
  std::string speech; /**< speech csv */
  std::string metadata; /**< data collection metadata file */
  tools::mps::MpsDataPaths mps; /**< MPS file paths */
};

/**
 * @brief This class is to load all data file paths from AEA data structure given a sequence path.
 * Each AEA collection sequence can only contain one Aria device and its associated data:<br>
 *
 * ├── sequencePath
 * │   ├── metadata.json
 * │   ├── recording.vrs
 * │   ├── speech.csv
 * │   ├── mps
 * │   │   ├── {SEE MpsDataPathsProvider.h}
 *
 * This class allows you use dataset root to query all data associated with a single device
 * recording.
 */
class AriaEverydayActivitiesDataPathsProvider {
 public:
  explicit AriaEverydayActivitiesDataPathsProvider(const std::string& sequencePath);

  /**
   * @brief Get the resulting data paths
   * @return AEA data paths object that can be fed to the AEA data provider
   */
  AriaEverydayActivitiesDataPaths getDataPaths() const;

  /**
   * @brief Get the location number. This number is found in the metadata json file, and is also
   * embeded in the sequence name
   */
  int getLocationNumber() const;

  /**
   * @brief Get the script number. This number is found in the metadata json file, and is also
   * embeded in the sequence name
   */
  int getScriptNumber() const;

  /**
   * @brief Get the sequence number. This number is found in the metadata json file, and is also
   * embeded in the sequence name
   */
  int getSequenceNumber() const;

  /**
   * @brief Get the recording number. This number is found in the metadata json file, and is also
   * embeded in the sequence name
   */
  int getRecordingNumber() const;

  /**
   * @brief Get the recordings that were collected at the same time and location as the current
   * recording. This data can be found in the metadata json file
   * @return vector of sequence names
   */
  std::vector<std::string> getConcurrentRecordings() const;

  /**
   * @brief Get the name of the current dataset
   */
  std::string getDatasetName() const;

  /**
   * @brief Get the version of the current dataset
   */
  std::string getDatasetVersion() const;

 private:
  void validateDatasetVersion() const;
  void loadSequenceMetadata();
  void loadDataPaths();
  std::string sequencePath_;

  int location_{-1};
  int script_{-1};
  int sequence_{-1};
  int recording_{-1};
  std::string datasetName_;
  std::string datasetVersion_;
  std::vector<std::string> concurrentRecordings_;
  AriaEverydayActivitiesDataPaths dataPaths_;
};

} // namespace projectaria::dataset::aea
