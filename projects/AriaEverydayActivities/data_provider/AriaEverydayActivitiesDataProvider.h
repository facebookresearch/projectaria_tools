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

#include <optional>

#include "AriaEverydayActivitiesDataPathsProvider.h"
#include "AriaEverydayActivitiesDataTypes.h"
#include "SpeechDataProvider.h"
#include "data_provider/VrsDataProvider.h"
#include "mps/MpsDataProvider.h"

namespace projectaria::dataset::aea {

/**
 * @brief This is the core data loader that should provide all the data access you will need for
 * an AEA sequence.
 */
class AriaEverydayActivitiesDataProvider {
 public:
  /**
   * @brief Constructor that takes an AEA data paths object, which can be created using
   * AriaEverydayActivitiesDataPathsProvider
   */
  explicit AriaEverydayActivitiesDataProvider(const AriaEverydayActivitiesDataPaths& dataPaths);

  /**
   * @brief Constructor that takes an AEA sequence path. This calls the
   * AriaEverydayActivitiesDataPathsProvider to create the paths
   */
  explicit AriaEverydayActivitiesDataProvider(const std::string& sequencePath);

  // ---- Functions to check availability of data ----
  [[nodiscard]] bool hasAriaData() const;

  [[nodiscard]] bool hasSpeechData() const;

  [[nodiscard]] bool hasMpsData() const;

  // public access to the vrs data provider for all access to raw sensor data
  std::shared_ptr<projectaria::tools::data_provider::VrsDataProvider> vrs;

  // public access to the speech data provider
  std::shared_ptr<SpeechDataProvider> speech;

  // public access to the speech data provider
  std::shared_ptr<tools::mps::MpsDataProvider> mps;

 private:
  void validateDatasetVersion() const;
  void loadVrs();
  void loadSpeech();
  void loadMps();

  AriaEverydayActivitiesDataPaths dataPaths_;
};

} // namespace projectaria::dataset::aea
