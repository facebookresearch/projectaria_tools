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

#include "AriaEverydayActivitiesDataTypes.h"

namespace projectaria::dataset::aea {

/**
 * @brief Class for reading and querying speech data generated in the AEA dataset
 */
class SpeechDataProvider {
 public:
  explicit SpeechDataProvider(const std::string& speechFilepath);

  // ---- Functions to access the data ----

  /**
   * @brief Get sentence data given a query timestamp. A sentence is a series of words, where each
   * sentence has a start and end timestamp, and each word has a start and end timestamp.
   * Note: TimeQueryOptions is ignored if there is a sentence that contains the query device
   * timestamp
   */
  std::optional<SentenceData> getSentenceDataByTimestampNs(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Get word data given a query timestamp. A word has a start and end timestamp and
   * confidence level. Note: TimeQueryOptions is ignored if there is a word that contains
   * the query device timestamp
   */
  std::optional<WordData> getWordDataByTimestampNs(
      int64_t deviceTimeStampNs,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) const;

  /**
   * @brief Check whether the speech data is empty
   */
  bool empty() const;

 private:
  void loadSpeechData();

  std::string speechFilepath_;
  SentenceDataSeries speechData_;
};

} // namespace projectaria::dataset::aea
