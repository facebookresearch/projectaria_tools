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

#include <iostream>
#include <map>
#include <numeric>
#include <string>

#include <data_provider/TimeTypes.h>

namespace projectaria::dataset::aea {

/**
 * @brief word data extracted from speech
 */
struct WordData {
  int64_t startTimestampNs;
  int64_t endTimestampNs;
  std::string word;
  double confidence;
};

/**
 * @brief ordered map from start timestamp -> WordData
 */
using WordDataSeries = std::map<int64_t, WordData>;

/**
 * @brief sentence data which contains a series of words
 */
struct SentenceData {
  int64_t startTimestampNs;
  int64_t endTimestampNs;
  WordDataSeries words;

  [[nodiscard]] std::string toString() const {
    if (words.empty()) {
      return {};
    }
    return std::accumulate(
        std::next(words.begin()),
        words.end(),
        words.begin()->second.word,
        [](const std::string& a, const auto& b) { return a + " " + b.second.word; });
  }
};

/**
 * @brief ordered map from start timestamp -> SentenceData
 */
using SentenceDataSeries = std::map<int64_t, SentenceData>;

/**
 * @brief TimeQueryOptions are re-used from data_provider lib
 */
using TimeQueryOptions = projectaria::tools::data_provider::TimeQueryOptions;

} // namespace projectaria::dataset::aea
