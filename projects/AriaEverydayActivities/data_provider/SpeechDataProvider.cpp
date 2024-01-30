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

#include "SpeechDataProvider.h"

#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <vector>

#include <data_provider/QueryMapByTimestamp.h>

#define DEFAULT_LOG_CHANNEL "SpeechDataProvider"
#include <logging/Log.h>

namespace fs = std::filesystem;

namespace projectaria::dataset::aea {

using namespace projectaria::tools::data_provider;

const std::set<char> kSentenceEndChars{'.', '?', '!'};
const char kCsvDelimiter = ',';

namespace {

// Each line has timestamp_start, timestamp_end, text, confidence.
// Text might contain commas in the string which will be separated with the std::getline function.
// Since we know that there are only 4 column, then we take first two as timestamps, the last as
// confidence, and the rest get combined into the word
WordData getWordDataFromLine(std::stringstream& ss) {
  WordData wordData;
  std::vector<std::string> tokens;
  std::string token;
  while (std::getline(ss, token, kCsvDelimiter)) {
    tokens.emplace_back(token);
  }

  int numTokens = tokens.size();
  wordData.startTimestampNs = std::stoll(tokens.at(0));
  wordData.endTimestampNs = std::stoll(tokens.at(1));
  wordData.confidence = std::stod(tokens.at(numTokens - 1));

  // if there are 4 tokens, then we know the word is all contained in one
  if (numTokens == 4) {
    wordData.word = tokens.at(2);
    return wordData;
  }

  // otherwise, we need to find the quotations and take everything in between them
  std::string ss_str = ss.str();
  int first = ss_str.find_first_of('\"');
  int last = ss_str.find_last_of('\"');
  for (int i = first + 1; i < last; i++) {
    wordData.word += ss_str[i];
  }

  return wordData;
}

} // namespace

SpeechDataProvider::SpeechDataProvider(const std::string& speechFilepath)
    : speechFilepath_(speechFilepath) {
  loadSpeechData();
}

std::optional<SentenceData> SpeechDataProvider::getSentenceDataByTimestampNs(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) const {
  if (empty()) {
    XR_LOGW("No speech data, cannot get sentence data\n");
    return {};
  }

  // check bounds
  if (deviceTimeStampNs < speechData_.begin()->second.startTimestampNs) {
    if (timeQueryOptions == TimeQueryOptions::Before) {
      return {};
    }
    return speechData_.begin()->second;
  }

  if (deviceTimeStampNs > speechData_.rbegin()->second.endTimestampNs) {
    if (timeQueryOptions == TimeQueryOptions::After) {
      return {};
    }
    return speechData_.rbegin()->second;
  }

  // check for a sentence that contains the query timestamp
  auto iterBefore =
      queryMapByTimestamp<SentenceData>(speechData_, deviceTimeStampNs, TimeQueryOptions::Before);
  if (deviceTimeStampNs <= iterBefore->second.endTimestampNs) {
    return iterBefore->second;
  }

  // if we get to here, then we know the query timestamp is in between sentences. Let's check the
  // one before and the one after
  auto iterAfter = std::next(iterBefore);
  switch (timeQueryOptions) {
    case TimeQueryOptions::Closest:
      if (deviceTimeStampNs - iterBefore->second.endTimestampNs <
          iterAfter->second.startTimestampNs - deviceTimeStampNs) {
        return iterBefore->second;
      } else {
        return iterAfter->second;
      }
      break;
    case TimeQueryOptions::Before:
      return iterBefore->second;
      break;
    case TimeQueryOptions::After:
      return iterAfter->second;
      break;
    default:
      XR_LOGE("invalid time query option, option not yet implemented");
      throw std::runtime_error{"invalid query option"};
  }
}

std::optional<WordData> SpeechDataProvider::getWordDataByTimestampNs(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) const {
  if (empty()) {
    XR_LOGW("No speech data, cannot get word data");
    return {};
  }

  auto maybeSentenceData = getSentenceDataByTimestampNs(deviceTimeStampNs, timeQueryOptions);
  if (!maybeSentenceData.has_value()) {
    return {};
  }
  const auto& sentenceData = maybeSentenceData.value();

  // check bounds
  if (deviceTimeStampNs < sentenceData.startTimestampNs) {
    if (timeQueryOptions == TimeQueryOptions::Before) {
      return {};
    }
    return sentenceData.words.begin()->second;
  }
  if (deviceTimeStampNs > sentenceData.endTimestampNs) {
    if (timeQueryOptions == TimeQueryOptions::After) {
      return {};
    }
    return sentenceData.words.rbegin()->second;
  }

  // check for a word that contains the query timestamp
  auto iterBefore = queryMapByTimestamp<WordData>(
      sentenceData.words, deviceTimeStampNs, TimeQueryOptions::Before);
  if (deviceTimeStampNs <= iterBefore->second.endTimestampNs) {
    return iterBefore->second;
  }

  // if we get to here, then we know the query timestamp is in between words. Let's check the one
  // before and after
  auto iterAfter = std::next(iterBefore);
  switch (timeQueryOptions) {
    case TimeQueryOptions::Closest:
      if (deviceTimeStampNs - iterBefore->second.endTimestampNs <
          iterAfter->second.startTimestampNs - deviceTimeStampNs) {
        return iterBefore->second;
      } else {
        return iterAfter->second;
      }
      break;
    case TimeQueryOptions::Before:
      return iterBefore->second;
      break;
    case TimeQueryOptions::After:
      return iterAfter->second;
      break;
    default:
      XR_LOGE("invalid time query option, option not yet implemented");
      throw std::runtime_error{"invalid query option"};
      break;
  }
}

void SpeechDataProvider::loadSpeechData() {
  if (speechFilepath_.empty()) {
    XR_LOGI("skip loading speech data because the data path is empty");
    return;
  }

  fs::path fileSpeech(speechFilepath_);
  std::ifstream fileStream(fileSpeech);
  if (!fileStream.is_open()) {
    throw std::runtime_error(fmt::format("Could not open file {} \n", speechFilepath_));
  }
  // skip header
  std::string line;
  std::getline(fileStream, line);
  SentenceData currentSentence;
  while (std::getline(fileStream, line)) {
    std::stringstream ss;
    ss.str(line);
    WordData currentWord = getWordDataFromLine(ss);

    // add to sentence
    if (currentSentence.words.empty()) {
      currentSentence.startTimestampNs = currentWord.startTimestampNs;
    }
    currentSentence.endTimestampNs = currentWord.endTimestampNs;
    currentSentence.words.emplace(currentWord.startTimestampNs, currentWord);

    // check if this ends a sentence
    if (kSentenceEndChars.find(currentWord.word[currentWord.word.size() - 1]) !=
        kSentenceEndChars.end()) {
      speechData_.emplace(currentSentence.startTimestampNs, currentSentence);
      currentSentence = {};
    }
  }

  // add any remainder
  if (!currentSentence.words.empty()) {
    speechData_.emplace(currentSentence.startTimestampNs, currentSentence);
  }

  fileStream.close();
}

bool SpeechDataProvider::empty() const {
  return speechData_.empty();
}

} // namespace projectaria::dataset::aea
