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

#include <filesystem>

#include <gtest/gtest.h>

#include "AriaEverydayActivitiesDataPathsFormat.h"
#include "AriaEverydayActivitiesDataProvider.h"

using namespace projectaria::dataset::aea;

#define STRING(x) #x
#define XSTRING(x) std::string(STRING(x))

static const std::string aeaTestDataPath = XSTRING(TEST_FOLDER);

namespace fs = std::filesystem;

bool areSentencesEqual(const SentenceData& s1, const SentenceData& s2) {
  if (s1.startTimestampNs != s2.startTimestampNs) {
    std::cerr << "Start timestamp mismatch: " << s1.startTimestampNs << ", " << s2.startTimestampNs
              << std::endl;
    return false;
  }
  if (s1.endTimestampNs != s2.endTimestampNs) {
    std::cerr << "End timestamp mismatch: " << s1.endTimestampNs << ", " << s2.endTimestampNs
              << std::endl;
    return false;
  }
  if (s1.toString() != s2.toString()) {
    std::cerr << "Sentence mismatch: " << s1.toString() << ", " << s2.toString() << std::endl;
    return false;
  }
  return true;
}

bool areWordsEqual(const WordData& w1, const WordData& w2) {
  if (w1.startTimestampNs != w2.startTimestampNs) {
    std::cerr << "Start timestamp mismatch: " << w1.startTimestampNs << ", " << w2.startTimestampNs
              << std::endl;
    return false;
  }
  if (w1.endTimestampNs != w2.endTimestampNs) {
    std::cerr << "End timestamp mismatch: " << w1.endTimestampNs << ", " << w2.endTimestampNs
              << std::endl;
    return false;
  }
  if (w1.confidence != w2.confidence) {
    std::cerr << "Confidence timestamp mismatch: " << w1.confidence << ", " << w2.confidence
              << std::endl;
    return false;
  }
  if (w1.word != w2.word) {
    std::cerr << "Word mismatch: " << w1.word << ", " << w2.word << std::endl;
    return false;
  }
  return true;
}

TEST(AeaDataPathsProvider, PathsAPI) {
  // Construct a ADT data provider from the test data path
  const auto dataPathsProvider = AriaEverydayActivitiesDataPathsProvider(aeaTestDataPath);
  const auto dataPaths = dataPathsProvider.getDataPaths();

  // Check metadata
  EXPECT_EQ(dataPathsProvider.getLocationNumber(), 1);
  EXPECT_EQ(dataPathsProvider.getScriptNumber(), 1);
  EXPECT_EQ(dataPathsProvider.getSequenceNumber(), 1);
  EXPECT_EQ(dataPathsProvider.getRecordingNumber(), 1);
  EXPECT_TRUE(dataPathsProvider.getConcurrentRecordings().empty());
  EXPECT_EQ(dataPathsProvider.getDatasetName(), "AEA_2024");
  EXPECT_EQ(dataPathsProvider.getDatasetVersion(), "1.0");

  // check that loading an invalid path will fail
  EXPECT_THROW(AriaEverydayActivitiesDataPathsProvider("/tmp");, std::runtime_error);

  // check that all filepaths exists, except for MPS SLAM which could not be run on such a short
  // dataset
  EXPECT_TRUE(fs::exists(dataPaths.ariaVrs));
  EXPECT_TRUE(fs::exists(dataPaths.speech));
  EXPECT_TRUE(fs::exists(dataPaths.metadata));

  EXPECT_TRUE(fs::exists(dataPaths.mps.eyegaze.generalEyegaze));
  EXPECT_TRUE(fs::exists(dataPaths.mps.eyegaze.summary));

  EXPECT_TRUE(dataPaths.mps.slam.closedLoopTrajectory.empty());
  EXPECT_TRUE(dataPaths.mps.slam.openLoopTrajectory.empty());
  EXPECT_TRUE(dataPaths.mps.slam.semidensePoints.empty());
  EXPECT_TRUE(dataPaths.mps.slam.semidenseObservations.empty());
  EXPECT_TRUE(dataPaths.mps.slam.onlineCalibration.empty());
  EXPECT_TRUE(dataPaths.mps.slam.summary.empty());
}

TEST(AeaDataProvider, BasicAPI) {
  // Construct a ADT data provider from the test data path
  const auto dataPathsProvider = AriaEverydayActivitiesDataPathsProvider(aeaTestDataPath);
  const auto dataPaths = dataPathsProvider.getDataPaths();
  const auto dp = AriaEverydayActivitiesDataProvider(dataPaths);

  // check existence of data
  EXPECT_TRUE(dp.hasAriaData());
  EXPECT_TRUE(dp.hasSpeechData());

  // check vrs data provider basics
  EXPECT_TRUE(dp.vrs);
  EXPECT_FALSE(dp.vrs->getAllStreams().empty());
  EXPECT_TRUE(dp.vrs->getDeviceCalibration());
}

TEST(AeaDataProvider, Speech) {
  // Construct a ADT data provider from the test data path
  const auto dataPathsProvider = AriaEverydayActivitiesDataPathsProvider(aeaTestDataPath);
  const auto dataPaths = dataPathsProvider.getDataPaths();
  const auto dp = AriaEverydayActivitiesDataProvider(dataPaths);

  const std::vector<int64_t> startTsNs{
      101217000000,
      128657000000,
      129657000000,
      129817000000,
      129977000000,
      130217000000,
      130377000000,
      196337000000,
      197337000000,
      197497000000,
      197657000000,
      198217000000,
      198377000000};

  const std::vector<int64_t> endTsNs{
      102217000000,
      129657000000,
      129817000000,
      129977000000,
      130217000000,
      130377000000,
      131017000000,
      197337000000,
      197497000000,
      197657000000,
      198217000000,
      198377000000,
      198937000000};

  // check first and second sentences
  auto maybeSentenceDataFirst = dp.speech->getSentenceDataByTimestampNs(startTsNs.front());
  EXPECT_TRUE(maybeSentenceDataFirst.has_value());
  const auto& s1 = maybeSentenceDataFirst.value();
  EXPECT_EQ(s1.toString(), "and what is there like looking at?");
  EXPECT_EQ(s1.startTimestampNs, startTsNs[0]);
  EXPECT_EQ(s1.endTimestampNs, endTsNs[6]);

  auto maybeSentenceDataLast = dp.speech->getSentenceDataByTimestampNs(startTsNs[7]);
  EXPECT_TRUE(maybeSentenceDataLast.has_value());
  const auto& s2 = maybeSentenceDataLast.value();
  EXPECT_EQ(s2.toString(), "I'm in the masters, right now.");
  EXPECT_EQ(s2.startTimestampNs, startTsNs[7]);
  EXPECT_EQ(s2.endTimestampNs, endTsNs.back());

  // Check before and after query options
  EXPECT_FALSE(
      dp.speech->getSentenceDataByTimestampNs(startTsNs.front() - 1, TimeQueryOptions::Before)
          .has_value());
  EXPECT_FALSE(dp.speech->getSentenceDataByTimestampNs(endTsNs.back() + 1, TimeQueryOptions::After)
                   .has_value());
  auto maybeS1 =
      dp.speech->getSentenceDataByTimestampNs(startTsNs.front() - 1, TimeQueryOptions::After)
          .value();
  EXPECT_TRUE(areSentencesEqual(s1, maybeS1));
  maybeS1 =
      dp.speech->getSentenceDataByTimestampNs(startTsNs.front() - 1, TimeQueryOptions::Closest)
          .value();
  EXPECT_TRUE(areSentencesEqual(s1, maybeS1));

  auto maybeS2 =
      dp.speech->getSentenceDataByTimestampNs(startTsNs[7] + 1, TimeQueryOptions::After).value();
  EXPECT_TRUE(areSentencesEqual(s2, maybeS2));
  maybeS2 =
      dp.speech->getSentenceDataByTimestampNs(startTsNs[7] + 1, TimeQueryOptions::Closest).value();
  EXPECT_TRUE(areSentencesEqual(s2, maybeS2));
  maybeS2 = dp.speech->getSentenceDataByTimestampNs(startTsNs.back() + 1, TimeQueryOptions::Closest)
                .value();
  EXPECT_TRUE(areSentencesEqual(s2, maybeS2));
  maybeS2 = dp.speech->getSentenceDataByTimestampNs(startTsNs.back() + 1, TimeQueryOptions::Before)
                .value();
  EXPECT_TRUE(areSentencesEqual(s2, maybeS2));

  // check word query
  auto maybeWordData = dp.speech->getWordDataByTimestampNs(startTsNs[5]);
  EXPECT_TRUE(maybeWordData.has_value());
  const auto& w5 = maybeWordData.value();
  EXPECT_EQ(w5.word, "looking");
  EXPECT_EQ(w5.startTimestampNs, startTsNs[5]);
  EXPECT_EQ(w5.endTimestampNs, endTsNs[5]);

  auto w4 = dp.speech->getWordDataByTimestampNs(startTsNs[5] - 1, TimeQueryOptions::Before);
  EXPECT_EQ(w4.value().word, "like");
  auto w6 = dp.speech->getWordDataByTimestampNs(endTsNs[5] + 1, TimeQueryOptions::After);
  EXPECT_EQ(w6.value().word, "at?");
}

TEST(AeaDataProvider, Mps) {
  // Construct a ADT data provider from the test data path
  const auto dataPathsProvider = AriaEverydayActivitiesDataPathsProvider(aeaTestDataPath);
  const auto dataPaths = dataPathsProvider.getDataPaths();
  const auto dp = AriaEverydayActivitiesDataProvider(dataPaths);

  EXPECT_TRUE(dp.hasMpsData());
  EXPECT_TRUE(dp.mps);
  EXPECT_TRUE(dp.mps->hasGeneralEyeGaze());
  EXPECT_FALSE(dp.mps->hasPersonalizedEyeGaze());
  EXPECT_FALSE(dp.mps->hasOpenLoopPoses());
  EXPECT_FALSE(dp.mps->hasClosedLoopPoses());
  EXPECT_FALSE(dp.mps->hasOnlineCalibrations());
  EXPECT_FALSE(dp.mps->hasSemidensePointCloud());
  EXPECT_FALSE(dp.mps->hasSemidenseObservations());

  const int64_t firstTsNs = 86435306000;
  auto maybeEyeGaze1 = dp.mps->getGeneralEyeGaze(firstTsNs);
  EXPECT_TRUE(maybeEyeGaze1.has_value());
  auto eG1 = maybeEyeGaze1.value();
  EXPECT_NEAR(eG1.yaw, -0.36566, 1e-5);
  EXPECT_NEAR(eG1.pitch, -0.121962, 1e-5);
  EXPECT_EQ(eG1.session_uid, "e7e91580-ee57-41c0-98c6-5f1272a7176e");
}
