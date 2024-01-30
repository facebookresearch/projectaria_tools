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
#include <unordered_map>

namespace projectaria::dataset::aea {

// sequence metadata keys
inline const std::string kLocationKey = "location";
inline const std::string kScriptKey = "script";
inline const std::string kSequenceKey = "sequence";
inline const std::string kRecordingKey = "recording";
inline const std::string kConcurrentRecordingsKey = "concurrent_recordings";

// dataset version
inline const std::string kDatasetVersionKey = "dataset_version";
inline const std::string kDatasetNameDefault = "AEA_2024";
inline const std::string kDatasetVersionDefault = "1.0";
inline const std::string kDatasetNameKey = "dataset_name";
inline const std::unordered_map<std::string, std::string> kLatestDatasetVersions{
    {"AEA_2024", "1.0"}};

} // namespace projectaria::dataset::aea
