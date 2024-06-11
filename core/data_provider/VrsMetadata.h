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

#include <map>
#include <string>

namespace projectaria::tools::data_provider {

enum class MetadataTimeSyncMode : int {
  NotEnabled = 0,
  Timecode = 1,
  Ntp = 2,
  TicSyncClient = 3,
  TicSyncServer = 4,
};

/**
 * @brief A convenience struct for VRS files containing recordings.
 * @details This struct is not exactly the VRS file metadata tag, but includes
 * essential information, such as device serial, from other tags in the VRS
 * file. There may be some keys in the file metadata that are not copied to this
 * convenience struct.
 */
struct VrsMetadata {
  std::string deviceSerial;
  std::string sharedSessionId;
  std::string recordingProfile;
  MetadataTimeSyncMode timeSyncMode = MetadataTimeSyncMode::NotEnabled;
  std::string deviceId;
  std::string filename;
  uint64_t startTimeEpochSec = 0;
  uint64_t endTimeEpochSec = 0;
  uint64_t durationSec = 0;
};

} // namespace projectaria::tools::data_provider
