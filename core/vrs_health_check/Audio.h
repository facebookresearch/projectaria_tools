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

#include "Periodic.h"

#include <data_provider/players/AudioPlayer.h>

namespace projectaria::tools::vrs_check {

class Audio : public Periodic {
 public:
  Audio(vrs::StreamId streamId, float minScore);
  bool setup(vrs::RecordFileReader& reader) override; // Setup the audio player

 private:
  void processData(const data_provider::AudioDataRecord& record);
  std::unique_ptr<data_provider::AudioPlayer> audioPlayer_;
  // Whether the callback is being used during the preprocess or analysis phase
  bool preprocess_ = false;
  // Number of samples per each data record, assume the same for each record
  uint64_t samplesPerRecord_;
};

} // namespace projectaria::tools::vrs_check
