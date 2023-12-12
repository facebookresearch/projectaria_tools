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

#include "Audio.h"

#define DEFAULT_LOG_CHANNEL "VrsHealthCheck:Audio"
#include <logging/Log.h>

namespace projectaria::tools::vrs_check {

Audio::Audio(vrs::StreamId streamId, float minScore) : Periodic(streamId, minScore) {}

bool Audio::setup(vrs::RecordFileReader& reader) {
  data_provider::AudioCallback callback = [&](const data_provider::AudioData&,
                                              const data_provider::AudioDataRecord& record,
                                              const data_provider::AudioConfig&,
                                              const bool) {
    processData(record);
    return true;
  };
  audioPlayer_ = std::make_unique<data_provider::AudioPlayer>(streamId_);
  if (!audioPlayer_) {
    XR_LOGE("Cannot create playable for {}", streamId_.getName());
    return false;
  }
  audioPlayer_->setCallback(callback);
  reader.setStreamPlayer(streamId_, audioPlayer_.get());

  // Parse the configuration record
  if (!reader.readFirstConfigurationRecord(streamId_)) {
    XR_LOGE("Stream {} is missing a configuration record", streamId_.getName());
    return false;
  }

  const auto& config = audioPlayer_->getConfigRecord();
  periodUs_ = std::round(1 / static_cast<double>(config.sampleRate) * 1e6);
  setMaxDeviationFromPeriodUs();

  // Calculate total and expected
  if (reader.getRecordCount(streamId_, vrs::Record::Type::DATA) > 0) {
    const auto* firstRecord = reader.getRecordByTime(streamId_, vrs::Record::Type::DATA, 0.0);
    // Enter preprocess mode so the callback can decode samples per record
    preprocess_ = true;
    if (reader.readRecord(*firstRecord)) {
      XR_LOGE("{}: Failed to read the first record", streamId_.getName());
      return false;
    }
    // Exit preprocess mode
    preprocess_ = false;
    const uint64_t firstTimestampUs = firstRecord->timestamp * 1e6;
    const uint64_t lastTimestampUs =
        reader.getLastRecord(streamId_, vrs::Record::Type::DATA)->timestamp * 1e6;
    stats_.total = samplesPerRecord_ * reader.getRecordCount(streamId_, vrs::Record::Type::DATA);
    stats_.expected =
        std::round((lastTimestampUs - firstTimestampUs) / static_cast<float>(periodUs_)) +
        samplesPerRecord_;
  }

  return true;
}

void Audio::processData(const data_provider::AudioDataRecord& record) {
  // This block is used in the preprocess stage
  const int numSamples = record.captureTimestampsNs.size();
  if (preprocess_) {
    samplesPerRecord_ = numSamples;
    return;
  }

  std::lock_guard lock{mutex_};
  for (int i = 0; i < numSamples; i++) {
    processTimestamp(record.captureTimestampsNs[i] / 1e3);
  }
}

} // namespace projectaria::tools::vrs_check
