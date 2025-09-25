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

#include <cassert>
#include <cmath>

#include "AudioPlayer.h"

#include <vrs/ErrorCode.h>
#include <vrs/RecordFileReader.h>

namespace projectaria::tools::data_provider {

bool AudioPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config = getExpectedLayout<datalayout::AudioConfigRecordMetadata>(dl, blockIndex);
    configRecord_.streamId = config.streamId.get();
    configRecord_.numChannels = config.numChannels.get();
    configRecord_.sampleRate = config.sampleRate.get();
    configRecord_.sampleFormat = config.sampleFormat.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::AudioDataRecordMetadata>(dl, blockIndex);
    data.captureTimestampsNs.get(dataRecord_.captureTimestampsNs);
    dataRecord_.audioMuted = data.audioMuted.get();
    nextTimestampSec_ = std::nextafter(r.timestamp, std::numeric_limits<double>::max());
  }
  return true;
}

bool AudioPlayer::onAudioRead(
    const vrs::CurrentRecord& r,
    size_t /* idx */,
    const vrs::ContentBlock& cb) {
  const auto& audioSpec = cb.audio();
  assert(audioSpec.getSampleFormat() == vrs::AudioSampleFormat::S32_LE);
  data_.data.clear();
  std::vector<int32_t> dataVec(audioSpec.getSampleCount() * audioSpec.getChannelCount());
  if (r.reader->read(dataVec) == 0) {
    // Actually read the audio data
    data_.data = dataVec;
    callback_(data_, dataRecord_, configRecord_, verbose_);
    if (verbose_) {
      fmt::print(
          "{:.3f} {} [{}]: {} {}x{} samples.\n",
          r.timestamp,
          r.streamId.getName(),
          r.streamId.getNumericName(),
          audioSpec.asString(),
          audioSpec.getSampleCount(),
          audioSpec.getChannelCount());
    }
  }
  return true;
}

} // namespace projectaria::tools::data_provider
