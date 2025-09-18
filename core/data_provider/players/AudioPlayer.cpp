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

#include <opus.h>
#include <opus_multistream.h>
#include <vrs/ErrorCode.h>
#include <vrs/RecordFileReader.h>

namespace projectaria::tools::data_provider {

static constexpr uint32_t kStereoMultiplier = 2;

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
  auto& audioSpec = cb.audio();

  // Read Opus-encoded audio data with decoding
  if (audioSpec.getAudioFormat() == vrs::AudioFormat::OPUS &&
      audioSpec.getSampleFormat() == vrs::AudioSampleFormat::S16_LE) {
    bool decodeSuccessful = readAndDecodeAudioData(r, cb);
    if (!decodeSuccessful) {
      return false;
    }
  }
  // Read normal audio data
  else if (audioSpec.getSampleFormat() == vrs::AudioSampleFormat::S32_LE) {
    data_.data.clear();
    std::vector<int32_t> dataVec(
        audioSpec.getSampleCount() * static_cast<size_t>(audioSpec.getChannelCount()));
    if (r.reader->read(dataVec) == 0) {
      // Actually read the audio data
      data_.data = dataVec;
      data_.maxAmplitude = static_cast<double>(std::numeric_limits<int32_t>::max());
      callback_(data_, dataRecord_, configRecord_, verbose_);
    }
  } else {
    throw std::runtime_error(
        fmt::format("Unsupported audio sample format: {}", audioSpec.getSampleFormatAsString()));
  }

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
  return true;
}

bool AudioPlayer::readAndDecodeAudioData(const vrs::CurrentRecord& r, const vrs::ContentBlock& cb) {
  // Specs from ContentBlock
  const auto& audioSpec = cb.audio();
  uint32_t numChannels = audioSpec.getChannelCount();
  uint32_t numSamples = audioSpec.getSampleCount();

  // Separate mono and coupled channels
  uint32_t totalCoupledAudioChannel = kStereoMultiplier * audioSpec.getStereoPairCount();
  uint32_t totalMonoChannel = audioSpec.getChannelCount() - totalCoupledAudioChannel;
  uint32_t totalAudioStreamCount = totalMonoChannel + audioSpec.getStereoPairCount();
  uint32_t sampleRate = audioSpec.getSampleRate();

  // Create a mapping to map input channels to output channels, as required by Opus.
  std::vector<uint8_t> mapping(numChannels);
  for (uint32_t i = 0; i < numChannels; ++i) {
    mapping[i] = i;
  }

  // Create Opus decoder
  int errorCode = 0;
  auto* decoder = opus_multistream_decoder_create(
      sampleRate,
      numChannels,
      totalAudioStreamCount,
      audioSpec.getStereoPairCount(),
      mapping.data(),
      &errorCode);

  if (errorCode != OPUS_OK || decoder == nullptr) {
    fmt::print(
        "WARNING: Couldn't create Opus decoder. Error {}: {}\n",
        errorCode,
        opus_strerror(errorCode));
    return false;
  }

  // Reserve raw buffers for input and output of decoding
  size_t payloadSize = cb.getBlockSize();
  std::vector<unsigned char> decodeInput(payloadSize);
  std::vector<int16_t> decodeOutput((size_t)numSamples * numChannels);

  // First, read audio data into raw buffer (payload)
  if (r.reader->read(decodeInput) != 0) {
    fmt::print("ERROR: Cannot read audio data into decoding payload \n");
    return false;
  }

  // Then perform decoding
  opus_int32 result = opus_multistream_decode(
      decoder,
      decodeInput.data(),
      payloadSize,
      reinterpret_cast<int16_t*>(decodeOutput.data()),
      numSamples,
      0);
  // Check if valid number of data has been decoded
  if (result == 0) {
    fmt::print("ERROR: Audio decoding has returned 0 valid decoded samples. \n");
    return false;
  }

  data_.data.clear();
  for (const auto& decodeValue : decodeOutput) {
    data_.data.push_back(static_cast<int32_t>(decodeValue));
  }
  data_.maxAmplitude = static_cast<double>(std::numeric_limits<int16_t>::max());
  callback_(data_, dataRecord_, configRecord_, verbose_);

  // Cleaning up
  opus_multistream_decoder_destroy(decoder);
  return true;
}

} // namespace projectaria::tools::data_provider
