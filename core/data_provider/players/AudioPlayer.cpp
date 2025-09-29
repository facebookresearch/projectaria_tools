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
#include <vrs/RecordFormat.h>

namespace projectaria::tools::data_provider {

static constexpr uint32_t kStereoMultiplier = 2;

// Destructor to clean up Opus decoder
AudioPlayer::~AudioPlayer() {
  if (opusDecoder_) {
    opus_multistream_decoder_destroy(opusDecoder_);
    opusDecoder_ = nullptr;
  }
}

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

  // Detect and store audio format on first read
  if (detectedAudioFormat_ == vrs::AudioFormat::UNDEFINED) {
    detectedAudioFormat_ = audioSpec.getAudioFormat();
  }

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

  uint32_t sampleRate = audioSpec.getSampleRate();

  // Check if decoder needs to be recreated due to:
  // 1. Spec changes (codec parameters changed)
  // 2. Random access seek (current timestamp < last decoded timestamp)
  double currentTimestamp = r.timestamp;
  bool needsReset = false;

  if (opusDecoder_ != nullptr && !lastDecoderSpec_.isCompatibleWith(audioSpec)) {
    needsReset = true;
  }

  // Reset decoder if reversed timestamp is found
  if (opusDecoder_ != nullptr && lastDecodedTimestamp_ >= 0.0 &&
      currentTimestamp < lastDecodedTimestamp_) {
    needsReset = true;
  }

  // Decoding should only be done ONCE for the same audio data packet
  if (currentTimestamp == lastDecodedTimestamp_) {
    return true;
  }

  if (needsReset) {
    resetOpusDecoder();
  }

  // Initialize decoder if not yet initialized
  if (opusDecoder_ == nullptr) {
    // Separate mono and coupled channels
    uint32_t totalCoupledAudioChannel = kStereoMultiplier * audioSpec.getStereoPairCount();
    uint32_t totalMonoChannel = audioSpec.getChannelCount() - totalCoupledAudioChannel;
    uint32_t totalAudioStreamCount = totalMonoChannel + audioSpec.getStereoPairCount();

    // Validate channel counts
    if (numChannels > 255 || numChannels == 0) {
      fmt::print("ERROR: Invalid channel count of {}\n", numChannels);
      return false;
    }
    if (numChannels < totalCoupledAudioChannel) {
      fmt::print(
          "ERROR: Invalid channel count of {} and stereo channel count of {}\n",
          numChannels,
          totalCoupledAudioChannel);
      return false;
    }

    // Create mapping to map input channels to output channels, as required by Opus
    std::vector<uint8_t> mapping(numChannels);
    for (uint32_t i = 0; i < totalCoupledAudioChannel + totalMonoChannel; ++i) {
      mapping[i] = i;
    }

    // Create persistent Opus decoder
    int errorCode = 0;
    opusDecoder_ = opus_multistream_decoder_create(
        sampleRate,
        numChannels,
        totalAudioStreamCount,
        audioSpec.getStereoPairCount(),
        mapping.data(),
        &errorCode);

    if (errorCode != OPUS_OK || opusDecoder_ == nullptr) {
      fmt::print(
          "ERROR: Couldn't create Opus decoder. Error {}: {}\n",
          errorCode,
          opus_strerror(errorCode));
      return false;
    }

    // Store the spec for compatibility checking
    lastDecoderSpec_ = audioSpec;
  }

  // Add fallback for sample count
  if (numSamples == 0) {
    // Use maximum possible according to Opus spec: 120 ms
    numSamples = sampleRate * 120 / 1000;
  }

  // Reserve raw buffers for input and output of decoding
  size_t payloadSize = cb.getBlockSize();
  std::vector<unsigned char> decodeInput(payloadSize);
  std::vector<int16_t> decodeOutput((size_t)numSamples * numChannels);

  // First, read audio data into raw buffer (payload)
  if (r.reader->read(decodeInput) != 0) {
    fmt::print("ERROR: Cannot read audio data into decoding payload\n");
    return false;
  }

  // Then perform decoding using persistent decoder
  opus_int32 result = opus_multistream_decode(
      opusDecoder_,
      decodeInput.data(),
      payloadSize,
      reinterpret_cast<int16_t*>(decodeOutput.data()),
      numSamples,
      0);

  // Fix error checking: result > 0 means success, not == 0
  if (result > 0) {
    // Success: result contains actual number of decoded samples
    data_.data.clear();
    data_.data.reserve(result * numChannels);

    // Only process the actual decoded samples (not the full buffer)
    for (int i = 0; i < result * numChannels; ++i) {
      data_.data.push_back(static_cast<int32_t>(decodeOutput[i]));
    }
    data_.maxAmplitude = static_cast<double>(std::numeric_limits<int16_t>::max());

    // Update timestamp tracking after successful decode
    lastDecodedTimestamp_ = currentTimestamp;

    callback_(data_, dataRecord_, configRecord_, verbose_);

    return true;
  } else {
    // Error: result contains error code
    fmt::print("ERROR: Audio decoding failed. Error {}: {}\n", result, opus_strerror(result));
    return false;
  }
}

void AudioPlayer::resetOpusDecoder() {
  // Destroy existing decoder
  if (opusDecoder_ != nullptr) {
    opus_multistream_decoder_destroy(opusDecoder_);
    opusDecoder_ = nullptr;
    lastDecodedTimestamp_ = -1.0; // Reset timestamp tracking
  }
}

} // namespace projectaria::tools::data_provider
