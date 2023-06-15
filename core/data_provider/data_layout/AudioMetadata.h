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

#include <cstdint>

#include <vrs/DataLayout.h>
#include <vrs/DataLayoutConventions.h>
#include <vrs/DataPieces.h>

// Note: The VRS stream type for audio data is
// vrs::RecordableTypeId::StereoAudioRecordableClass.

namespace datalayout {

struct AudioConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;

  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};

  // Number of channels in the audio stream.
  vrs::DataPieceValue<std::uint8_t> numChannels{vrs::datalayout_conventions::kAudioChannelCount};

  // Number of samples per second. Typical values: 44100Hz.
  vrs::DataPieceValue<std::uint32_t> sampleRate{vrs::datalayout_conventions::kAudioSampleRate};

  // Format of each subsample, deciding the bits and type per subsample.
  // Convertible from vrs::AudioSampleFormat.
  vrs::DataPieceValue<std::uint8_t> sampleFormat{vrs::datalayout_conventions::kAudioSampleFormat};

  vrs::AutoDataLayoutEnd endLayout;
};

struct AudioDataRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;

  // A list of timestamps of each sample in the block, following the same order they are stored in
  // the ContentBlock.
  vrs::DataPieceVector<std::int64_t> captureTimestampsNs{"capture_timestamps_ns"};

  // Set 1 for muted, 0 otherwise.
  vrs::DataPieceValue<std::uint8_t> audioMuted{"audio_muted"};

  vrs::AutoDataLayoutEnd endLayout;
};

} // namespace datalayout
