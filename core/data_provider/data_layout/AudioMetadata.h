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

// Fields Gen2 recordings add to the configuration. Optional, because writers of
// other streams leave them out: an unset format reads as undefined.
struct AudioFormatConfigFields {
  // Format of the audio stream, convertible from vrs::AudioFormat (PCM or Opus).
  vrs::DataPieceValue<std::uint8_t> audioFormat{vrs::datalayout_conventions::kAudioFormat};

  // Number of stereo pairs, for Opus compression.
  vrs::DataPieceValue<std::uint8_t> stereoPairCount{
      vrs::datalayout_conventions::kAudioStereoPairCount};

  // One bit per enabled microphone.
  vrs::DataPieceValue<std::uint8_t> enabledMicIdMask{"enabled_mic_id_mask"};
};

struct AudioConfigRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;

  explicit AudioConfigRecordMetadata(bool allocateFormatFields = false)
      : formatFields(allocateFormatFields) {}

  vrs::DataPieceValue<std::uint32_t> streamId{"stream_id"};

  // Number of channels in the audio stream.
  vrs::DataPieceValue<std::uint8_t> numChannels{vrs::datalayout_conventions::kAudioChannelCount};

  // Number of samples per second. Typical values: 44100Hz.
  vrs::DataPieceValue<std::uint32_t> sampleRate{vrs::datalayout_conventions::kAudioSampleRate};

  // Format of each subsample, deciding the bits and type per subsample.
  // Convertible from vrs::AudioSampleFormat.
  vrs::DataPieceValue<std::uint8_t> sampleFormat{vrs::datalayout_conventions::kAudioSampleFormat};

  const vrs::OptionalDataPieces<AudioFormatConfigFields> formatFields;

  vrs::AutoDataLayoutEnd endLayout;
};

// Fields Gen2 recordings add to each block. Optional, because writers of other
// streams leave them out: an unset sample count reads as an empty block.
struct AudioBlockDataFields {
  // Number of samples in the block.
  vrs::DataPieceValue<std::uint32_t> sampleCount{vrs::datalayout_conventions::kAudioSampleCount};

  // Sequence number of the block.
  vrs::DataPieceValue<std::uint64_t> frameNumber{"frame_number"};

  // Hardware capture time of the sample at `hardwareCaptureFrameIndex`, -1 when unknown.
  vrs::DataPieceValue<std::int64_t> hardwareCaptureTimestampNs{"hardware_capture_timestamp_ns"};

  // Sample index since the stream started that `hardwareCaptureTimestampNs` refers to, -1 when
  // unknown.
  vrs::DataPieceValue<std::int64_t> hardwareCaptureFrameIndex{"hardware_capture_frame_index"};

  // Time the block arrived at the recorder, the source of the record timestamp.
  vrs::DataPieceValue<std::int64_t> arrivalTimestampNs{"arrival_timestamp_ns"};
};

struct AudioDataRecordMetadata : public vrs::AutoDataLayout {
  static constexpr uint32_t kVersion = 2;

  explicit AudioDataRecordMetadata(bool allocateBlockFields = false)
      : blockFields(allocateBlockFields) {}

  // A list of timestamps of each sample in the block, following the same order they are stored in
  // the ContentBlock.
  vrs::DataPieceVector<std::int64_t> captureTimestampsNs{"capture_timestamps_ns"};

  // Set 1 for muted, 0 otherwise.
  vrs::DataPieceValue<std::uint8_t> audioMuted{"audio_muted"};

  const vrs::OptionalDataPieces<AudioBlockDataFields> blockFields;

  vrs::AutoDataLayoutEnd endLayout;
};

} // namespace datalayout
