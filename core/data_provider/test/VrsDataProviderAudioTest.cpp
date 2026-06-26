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

#include <data_provider/data_layout/AudioMetadata.h>
#include <data_provider/players/AudioPlayer.h>

#include <gtest/gtest.h>

#include <vrs/DataSource.h>
#include <vrs/RecordFileReader.h>
#include <vrs/RecordFileWriter.h>
#include <vrs/RecordFormat.h>
#include <vrs/Recordable.h>
#include <vrs/os/Utils.h>

#include <cstdint>
#include <limits>
#include <vector>

using namespace projectaria::tools::data_provider;

namespace {

constexpr uint8_t kNumChannels = 2;
constexpr uint32_t kSampleRate = 16000;

// Writes a single PCM/S16 stereo audio stream using the same datalayout the Aria
// recorder produces. The audio content block is the trailing block, so VRS
// derives the sample count from the remaining payload size (the recorder omits
// an explicit sample-count field).
class PcmS16AudioRecordable : public vrs::Recordable {
 public:
  explicit PcmS16AudioRecordable(std::vector<int16_t> interleavedSamples)
      : vrs::Recordable(vrs::RecordableTypeId::StereoAudioRecordableClass, "test/microphone"),
        samples_(std::move(interleavedSamples)) {
    addRecordFormat(vrs::Record::Type::CONFIGURATION, 1, config_.getContentBlock(), {&config_});
    addRecordFormat(
        vrs::Record::Type::DATA,
        1,
        data_.getContentBlock() + vrs::ContentBlock(vrs::AudioFormat::PCM, kNumChannels),
        {&data_});
  }

  const vrs::Record* createConfigurationRecord() override {
    config_.streamId.set(0);
    config_.numChannels.set(kNumChannels);
    config_.sampleRate.set(kSampleRate);
    config_.sampleFormat.set(static_cast<uint8_t>(vrs::AudioSampleFormat::S16_LE));
    return createRecord(0.0, vrs::Record::Type::CONFIGURATION, 1, vrs::DataSource(config_));
  }

  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }

  void writeDataRecord() {
    const auto numFrames = static_cast<uint32_t>(samples_.size() / kNumChannels);
    std::vector<int64_t> captureTimestampsNs(numFrames);
    for (uint32_t i = 0; i < numFrames; ++i) {
      captureTimestampsNs[i] = static_cast<int64_t>(i);
    }
    data_.captureTimestampsNs.stage(captureTimestampsNs);
    data_.audioMuted.set(0);
    createRecord(
        1.0,
        vrs::Record::Type::DATA,
        1,
        vrs::DataSource(
            data_, vrs::DataSourceChunk(samples_.data(), samples_.size() * sizeof(int16_t))));
  }

 private:
  datalayout::AudioConfigRecordMetadata config_;
  datalayout::AudioDataRecordMetadata data_;
  std::vector<int16_t> samples_;
};

// Removes the temp file on scope exit so an early ASSERT failure cannot leak it.
struct TempFileGuard {
  std::string path;
  ~TempFileGuard() {
    if (!path.empty()) {
      vrs::os::remove(path);
    }
  }
};

void writeTempPcmS16Vrs(const std::string& path, const std::vector<int16_t>& samples) {
  vrs::RecordFileWriter writer;
  PcmS16AudioRecordable recordable(samples);
  writer.addRecordable(&recordable);
  recordable.createConfigurationRecord();
  recordable.createStateRecord();
  recordable.writeDataRecord();
  ASSERT_EQ(writer.writeToFile(path), 0);
}

} // namespace

// AudioPlayer must read a raw PCM/S16 stream (it used to throw "Unsupported audio
// sample format: int16le"), widening the int16 samples verbatim to int32 and
// reporting INT16_MAX as the amplitude scale so amplitude-normalizing consumers
// stay correct regardless of the on-disk sample width.
TEST(AudioPlayerTest, ReadsPcmS16WidenedToInt32) {
  // Independently chosen interleaved [L, R] frames spanning the int16 range.
  const std::vector<int16_t> samples = {0, -1, 100, -100, 32767, -32768, 1234, -4321};
  const std::vector<int32_t> expected(samples.begin(), samples.end());

  const std::string path =
      vrs::os::getUniquePath(vrs::os::getTempFolder() + "projectaria_pcm_s16_audio_test");
  TempFileGuard guard{path};
  ASSERT_NO_FATAL_FAILURE(writeTempPcmS16Vrs(path, samples));

  vrs::RecordFileReader reader;
  ASSERT_EQ(reader.openFile(path), 0);

  vrs::StreamId audioStreamId;
  for (const auto& id : reader.getStreams()) {
    if (id.getTypeId() == vrs::RecordableTypeId::StereoAudioRecordableClass) {
      audioStreamId = id;
      break;
    }
  }
  ASSERT_TRUE(audioStreamId.isValid());

  AudioPlayer player(audioStreamId);
  reader.setStreamPlayer(audioStreamId, &player);
  ASSERT_EQ(reader.readAllRecords(), 0);
  reader.closeFile();

  EXPECT_EQ(player.getConfigRecord().numChannels, kNumChannels);
  EXPECT_EQ(player.getConfigRecord().sampleRate, kSampleRate);
  EXPECT_EQ(player.getDetectedAudioFormat(), vrs::AudioFormat::PCM);

  const AudioData& audioData = player.getData();
  EXPECT_DOUBLE_EQ(audioData.maxAmplitude, std::numeric_limits<int16_t>::max());
  EXPECT_EQ(audioData.data, expected);
}
