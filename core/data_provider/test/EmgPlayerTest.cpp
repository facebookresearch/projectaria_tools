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

#include <data_provider/players/EmgPlayer.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

using namespace projectaria::tools::data_provider;

namespace {
// Pack sample-major ADC counts into a big-endian (most-significant byte first) blob, the inverse of
// the layout decodeEmgSamples is expected to unpack.
std::string packBigEndian(const std::vector<uint16_t>& counts) {
  std::string blob;
  blob.reserve(counts.size() * sizeof(uint16_t));
  for (const uint16_t value : counts) {
    blob.push_back(static_cast<char>((value >> 8) & 0xFF));
    blob.push_back(static_cast<char>(value & 0xFF));
  }
  return blob;
}

EmgImuSample makeSample(const std::vector<uint16_t>& counts, uint32_t encoding = 0) {
  EmgImuSample sample;
  sample.packedChannelData = packBigEndian(counts);
  sample.encoding = encoding;
  return sample;
}
} // namespace

TEST(DecodeEmgSamples, decodesBigEndianSampleMajorBatch) {
  EmgData data;
  data.channelCount = 2;
  data.samplesPerBatch = 2;
  data.bitsPerAdcReading = 16;
  // Two samples, each carrying samplesPerBatch * channelCount counts in sample-major order.
  data.emg.push_back(makeSample({0x0102, 0x0304, 0x0506, 0x0708}));
  data.emg.push_back(makeSample({0x1112, 0x1314, 0x1516, 0x1718}));

  const DecodedEmgSamples decoded = decodeEmgSamples(data);

  const std::vector<uint16_t> expectedValues{
      0x0102, 0x0304, 0x0506, 0x0708, 0x1112, 0x1314, 0x1516, 0x1718};
  EXPECT_EQ(decoded.values, expectedValues);
  EXPECT_EQ(decoded.numRows, 4u); // samplesPerBatch (2) * number of samples (2)
  EXPECT_EQ(decoded.numChannels, 2u);
}

TEST(DecodeEmgSamples, skipsMalformedBlobAndKeepsWellFormedSamples) {
  EmgData data;
  data.channelCount = 2;
  data.samplesPerBatch = 2;
  data.bitsPerAdcReading = 16;
  data.emg.push_back(makeSample({0x0102, 0x0304, 0x0506, 0x0708}));
  EmgImuSample malformed;
  malformed.packedChannelData =
      std::string(3, '\0'); // not samplesPerBatch * channelCount * 2 bytes
  data.emg.push_back(malformed);

  const DecodedEmgSamples decoded = decodeEmgSamples(data);

  const std::vector<uint16_t> expectedValues{0x0102, 0x0304, 0x0506, 0x0708};
  EXPECT_EQ(decoded.values, expectedValues);
  EXPECT_EQ(decoded.numRows, 2u);
}

TEST(DecodeEmgSamples, returnsEmptyWhenChannelCountIsZero) {
  EmgData data;
  data.channelCount = 0;
  data.samplesPerBatch = 2;
  data.bitsPerAdcReading = 16;

  const DecodedEmgSamples decoded = decodeEmgSamples(data);

  EXPECT_TRUE(decoded.values.empty());
  EXPECT_EQ(decoded.numRows, 0u);
  EXPECT_EQ(decoded.numChannels, 0u);
}

TEST(DecodeEmgSamples, throwsOnUnsupportedBitDepth) {
  EmgData data;
  data.channelCount = 2;
  data.samplesPerBatch = 2;
  data.bitsPerAdcReading = 12;

  EXPECT_THROW(decodeEmgSamples(data), std::invalid_argument);
}

TEST(DecodeEmgSamples, throwsOnNonZeroEncoding) {
  EmgData data;
  data.channelCount = 2;
  data.samplesPerBatch = 2;
  data.bitsPerAdcReading = 16;
  data.emg.push_back(makeSample({0x0102, 0x0304, 0x0506, 0x0708}, /*encoding=*/1));

  EXPECT_THROW(decodeEmgSamples(data), std::invalid_argument);
}
