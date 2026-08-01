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

#include <calibration/NeuralBandEmgCalibration.h>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <tuple>

namespace projectaria::tools::calibration {
namespace {

// Example truncated-EMG parameters (13-bit streamed samples with 3 LSBs dropped).
constexpr float kTruncatedAnalogGain = 30.0f;
constexpr float kTruncatedVrefPos = 1.6f;
constexpr float kTruncatedVrefNeg = 0.0f;
constexpr float kTruncatedVrefDc = 0.0f;
constexpr uint32_t kTruncatedStreamedBitWidth = 13;
constexpr uint32_t kTruncatedDroppedLsb = 3;

// Opaque chip code; tests only assert it round-trips as an integer.
constexpr uint32_t kOpaqueChipCode = 3;

NeuralBandEmgCalibration makeTruncatedCalibration() {
  return NeuralBandEmgCalibration{
      "emg",
      kTruncatedAnalogGain,
      /*adcCountLevels=*/65536, // full 16-bit; unused for truncated math
      kTruncatedVrefPos,
      kTruncatedVrefNeg,
      kTruncatedVrefDc,
      /*isTruncated=*/true,
      kTruncatedStreamedBitWidth,
      kTruncatedDroppedLsb,
      kOpaqueChipCode};
}

// Independently expressed reference for the truncated formula, using unshifted
// signed math so an impl error in the unsigned+cast trick can't mirror-match.
double referenceTruncatedVolts(
    uint32_t adcCount,
    uint32_t streamedBitWidth,
    uint32_t droppedLsb,
    float vrefPos,
    float vrefNeg,
    float analogGain) {
  const int64_t signedAdc = static_cast<int64_t>(adcCount) - (int64_t{1} << (streamedBitWidth - 1));
  const int64_t lsbAdjusted = signedAdc * (int64_t{1} << droppedLsb);
  const double fsr = std::ldexp(1.0, static_cast<int>(droppedLsb + streamedBitWidth));
  return static_cast<double>(lsbAdjusted) / fsr * static_cast<double>(vrefPos - vrefNeg) /
      static_cast<double>(analogGain);
}

// Independently expressed reference for the non-truncated formula. Kept as
// a separate expression so an implementation-side transcription error can't
// mirror-match the reference.
double referenceNonTruncatedVolts(
    uint32_t adcCount,
    uint32_t adcCountLevels,
    float vrefPos,
    float vrefNeg,
    float vrefDc,
    float analogGain) {
  return ((static_cast<double>(adcCount) * (vrefPos - vrefNeg) / adcCountLevels) - vrefDc) /
      analogGain;
}

} // namespace

TEST(NeuralBandEmgCalibrationTest, Truncated_ZeroPointMapsToZeroVolts) {
  const auto calib = makeTruncatedCalibration();
  const uint32_t zeroPoint = 1u << (kTruncatedStreamedBitWidth - 1);
  EXPECT_DOUBLE_EQ(calib.adcToVolts(zeroPoint), 0.0);
}

TEST(NeuralBandEmgCalibrationTest, Truncated_MatchesReferenceOnPositiveSample) {
  const auto calib = makeTruncatedCalibration();
  constexpr uint32_t kSample = 5000; // above zero point 4096
  const double expected = referenceTruncatedVolts(
      kSample,
      kTruncatedStreamedBitWidth,
      kTruncatedDroppedLsb,
      kTruncatedVrefPos,
      kTruncatedVrefNeg,
      kTruncatedAnalogGain);
  EXPECT_NEAR(calib.adcToVolts(kSample), expected, 1e-12);
}

TEST(NeuralBandEmgCalibrationTest, Truncated_MatchesReferenceOnNegativeSample) {
  const auto calib = makeTruncatedCalibration();
  constexpr uint32_t kSample = 3000; // below zero point 4096
  const double expected = referenceTruncatedVolts(
      kSample,
      kTruncatedStreamedBitWidth,
      kTruncatedDroppedLsb,
      kTruncatedVrefPos,
      kTruncatedVrefNeg,
      kTruncatedAnalogGain);
  EXPECT_NEAR(calib.adcToVolts(kSample), expected, 1e-12);
}

TEST(NeuralBandEmgCalibrationTest, Truncated_ExtremesMapToBoundedFraction) {
  const auto calib = makeTruncatedCalibration();
  const double voltsAtMin = calib.adcToVolts(0);
  const double voltsAtMax = calib.adcToVolts((1u << kTruncatedStreamedBitWidth) - 1);
  const double vrangeOverGain =
      static_cast<double>(kTruncatedVrefPos - kTruncatedVrefNeg) / kTruncatedAnalogGain;
  // Truncated formula anchors on 2^bw / 2 → span is (-0.5, +0.5) of vrange/gain.
  EXPECT_LT(voltsAtMin, 0.0);
  EXPECT_GT(voltsAtMax, 0.0);
  EXPECT_NEAR(voltsAtMin, -0.5 * vrangeOverGain, 1e-12);
  EXPECT_NEAR(
      voltsAtMax, (0.5 - std::ldexp(1.0, -kTruncatedStreamedBitWidth)) * vrangeOverGain, 1e-12);
}

TEST(NeuralBandEmgCalibrationTest, NonTruncated_ZeroVoltsWhenAdcInputEqualsDcBias) {
  // vrefDc = (vrefPos-vrefNeg)/2, so adcCount=levels/2 sits at the DC bias
  // point: the non-truncated formula returns 0 there.
  const NeuralBandEmgCalibration calib{
      "emg",
      /*analogGain=*/380.0f,
      /*adcCountLevels=*/65536,
      /*adcVrefPos=*/5.0f,
      /*adcVrefNeg=*/0.0f,
      /*adcVrefDc=*/2.5f,
      /*isTruncated=*/false,
      /*streamedBitWidth=*/0,
      /*droppedLsb=*/0,
      /*adcChipCode=*/1};
  EXPECT_DOUBLE_EQ(calib.adcToVolts(32768u), 0.0);
}

TEST(NeuralBandEmgCalibrationTest, NonTruncated_MatchesXplatReferenceAcrossRange) {
  const NeuralBandEmgCalibration calib{
      "emg",
      /*analogGain=*/380.0f,
      /*adcCountLevels=*/65536,
      /*adcVrefPos=*/5.0f,
      /*adcVrefNeg=*/0.0f,
      /*adcVrefDc=*/2.5f,
      /*isTruncated=*/false,
      /*streamedBitWidth=*/0,
      /*droppedLsb=*/0,
      /*adcChipCode=*/1};
  for (const uint32_t sample : {0u, 1u, 1000u, 32768u, 40000u, 65535u}) {
    const double expected = referenceNonTruncatedVolts(sample, 65536, 5.0f, 0.0f, 2.5f, 380.0f);
    EXPECT_NEAR(calib.adcToVolts(sample), expected, 1e-12) << "sample " << sample;
  }
}

TEST(NeuralBandEmgCalibrationTest, NonTruncated_UsesVrefDcExplicitly) {
  // vrefDc deliberately offset from vrange/2; a bug that centered on
  // adcCountLevels/2 instead of subtracting adcVrefDc would return 0 at
  // adcCount = 512 and thus fail this test.
  const NeuralBandEmgCalibration calib{
      "emg",
      /*analogGain=*/1.0f,
      /*adcCountLevels=*/1024,
      /*adcVrefPos=*/5.0f,
      /*adcVrefNeg=*/1.0f,
      /*adcVrefDc=*/3.0f, // NOT (5-1)/2 = 2; deliberately offset
      /*isTruncated=*/false,
      0,
      0,
      /*adcChipCode=*/0};
  // ((512 * 4/1024) - 3) / 1 = (2 - 3) = -1 V
  EXPECT_DOUBLE_EQ(calib.adcToVolts(512u), -1.0);
  // ((768 * 4/1024) - 3) / 1 = (3 - 3) = 0 V (the vrefDc point)
  EXPECT_DOUBLE_EQ(calib.adcToVolts(768u), 0.0);
}

TEST(NeuralBandEmgCalibrationTest, VectorizedAdcToVoltsMatchesPerSample) {
  const auto calib = makeTruncatedCalibration();
  const std::vector<uint16_t> raw{0, 1000, 4096, 5000, 8000};
  const std::vector<double> got = calib.adcToVolts(raw);
  ASSERT_EQ(got.size(), raw.size());
  for (size_t i = 0; i < raw.size(); ++i) {
    EXPECT_DOUBLE_EQ(got[i], calib.adcToVolts(static_cast<uint32_t>(raw[i]))) << "sample " << i;
  }
}

TEST(NeuralBandEmgCalibrationTest, Truncated_VoltsToAdcRoundTripsOnGrid) {
  const auto calib = makeTruncatedCalibration();
  for (const uint32_t sample : {0u, 1u, 100u, 4096u, 5000u, 7000u, 8191u}) {
    EXPECT_EQ(calib.voltsToAdc(calib.adcToVolts(sample)), sample) << "sample " << sample;
  }
}

TEST(NeuralBandEmgCalibrationTest, NonTruncated_VoltsToAdcRoundTripsOnGrid) {
  const NeuralBandEmgCalibration calib{
      "emg", 380.0f, 65536, 5.0f, 0.0f, 2.5f, false, 0, 0, /*adcChipCode=*/1};
  for (const uint32_t sample : {0u, 1u, 32768u, 40000u, 65535u}) {
    EXPECT_EQ(calib.voltsToAdc(calib.adcToVolts(sample)), sample) << "sample " << sample;
  }
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_EmptyStringReturnsNullopt) {
  EXPECT_FALSE(NeuralBandEmgCalibration::fromParamsJson("").has_value());
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_MalformedJsonReturnsNullopt) {
  EXPECT_FALSE(NeuralBandEmgCalibration::fromParamsJson("{not_json").has_value());
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_NonObjectRootReturnsNullopt) {
  EXPECT_FALSE(NeuralBandEmgCalibration::fromParamsJson("[]").has_value());
  EXPECT_FALSE(NeuralBandEmgCalibration::fromParamsJson("42").has_value());
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_MissingRequiredFieldReturnsNullopt) {
  // Missing analog_gain.
  EXPECT_FALSE(
      NeuralBandEmgCalibration::fromParamsJson(R"({"daq_vref_pos":5.0,"daq_vref_neg":0.0})")
          .has_value());
  // Missing daq_vref_pos.
  EXPECT_FALSE(
      NeuralBandEmgCalibration::fromParamsJson(R"({"analog_gain":380.0,"daq_vref_neg":0.0})")
          .has_value());
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_ParsesTruncatedExample) {
  // Example device.emg_calibration JSON blob for a truncated 13-bit stream.
  const std::string json = R"({
    "analog_gain": 30.0,
    "daq_bits": 16,
    "daq_range_max": 8191,
    "daq_vref_dc": 0.8,
    "daq_vref_neg": 0.0,
    "daq_vref_pos": 1.6,
    "daq_zero_point": 4095,
    "emg_adc_chip": 3,
    "emg_truncation_bits_transmitted_per_sample": 13,
    "emg_truncation_dropped_lsb": 3,
    "emg_truncation_dropped_msb": 0
  })";
  const auto calib = NeuralBandEmgCalibration::fromParamsJson(json, "emg");
  ASSERT_TRUE(calib.has_value());
  EXPECT_EQ(calib->getLabel(), "emg");
  EXPECT_FLOAT_EQ(calib->getAnalogGain(), 30.0f);
  EXPECT_EQ(calib->getAdcCountLevels(), 8192u);
  EXPECT_FLOAT_EQ(calib->getAdcVrefPos(), 1.6f);
  EXPECT_FLOAT_EQ(calib->getAdcVrefNeg(), 0.0f);
  EXPECT_FLOAT_EQ(calib->getAdcVrefDc(), 0.8f);
  EXPECT_TRUE(calib->isTruncated());
  EXPECT_EQ(calib->getStreamedBitWidth(), 13u);
  EXPECT_EQ(calib->getDroppedLsb(), 3u);
  EXPECT_EQ(calib->getAdcChipCode(), 3u);
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_TruncationOffWhenStreamedBitWidthAbsent) {
  const std::string json = R"({
    "analog_gain": 380.0,
    "daq_range_max": 65535,
    "daq_vref_pos": 5.0,
    "daq_vref_neg": 0.0,
    "daq_vref_dc": 2.5,
    "emg_adc_chip": 1
  })";
  const auto calib = NeuralBandEmgCalibration::fromParamsJson(json);
  ASSERT_TRUE(calib.has_value());
  EXPECT_FALSE(calib->isTruncated());
  EXPECT_EQ(calib->getAdcCountLevels(), 65536u);
  EXPECT_EQ(calib->getAdcChipCode(), 1u);
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_AdcChipCodePassedThroughAsOpaqueUint) {
  const std::string json = R"({
    "analog_gain": 1.0,
    "daq_vref_pos": 1.0,
    "daq_vref_neg": 0.0,
    "emg_adc_chip": 99
  })";
  const auto calib = NeuralBandEmgCalibration::fromParamsJson(json);
  ASSERT_TRUE(calib.has_value());
  EXPECT_EQ(calib->getAdcChipCode(), 99u);
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_ZeroAnalogGainReturnsNullopt) {
  const std::string json = R"({"analog_gain":0.0,"daq_vref_pos":5.0,"daq_vref_neg":0.0})";
  EXPECT_FALSE(NeuralBandEmgCalibration::fromParamsJson(json).has_value());
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_VrefPosLessThanNegReturnsNullopt) {
  const std::string json = R"({"analog_gain":1.0,"daq_vref_pos":0.0,"daq_vref_neg":5.0})";
  EXPECT_FALSE(NeuralBandEmgCalibration::fromParamsJson(json).has_value());
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_DaqRangeMaxUint32MaxReturnsNullopt) {
  const std::string json = R"({
    "analog_gain": 1.0,
    "daq_vref_pos": 5.0,
    "daq_vref_neg": 0.0,
    "daq_range_max": 4294967295
  })";
  EXPECT_FALSE(NeuralBandEmgCalibration::fromParamsJson(json).has_value());
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_TruncationOutOfRangeReturnsNullopt) {
  // streamedBitWidth > 31.
  EXPECT_FALSE(
      NeuralBandEmgCalibration::fromParamsJson(R"({
    "analog_gain": 1.0, "daq_vref_pos": 1.0, "daq_vref_neg": 0.0,
    "emg_truncation_bits_transmitted_per_sample": 32,
    "emg_truncation_dropped_lsb": 0
  })")
          .has_value());
  // streamedBitWidth + droppedLsb > 31.
  EXPECT_FALSE(
      NeuralBandEmgCalibration::fromParamsJson(R"({
    "analog_gain": 1.0, "daq_vref_pos": 1.0, "daq_vref_neg": 0.0,
    "emg_truncation_bits_transmitted_per_sample": 20,
    "emg_truncation_dropped_lsb": 15
  })")
          .has_value());
}

TEST(NeuralBandEmgCalibrationTest, FromParamsJson_AcceptsFloatEncodedUintFields) {
  // Writer may emit integer-valued fields as JSON floats (e.g. `8191.0`);
  // getUintOr must accept these rather than silently falling back.
  const std::string json = R"({
    "analog_gain": 1.0,
    "daq_vref_pos": 1.0,
    "daq_vref_neg": 0.0,
    "daq_range_max": 8191.0,
    "emg_adc_chip": 3.0
  })";
  const auto calib = NeuralBandEmgCalibration::fromParamsJson(json);
  ASSERT_TRUE(calib.has_value());
  EXPECT_EQ(calib->getAdcCountLevels(), 8192u);
  EXPECT_EQ(calib->getAdcChipCode(), 3u);
}

TEST(NeuralBandEmgCalibrationTest, AdcToVolts_TruncatedAdcCountOutOfRangeThrows) {
  const auto calib = makeTruncatedCalibration();
  // streamedBitWidth=13 → valid range is [0, 8192). 8192 exceeds.
  EXPECT_THROW(std::ignore = calib.adcToVolts(8192u), std::invalid_argument);
  EXPECT_THROW(std::ignore = calib.adcToVolts(999999u), std::invalid_argument);
}

TEST(NeuralBandEmgCalibrationTest, VoltsToAdc_TruncatedOutOfRangeThrows) {
  const auto calib = makeTruncatedCalibration();
  // Well beyond ±0.5 * vrange/gain (the truncated formula's representable span).
  EXPECT_THROW(std::ignore = calib.voltsToAdc(1e6), std::invalid_argument);
  EXPECT_THROW(std::ignore = calib.voltsToAdc(-1e6), std::invalid_argument);
}

TEST(NeuralBandEmgCalibrationTest, VoltsToAdc_NonTruncatedOutOfRangeThrows) {
  const NeuralBandEmgCalibration calib{
      "emg", 380.0f, 65536, 5.0f, 0.0f, 2.5f, false, 0, 0, /*adcChipCode=*/1};
  EXPECT_THROW(std::ignore = calib.voltsToAdc(1e6), std::invalid_argument);
  EXPECT_THROW(std::ignore = calib.voltsToAdc(-1e6), std::invalid_argument);
}

} // namespace projectaria::tools::calibration
