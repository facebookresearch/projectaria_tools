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

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace projectaria::tools::calibration {

NeuralBandEmgCalibration::NeuralBandEmgCalibration(
    std::string label,
    float analogGain,
    uint32_t adcCountLevels,
    float adcVrefPos,
    float adcVrefNeg,
    float adcVrefDc,
    bool isTruncated,
    uint32_t streamedBitWidth,
    uint32_t droppedLsb,
    uint32_t adcChipCode)
    : label_(std::move(label)),
      analogGain_(analogGain),
      adcCountLevels_(adcCountLevels),
      adcVrefPos_(adcVrefPos),
      adcVrefNeg_(adcVrefNeg),
      adcVrefDc_(adcVrefDc),
      isTruncated_(isTruncated),
      streamedBitWidth_(streamedBitWidth),
      droppedLsb_(droppedLsb),
      adcChipCode_(adcChipCode) {}

const std::string& NeuralBandEmgCalibration::getLabel() const {
  return label_;
}
float NeuralBandEmgCalibration::getAnalogGain() const {
  return analogGain_;
}
uint32_t NeuralBandEmgCalibration::getAdcCountLevels() const {
  return adcCountLevels_;
}
float NeuralBandEmgCalibration::getAdcVrefPos() const {
  return adcVrefPos_;
}
float NeuralBandEmgCalibration::getAdcVrefNeg() const {
  return adcVrefNeg_;
}
float NeuralBandEmgCalibration::getAdcVrefDc() const {
  return adcVrefDc_;
}
bool NeuralBandEmgCalibration::isTruncated() const {
  return isTruncated_;
}
uint32_t NeuralBandEmgCalibration::getStreamedBitWidth() const {
  return streamedBitWidth_;
}
uint32_t NeuralBandEmgCalibration::getDroppedLsb() const {
  return droppedLsb_;
}
uint32_t NeuralBandEmgCalibration::getAdcChipCode() const {
  return adcChipCode_;
}

namespace {

void checkCommonParams(float vrefPos, float vrefNeg, float analogGain, const char* fn) {
  if (vrefPos <= vrefNeg) {
    throw std::invalid_argument(std::string(fn) + ": adcVrefPos must be > adcVrefNeg");
  }
  if (analogGain == 0.0f) {
    throw std::invalid_argument(std::string(fn) + ": analogGain must not be zero");
  }
}

void checkTruncatedParams(uint32_t streamedBitWidth, uint32_t droppedLsb, const char* fn) {
  if (streamedBitWidth == 0 || streamedBitWidth > 31) {
    throw std::invalid_argument(std::string(fn) + ": streamedBitWidth must be in [1, 31]");
  }
  if (droppedLsb > 31 || droppedLsb + streamedBitWidth > 31) {
    throw std::invalid_argument(std::string(fn) + ": droppedLsb + streamedBitWidth must be <= 31");
  }
}

} // namespace

double NeuralBandEmgCalibration::adcToVolts(uint32_t adcCount) const {
  checkCommonParams(adcVrefPos_, adcVrefNeg_, analogGain_, "adcToVolts");

  if (isTruncated_) {
    checkTruncatedParams(streamedBitWidth_, droppedLsb_, "adcToVolts");
    if (adcCount >= (1u << streamedBitWidth_)) {
      throw std::invalid_argument("adcToVolts: adcCount exceeds streamed bit width range");
    }
    // Unsigned math on the recenter + shift avoids UB from left-shifting a
    // negative int; the reinterpret to int32_t preserves the two's-complement
    // bit pattern.
    const uint32_t unsignedAdc = adcCount - (1u << (streamedBitWidth_ - 1));
    const uint32_t lsbAdjusted = unsignedAdc << droppedLsb_;
    const auto signedAdjusted = static_cast<int32_t>(lsbAdjusted);
    const uint32_t fsrBitwidth = droppedLsb_ + streamedBitWidth_;
    const double quantized = static_cast<double>(signedAdjusted) / (1u << fsrBitwidth);
    return quantized *
        (static_cast<double>(adcVrefPos_ - adcVrefNeg_) / static_cast<double>(analogGain_));
  }

  if (adcCountLevels_ == 0) {
    throw std::invalid_argument("adcToVolts: adcCountLevels must be greater than 0");
  }
  return ((static_cast<double>(adcCount) * static_cast<double>(adcVrefPos_ - adcVrefNeg_) /
           static_cast<double>(adcCountLevels_)) -
          static_cast<double>(adcVrefDc_)) /
      static_cast<double>(analogGain_);
}

std::vector<double> NeuralBandEmgCalibration::adcToVolts(
    const std::vector<uint16_t>& adcCounts) const {
  std::vector<double> out;
  out.reserve(adcCounts.size());
  for (const uint16_t v : adcCounts) {
    out.push_back(adcToVolts(static_cast<uint32_t>(v)));
  }
  return out;
}

uint32_t NeuralBandEmgCalibration::voltsToAdc(double volts) const {
  checkCommonParams(adcVrefPos_, adcVrefNeg_, analogGain_, "voltsToAdc");

  if (isTruncated_) {
    checkTruncatedParams(streamedBitWidth_, droppedLsb_, "voltsToAdc");
    const auto vrange = static_cast<double>(adcVrefPos_ - adcVrefNeg_);
    const uint32_t fsrBitwidth = droppedLsb_ + streamedBitWidth_;
    const double signedAdjusted = volts * static_cast<double>(analogGain_) *
        static_cast<double>(uint64_t{1} << fsrBitwidth) / vrange;
    const double shiftBack = std::ldexp(1.0, static_cast<int>(droppedLsb_));
    const int64_t recentered = std::llround(signedAdjusted / shiftBack);
    const int64_t result =
        recentered + static_cast<int64_t>(uint64_t{1} << (streamedBitWidth_ - 1));
    const uint64_t upper = uint64_t{1} << streamedBitWidth_;
    if (result < 0 || std::cmp_greater_equal(result, upper)) {
      throw std::invalid_argument(
          "voltsToAdc: voltage out of representable range for streamed bit width");
    }
    return static_cast<uint32_t>(result);
  }

  if (adcCountLevels_ == 0) {
    throw std::invalid_argument("voltsToAdc: adcCountLevels must be greater than 0");
  }
  const auto vrange = static_cast<double>(adcVrefPos_ - adcVrefNeg_);
  const double scaled =
      (volts * static_cast<double>(analogGain_) + static_cast<double>(adcVrefDc_)) *
      static_cast<double>(adcCountLevels_) / vrange;
  const int64_t rounded = std::llround(scaled);
  if (rounded < 0 || std::cmp_greater_equal(rounded, adcCountLevels_)) {
    throw std::invalid_argument(
        "voltsToAdc: voltage out of representable range for adcCountLevels");
  }
  return static_cast<uint32_t>(rounded);
}

std::optional<NeuralBandEmgCalibration> NeuralBandEmgCalibration::fromParamsJson(
    const std::string& json,
    std::string label) {
  if (json.empty()) {
    return std::nullopt;
  }
  nlohmann::json parsed;
  try {
    parsed = nlohmann::json::parse(json);
  } catch (const nlohmann::json::exception&) {
    return std::nullopt;
  }
  if (!parsed.is_object()) {
    return std::nullopt;
  }

  if (!parsed.contains("analog_gain") || !parsed.contains("daq_vref_pos") ||
      !parsed.contains("daq_vref_neg")) {
    return std::nullopt;
  }

  const auto getFloatOr = [&](const char* key, float fallback) -> float {
    auto it = parsed.find(key);
    if (it == parsed.end() || !it->is_number()) {
      return fallback;
    }
    const float v = it->get<float>();
    return std::isfinite(v) ? v : fallback;
  };
  // Accept int and float encodings; writers sometimes emit integer-valued
  // fields as floats (e.g. `8191.0`).
  const auto getUintOr = [&](const char* key, uint32_t fallback) -> uint32_t {
    auto it = parsed.find(key);
    if (it == parsed.end() || !it->is_number()) {
      return fallback;
    }
    const double v = it->get<double>();
    if (!std::isfinite(v) || v < 0.0 ||
        v > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
      return fallback;
    }
    return static_cast<uint32_t>(std::llround(v));
  };

  const float analogGain = getFloatOr("analog_gain", 1.0f);
  const float vrefPos = getFloatOr("daq_vref_pos", 0.0f);
  const float vrefNeg = getFloatOr("daq_vref_neg", 0.0f);
  const float vrefDc = getFloatOr("daq_vref_dc", 0.0f);
  const uint32_t rangeMax = getUintOr("daq_range_max", 0);
  const uint32_t droppedLsb = getUintOr("emg_truncation_dropped_lsb", 0);
  const uint32_t streamedBitWidth = getUintOr("emg_truncation_bits_transmitted_per_sample", 0);

  if (analogGain == 0.0f || vrefPos <= vrefNeg) {
    return std::nullopt;
  }
  // daq_range_max + 1 would wrap to 0 at UINT32_MAX.
  if (rangeMax == std::numeric_limits<uint32_t>::max()) {
    return std::nullopt;
  }

  const bool isTruncated = streamedBitWidth != 0;
  if (isTruncated &&
      (streamedBitWidth > 31 || droppedLsb > 31 || droppedLsb + streamedBitWidth > 31)) {
    return std::nullopt;
  }

  // daq_range_max is inclusive (e.g. 65535 for 16 bit) → range_max + 1 levels.
  const uint32_t adcCountLevels = rangeMax + 1u;

  return NeuralBandEmgCalibration{
      std::move(label),
      analogGain,
      adcCountLevels,
      vrefPos,
      vrefNeg,
      vrefDc,
      isTruncated,
      streamedBitWidth,
      droppedLsb,
      getUintOr("emg_adc_chip", 0),
  };
}

} // namespace projectaria::tools::calibration
