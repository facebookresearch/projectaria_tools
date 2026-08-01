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
#include <optional>
#include <string>
#include <vector>

namespace projectaria::tools::calibration {

/**
 * @brief Wristband EMG calibration; converts raw ADC counts to voltage at the
 * electrode input. `adcToVolts()` selects the truncated or non-truncated
 * formula automatically based on `isTruncated`. `adcVrefDc` is subtracted in
 * the non-truncated formula only.
 */
class NeuralBandEmgCalibration {
 public:
  NeuralBandEmgCalibration() = default;
  explicit NeuralBandEmgCalibration(
      std::string label,
      float analogGain,
      uint32_t adcCountLevels,
      float adcVrefPos,
      float adcVrefNeg,
      float adcVrefDc,
      bool isTruncated,
      uint32_t streamedBitWidth,
      uint32_t droppedLsb,
      uint32_t adcChipCode);

  [[nodiscard]] const std::string& getLabel() const;
  [[nodiscard]] float getAnalogGain() const;
  [[nodiscard]] uint32_t getAdcCountLevels() const;
  [[nodiscard]] float getAdcVrefPos() const;
  [[nodiscard]] float getAdcVrefNeg() const;
  [[nodiscard]] float getAdcVrefDc() const;
  [[nodiscard]] bool isTruncated() const;
  [[nodiscard]] uint32_t getStreamedBitWidth() const;
  [[nodiscard]] uint32_t getDroppedLsb() const;
  /// Opaque ADC front-end identifier; 0 = unknown, other codes are
  /// hardware-specific and intentionally not decoded here.
  [[nodiscard]] uint32_t getAdcChipCode() const;

  /// Throws `std::invalid_argument` on invalid calibration parameters or
  /// on an `adcCount` outside the streamed bit-width range.
  [[nodiscard]] double adcToVolts(uint32_t adcCount) const;

  [[nodiscard]] std::vector<double> adcToVolts(const std::vector<uint16_t>& adcCounts) const;

  /// Round-to-nearest inverse of `adcToVolts`. Round-trip is exact only for
  /// voltages that lie on the ADC grid; out-of-range inputs throw.
  [[nodiscard]] uint32_t voltsToAdc(double volts) const;

  /// Parse the verbatim `device.emg_calibration` JSON blob from the CONFIG
  /// record. Returns nullopt on empty / malformed / non-object input, on
  /// missing required fields (`analog_gain`, `daq_vref_pos`, `daq_vref_neg`),
  /// or when any parsed value would make `adcToVolts` throw.
  static std::optional<NeuralBandEmgCalibration> fromParamsJson(
      const std::string& json,
      std::string label = "emg");

 private:
  std::string label_;
  float analogGain_ = 1.0f;
  uint32_t adcCountLevels_ = 0;
  float adcVrefPos_ = 0.0f;
  float adcVrefNeg_ = 0.0f;
  float adcVrefDc_ = 0.0f;
  bool isTruncated_ = false;
  uint32_t streamedBitWidth_ = 0;
  uint32_t droppedLsb_ = 0;
  uint32_t adcChipCode_ = 0;
};

} // namespace projectaria::tools::calibration
