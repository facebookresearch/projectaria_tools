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

#include <data_provider/data_layout/EyeGazeMetadata.h>
#include <data_provider/players/EyeGazePlayer.h>

#include <array>

#include <vrs/DataSource.h>
#include <vrs/RecordFileReader.h>
#include <vrs/RecordFileWriter.h>
#include <vrs/RecordFormat.h>
#include <vrs/Recordable.h>
#include <vrs/os/Utils.h>

#include <sophus/se3.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

using namespace projectaria::tools::data_provider;
using projectaria::tools::datalayout::CombinedFieldId;
using projectaria::tools::datalayout::EyeGazeConfigurationLayout;
using projectaria::tools::datalayout::EyeGazeLayout;
using projectaria::tools::datalayout::FieldProvenance;
using projectaria::tools::datalayout::getFieldProvenance;
using projectaria::tools::datalayout::setFieldProvenance;
using projectaria::tools::datalayout::SingleFieldId;

namespace {

struct TempFileGuard {
  std::string path;
  ~TempFileGuard() {
    if (!path.empty()) {
      vrs::os::remove(path);
    }
  }
};

class EyeGazeV2Recordable : public vrs::Recordable {
 public:
  EyeGazeV2Recordable(uint32_t provSingle, uint32_t provCombined, std::string paramsJson)
      : vrs::Recordable(vrs::RecordableTypeId::GazeRecordableClass, "test/eyegaze"),
        provSingle_(provSingle),
        provCombined_(provCombined),
        paramsJson_(std::move(paramsJson)) {
    addRecordFormat(vrs::Record::Type::CONFIGURATION, 1, config_.getContentBlock(), {&config_});
    addRecordFormat(vrs::Record::Type::DATA, 1, data_.getContentBlock(), {&data_});
  }

  const vrs::Record* createConfigurationRecord() override {
    config_.streamId.set(0);
    config_.nominalRateHz.set(90.0);
    config_.userCalibrated.set(false);
    config_.userCalibrationError.set(0.0f);
    config_.algorithmName.stage("test-algo");
    config_.algorithmVersion.stage("1.0");
    config_.userCalibrationParamsJson.stage(paramsJson_);
    config_.fieldProvenanceSingle.set(provSingle_);
    config_.fieldProvenanceCombined.set(provCombined_);
    return createRecord(0.0, vrs::Record::Type::CONFIGURATION, 1, vrs::DataSource(config_));
  }

  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }

  void writeDataRecord() {
    data_.trackerTimestampNs.set(1'000);
    data_.systemTimestampNs.set(2'000);
    createRecord(1.0, vrs::Record::Type::DATA, 1, vrs::DataSource(data_));
  }

 private:
  EyeGazeConfigurationLayout config_;
  EyeGazeLayout data_;
  const uint32_t provSingle_;
  const uint32_t provCombined_;
  const std::string paramsJson_;
};

// v1 config layout: strip the three v2 DataPieces. Used to simulate an older
// recording that the v2 reader must still handle gracefully.
class EyeGazeV1ConfigLayout : public vrs::AutoDataLayout {
 public:
  vrs::DataPieceValue<uint32_t> streamId{"stream_id"};
  vrs::DataPieceString algorithmVersion{"algorithm_version"};
  vrs::DataPieceString algorithmName{"algorithm_name"};
  vrs::DataPieceValue<double> nominalRateHz{"nominal_rate_hz"};
  vrs::DataPieceValue<vrs::Bool> userCalibrated{"user_calibrated"};
  vrs::DataPieceValue<float> userCalibrationError{"user_calibration_error"};
  vrs::AutoDataLayoutEnd endLayout;
};

class EyeGazeV1Recordable : public vrs::Recordable {
 public:
  EyeGazeV1Recordable()
      : vrs::Recordable(vrs::RecordableTypeId::GazeRecordableClass, "test/eyegaze") {
    addRecordFormat(vrs::Record::Type::CONFIGURATION, 1, config_.getContentBlock(), {&config_});
    addRecordFormat(vrs::Record::Type::DATA, 1, data_.getContentBlock(), {&data_});
  }
  const vrs::Record* createConfigurationRecord() override {
    config_.streamId.set(0);
    config_.nominalRateHz.set(90.0);
    config_.userCalibrated.set(false);
    config_.userCalibrationError.set(0.0f);
    return createRecord(0.0, vrs::Record::Type::CONFIGURATION, 1, vrs::DataSource(config_));
  }
  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }
  void writeDataRecord() {
    data_.trackerTimestampNs.set(1'000);
    data_.systemTimestampNs.set(2'000);
    createRecord(1.0, vrs::Record::Type::DATA, 1, vrs::DataSource(data_));
  }

 private:
  EyeGazeV1ConfigLayout config_;
  EyeGazeLayout data_;
};

template <typename RecordableT, typename... Args>
std::string writeVrsWithEyeGazeStream(Args&&... args) {
  // getUniquePath avoids collisions across suites / crashed prior runs
  // (matches VrsDataProviderAudioTest.cpp:122-123).
  auto path = vrs::os::getUniquePath(vrs::os::getTempFolder() + "eye_gaze_player_test");
  vrs::RecordFileWriter writer;
  RecordableT recordable{std::forward<Args>(args)...};
  writer.addRecordable(&recordable);
  recordable.createConfigurationRecord();
  recordable.createStateRecord();
  recordable.writeDataRecord();
  EXPECT_EQ(writer.writeToFile(path), 0);
  return path;
}

EyeGazePlayer readEyeGazeConfig(const std::string& path) {
  vrs::RecordFileReader reader;
  EXPECT_EQ(reader.openFile(path), 0);
  const auto streamIds = reader.getStreams();
  EXPECT_EQ(streamIds.size(), 1u);
  const auto streamId = *streamIds.begin();
  // T_Cpf_Device stub — never used because we only read the CONFIGURATION record.
  EyeGazePlayer player(streamId, Sophus::SE3f{});
  // setStreamPlayer takes a raw pointer to `player`; we close the file
  // before the return so the reader stops issuing callbacks before it
  // goes out of scope. player is returned by value — EyeGazePlayer's
  // defaulted move (see EyeGazePlayer.h) + NRVO handle it.
  reader.setStreamPlayer(streamId, &player);
  for (const auto* r : reader.getIndex(streamId)) {
    if (r->recordType == vrs::Record::Type::CONFIGURATION) {
      EXPECT_EQ(reader.readRecord(*r, &player, /*setupPlayer=*/true), 0);
      break;
    }
  }
  reader.closeFile();
  return player;
}

} // namespace

TEST(EyeGazeMetadata, FieldProvenanceHelperRoundtrip) {
  constexpr std::array<FieldProvenance, 4> kAll = {
      FieldProvenance::SUPPORTED,
      FieldProvenance::CALCULATED,
      FieldProvenance::HARDCODED,
      FieldProvenance::NOT_PRODUCED,
  };
  constexpr std::array<SingleFieldId, 5> kSingles = {
      SingleFieldId::GazeOrigin,
      SingleFieldId::GazeDirection,
      SingleFieldId::EntrancePupilPosition,
      SingleFieldId::PupilDiameter,
      SingleFieldId::Blink,
  };
  for (auto id : kSingles) {
    for (auto p : kAll) {
      const uint32_t packed = setFieldProvenance(0u, id, p);
      EXPECT_EQ(getFieldProvenance(packed, id), p);
      for (auto other : kSingles) {
        if (other == id) {
          continue;
        }
        EXPECT_EQ(getFieldProvenance(packed, other), FieldProvenance::SUPPORTED);
      }
    }
  }
  constexpr std::array<CombinedFieldId, 6> kCombined = {
      CombinedFieldId::GazeOriginCombined,
      CombinedFieldId::GazeDirectionCombined,
      CombinedFieldId::ConvergenceDistance,
      CombinedFieldId::InterocularDistance,
      CombinedFieldId::FoveatedGaze,
      CombinedFieldId::SpatialGazePoint,
  };
  for (auto id : kCombined) {
    for (auto p : kAll) {
      const uint32_t packed = setFieldProvenance(0u, id, p);
      EXPECT_EQ(getFieldProvenance(packed, id), p);
    }
  }
}

TEST(EyeGazeMetadata, WireFormatContract_RawUint32Unpack) {
  // 2 | (0<<2) | (3<<4) | (3<<6) | (3<<8) == 1010: slots 0..4 =
  // HARDCODED, SUPPORTED, NOT_PRODUCED, NOT_PRODUCED, NOT_PRODUCED.
  constexpr uint32_t kMlSinglePackedProvenance = 1010u;
  EXPECT_EQ(
      getFieldProvenance(kMlSinglePackedProvenance, SingleFieldId::GazeOrigin),
      FieldProvenance::HARDCODED);
  EXPECT_EQ(
      getFieldProvenance(kMlSinglePackedProvenance, SingleFieldId::GazeDirection),
      FieldProvenance::SUPPORTED);
  EXPECT_EQ(
      getFieldProvenance(kMlSinglePackedProvenance, SingleFieldId::EntrancePupilPosition),
      FieldProvenance::NOT_PRODUCED);
  EXPECT_EQ(
      getFieldProvenance(kMlSinglePackedProvenance, SingleFieldId::PupilDiameter),
      FieldProvenance::NOT_PRODUCED);
  EXPECT_EQ(
      getFieldProvenance(kMlSinglePackedProvenance, SingleFieldId::Blink),
      FieldProvenance::NOT_PRODUCED);
}

TEST(EyeGazePlayer, ReadV2ConfigStampsAllFields) {
  uint32_t provSingle = 0;
  provSingle =
      setFieldProvenance(provSingle, SingleFieldId::GazeOrigin, FieldProvenance::HARDCODED);
  provSingle =
      setFieldProvenance(provSingle, SingleFieldId::GazeDirection, FieldProvenance::SUPPORTED);
  uint32_t provCombined = 0;
  provCombined = setFieldProvenance(
      provCombined, CombinedFieldId::GazeOriginCombined, FieldProvenance::HARDCODED);
  provCombined = setFieldProvenance(
      provCombined, CombinedFieldId::SpatialGazePoint, FieldProvenance::CALCULATED);

  TempFileGuard g{writeVrsWithEyeGazeStream<EyeGazeV2Recordable>(
      provSingle, provCombined, std::string{R"({"schema":"v1"})"})};
  const auto player = readEyeGazeConfig(g.path);
  const auto& cfg = player.getConfigRecord();

  EXPECT_EQ(cfg.algorithmName, "test-algo");
  EXPECT_EQ(cfg.algorithmVersion, "1.0");
  EXPECT_EQ(cfg.userCalibrationParamsJson, "{\"schema\":\"v1\"}");
  EXPECT_EQ(cfg.fieldProvenanceSingle, provSingle);
  EXPECT_EQ(cfg.fieldProvenanceCombined, provCombined);
  EXPECT_EQ(cfg.getSingleFieldProvenance(SingleFieldId::GazeOrigin), FieldProvenance::HARDCODED);
  EXPECT_EQ(cfg.getSingleFieldProvenance(SingleFieldId::GazeDirection), FieldProvenance::SUPPORTED);
  EXPECT_EQ(
      cfg.getCombinedFieldProvenance(CombinedFieldId::SpatialGazePoint),
      FieldProvenance::CALCULATED);
}

TEST(EyeGazePlayer, ReadV1ConfigBackCompat) {
  TempFileGuard g{writeVrsWithEyeGazeStream<EyeGazeV1Recordable>()};
  const auto player = readEyeGazeConfig(g.path);
  const auto& cfg = player.getConfigRecord();

  EXPECT_EQ(cfg.algorithmName, "");
  EXPECT_EQ(cfg.algorithmVersion, "");
  EXPECT_EQ(cfg.userCalibrationParamsJson, "");
  EXPECT_EQ(cfg.fieldProvenanceSingle, 0u);
  EXPECT_EQ(cfg.fieldProvenanceCombined, 0u);
  EXPECT_EQ(cfg.getSingleFieldProvenance(SingleFieldId::GazeOrigin), FieldProvenance::SUPPORTED);
  EXPECT_EQ(
      cfg.getCombinedFieldProvenance(CombinedFieldId::SpatialGazePoint),
      FieldProvenance::SUPPORTED);
}
