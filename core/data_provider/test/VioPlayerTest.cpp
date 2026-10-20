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

#include <data_provider/data_layout/VioMetadata.h>
#include <data_provider/players/VioHighFrequencyPlayer.h>
#include <data_provider/players/VioPlayer.h>

#include <vrs/DataSource.h>
#include <vrs/RecordFileReader.h>
#include <vrs/RecordFileWriter.h>
#include <vrs/RecordFormat.h>
#include <vrs/Recordable.h>
#include <vrs/os/Utils.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

using namespace projectaria::tools::data_provider;
using projectaria::tools::datalayout::VioHighFrequencyResultDataLayout;
using projectaria::tools::datalayout::VioResultConfigurationLayout;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
const vrs::Point4Dd kNaNRotationXyzw{kNaN, kNaN, kNaN, kNaN};
const vrs::Point4Dd kIdentityRotationXyzw{0., 0., 0., 1.};
const std::string kSessionUuid = "01234567-89ab-cdef-0123-456789abcdef";

struct TempFileGuard {
  std::string path;
  ~TempFileGuard() {
    if (!path.empty()) {
      vrs::os::remove(path);
    }
  }
};

// VioResultDataLayout without online_calib_state, which VioPlayer treats as optional.
class VioTestDataLayout : public vrs::AutoDataLayout {
 public:
  vrs::DataPieceString frontendSessionUuid{"frontend_session_uuid"};
  vrs::DataPieceValue<std::int32_t> frameId{"frame_id"};
  vrs::DataPieceValue<std::int64_t> captureTimestampNs{"capture_timestamp_ns"};
  vrs::DataPieceValue<std::int8_t> status{"status"};
  vrs::DataPieceValue<vrs::Bool> isTrackerHealthy{"is_tracker_healthy"};
  vrs::DataPieceValue<vrs::Bool> isMapTrackingHealthy{"is_map_tracking_healthy"};
  vrs::DataPieceValue<vrs::Point3Dd> gravityVecInOdometry{"gravity_vec_odometry"};
  vrs::DataPieceValue<vrs::Point3Dd> txyz_Odometry_BodyImu{"txyz_odometry_bodyimu"};
  vrs::DataPieceValue<vrs::Point4Dd> qxyzw_Odometry_BodyImu{"qxyzw_odometry_bodyimu"};
  vrs::DataPieceValue<vrs::Point3Dd> txyz_BodyImu_Device{"txyz_bodyimu_device"};
  vrs::DataPieceValue<vrs::Point4Dd> qxyzw_BodyImu_Device{"qxyzw_bodyimu_device"};
  vrs::DataPieceValue<vrs::Point3Dd> linearVelocityInOdometry{"linear_velocity_odometry"};
  vrs::DataPieceValue<vrs::Point3Dd> angularVelocityInBodyImu{"angular_velocity_bodyimu"};
  vrs::AutoDataLayoutEnd endLayout;
};

template <typename DataLayoutT>
class VioRecordable : public vrs::Recordable {
 public:
  VioRecordable() : vrs::Recordable(vrs::RecordableTypeId::PoseRecordableClass, "test/vio") {
    addRecordFormat(vrs::Record::Type::CONFIGURATION, 1, config_.getContentBlock(), {&config_});
    addRecordFormat(vrs::Record::Type::DATA, 1, data_.getContentBlock(), {&data_});
  }

  const vrs::Record* createConfigurationRecord() override {
    config_.streamId.set(0);
    config_.nominalRateHz.set(10.0);
    config_.messageVersion.stage("1.0");
    return createRecord(0.0, vrs::Record::Type::CONFIGURATION, 1, vrs::DataSource(config_));
  }

  const vrs::Record* createStateRecord() override {
    return createRecord(0.0, vrs::Record::Type::STATE, 1);
  }

  DataLayoutT& data() {
    return data_;
  }

  void writeDataRecord(double recordTimeSec) {
    createRecord(recordTimeSec, vrs::Record::Type::DATA, 1, vrs::DataSource(data_));
  }

 private:
  VioResultConfigurationLayout config_;
  DataLayoutT data_;
};

void writeVioRecord(
    VioRecordable<VioTestDataLayout>& recordable,
    int32_t frameId,
    const vrs::Point4Dd& qxyzw_Odometry_BodyImu) {
  auto& data = recordable.data();
  data.frontendSessionUuid.stage(kSessionUuid);
  data.frameId.set(frameId);
  data.captureTimestampNs.set(frameId * 1'000);
  data.status.set(static_cast<int8_t>(VioStatus::VALID));
  data.isTrackerHealthy.set(true);
  data.isMapTrackingHealthy.set(true);
  data.gravityVecInOdometry.set({0., 0., -9.81});
  data.txyz_Odometry_BodyImu.set({1., 2., 3.});
  data.qxyzw_Odometry_BodyImu.set(qxyzw_Odometry_BodyImu);
  data.txyz_BodyImu_Device.set({0., 0., 0.});
  data.qxyzw_BodyImu_Device.set(kIdentityRotationXyzw);
  data.linearVelocityInOdometry.set({0., 0., 0.});
  data.angularVelocityInBodyImu.set({0., 0., 0.});
  recordable.writeDataRecord(frameId);
}

void writeVioHighFrequencyRecord(
    VioRecordable<VioHighFrequencyResultDataLayout>& recordable,
    int32_t index,
    const vrs::Point4Dd& qxyzw_Odometry_Device) {
  auto& data = recordable.data();
  data.frontendSessionUuid.stage(kSessionUuid);
  data.captureTimestampNs.set(index * 1'000);
  data.gravityVec_Odometry.set({0., 0., -9.81});
  data.txyz_Odometry_Device.set({1., 2., 3.});
  data.qxyzw_Odometry_Device.set(qxyzw_Odometry_Device);
  data.linearVelocity_Odometry.set({0., 0., 0.});
  data.angularVelocity_Device.set({0., 0., 0.});
  data.qualityScore.set(1.f);
  recordable.writeDataRecord(index);
}

// Writes one configuration record followed by one data record per entry of `rotations`.
template <typename DataLayoutT, typename WriteRecordT>
std::string writeVioVrs(const std::vector<vrs::Point4Dd>& rotations, WriteRecordT writeRecord) {
  auto path = vrs::os::getUniquePath(vrs::os::getTempFolder() + "vio_player_test");
  vrs::RecordFileWriter writer;
  VioRecordable<DataLayoutT> recordable;
  writer.addRecordable(&recordable);
  recordable.createConfigurationRecord();
  recordable.createStateRecord();
  for (size_t i = 0; i < rotations.size(); ++i) {
    writeRecord(recordable, static_cast<int32_t>(i + 1), rotations[i]);
  }
  EXPECT_EQ(writer.writeToFile(path), 0);
  return path;
}

// Reads every record of the file's only stream in order, calling `afterDataRecord` once each
// data record has been read, the way the by-index API reads a record and then the player cache.
template <typename PlayerT, typename AfterDataRecordT>
void readVioVrs(const std::string& path, PlayerT& player, AfterDataRecordT afterDataRecord) {
  vrs::RecordFileReader reader;
  ASSERT_EQ(reader.openFile(path), 0);
  const auto streamIds = reader.getStreams();
  ASSERT_EQ(streamIds.size(), 1u);
  const auto streamId = *streamIds.begin();
  reader.setStreamPlayer(streamId, &player);
  for (const auto* r : reader.getIndex(streamId)) {
    ASSERT_EQ(reader.readRecord(*r, &player, /*setupPlayer=*/true), 0);
    if (r->recordType == vrs::Record::Type::DATA) {
      afterDataRecord();
    }
  }
  reader.closeFile();
}

} // namespace

TEST(VioPlayer, SkipsARecordWithANaNPoseWithoutTouchingTheCachedRecord) {
  TempFileGuard g{writeVioVrs<VioTestDataLayout>(
      {kIdentityRotationXyzw, kNaNRotationXyzw, kIdentityRotationXyzw}, writeVioRecord)};

  VioPlayer player(vrs::StreamId{});
  std::vector<int64_t> deliveredTimestampsNs;
  player.setCallback([&](const FrontendOutput& data) {
    deliveredTimestampsNs.push_back(data.captureTimestampNs);
    return true;
  });
  std::vector<int64_t> cachedTimestampsNs;
  std::vector<FrameSetId> cachedFrameIds;
  readVioVrs(g.path, player, [&] {
    cachedTimestampsNs.push_back(player.getDataRecord().captureTimestampNs);
    cachedFrameIds.push_back(player.getDataRecord().frameID);
  });

  EXPECT_EQ(deliveredTimestampsNs, (std::vector<int64_t>{1'000, 3'000}));
  EXPECT_EQ(cachedTimestampsNs, (std::vector<int64_t>{1'000, 1'000, 3'000}));
  EXPECT_EQ(cachedFrameIds, (std::vector<FrameSetId>{1, 1, 3}));
}

TEST(VioHighFrequencyPlayer, SkipsARecordWithANaNPoseWithoutTouchingTheCachedRecord) {
  TempFileGuard g{writeVioVrs<VioHighFrequencyResultDataLayout>(
      {kIdentityRotationXyzw, kNaNRotationXyzw, kIdentityRotationXyzw},
      writeVioHighFrequencyRecord)};

  VioHighFrequencyPlayer player(vrs::StreamId{});
  std::vector<int64_t> deliveredTimestampsUs;
  player.setHighFrequencyCallback([&](const projectaria::tools::mps::OpenLoopTrajectoryPose& data,
                                      const VioHighFreqConfiguration&) {
    deliveredTimestampsUs.push_back(data.trackingTimestamp.count());
    return true;
  });
  std::vector<int64_t> cachedTimestampsUs;
  readVioVrs(g.path, player, [&] {
    cachedTimestampsUs.push_back(player.getDataRecord().trackingTimestamp.count());
  });

  EXPECT_EQ(deliveredTimestampsUs, (std::vector<int64_t>{1, 3}));
  EXPECT_EQ(cachedTimestampsUs, (std::vector<int64_t>{1, 1, 3}));
}
