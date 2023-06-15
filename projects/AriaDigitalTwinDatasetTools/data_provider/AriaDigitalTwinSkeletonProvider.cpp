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

#include "AriaDigitalTwinSkeletonProvider.h"

#include <fstream>

#include <cereal/external/rapidjson/document.h>
#include <cereal/external/rapidjson/rapidjson.h>

#include "AriaDigitalTwinUtils.h"

#define DEFAULT_LOG_CHANNEL "AriaDigitalTwinSkeletonProvider"
#include <logging/Log.h>

const std::string kFramesKey{"frames"};
const std::string kMarkersKey{"markers"};
const std::string kJointsKey{"joints"};
const std::string kTimestampKey{"timestamp_ns"};

namespace projectaria::dataset::adt {

AriaDigitalTwinSkeletonProvider::AriaDigitalTwinSkeletonProvider(
    const std::string& skeletonJsonPath) {
  std::filesystem::path jsonPath(skeletonJsonPath);
  if (!std::filesystem::exists(jsonPath)) {
    throw std::runtime_error{
        fmt::format("Could not open skeleton joints json file{} \n", skeletonJsonPath)};
  }
  readSkeletonJson(jsonPath);
}

void AriaDigitalTwinSkeletonProvider::readSkeletonJson(const std::filesystem::path& path) {
  frames_.clear();
  std::ifstream fileStream(path);
  if (!fileStream.is_open()) {
    throw std::runtime_error(fmt::format("Could not open skeleton file {} \n", path.string()));
  }

  std::stringstream buffer;
  buffer << fileStream.rdbuf();

  fb_rapidjson::Document jdoc;
  jdoc.Parse(buffer.str().c_str());

  if (!jdoc.HasMember(kFramesKey.c_str())) {
    XR_LOGE("key: '{}' not available in skeleton json file: {}", kFramesKey, path.string());
    throw std::runtime_error{"invalid json format"};
  }
  for (const fb_rapidjson::Value& frameJson : jdoc[kFramesKey.c_str()].GetArray()) {
    if (!frameJson.HasMember(kMarkersKey.c_str())) {
      XR_LOGE("key: '{}' not available in skeleton json file: {}", kMarkersKey, path.string());
      throw std::runtime_error{"invalid json format"};
    }
    if (!frameJson.HasMember(kJointsKey.c_str())) {
      XR_LOGE("key: '{}' not available in skeleton json file: {}", kJointsKey, path.string());
      throw std::runtime_error{"invalid json format"};
    }
    SkeletonFrame frame;
    for (const auto& point : frameJson[kMarkersKey.c_str()].GetArray()) {
      if (point.Size() != 3) {
        XR_LOGE("point has an invalid size");
        throw std::runtime_error{"invalid json format"};
      }
      Eigen::Vector3d p{point[0].GetDouble(), point[1].GetDouble(), point[2].GetDouble()};
      frame.markers.push_back(p);
    }
    for (const auto& point : frameJson[kJointsKey.c_str()].GetArray()) {
      if (point.Size() != 3) {
        XR_LOGE("point has an invalid size");
        throw std::runtime_error{"invalid json format"};
      }
      Eigen::Vector3d p{point[0].GetDouble(), point[1].GetDouble(), point[2].GetDouble()};
      frame.joints.push_back(p);
    }

    if (!frameJson.HasMember(kTimestampKey.c_str())) {
      XR_LOGE("key: '{}' not available in skeleton json file: {}", kTimestampKey, path.string());
      throw std::runtime_error{"invalid json format"};
    }
    int64_t timestampNs = frameJson[kTimestampKey.c_str()].GetInt64();
    frames_.emplace(timestampNs, frame);
  }
}

SkeletonFrameWithDt AriaDigitalTwinSkeletonProvider::getSkeletonByTimestampNs(
    int64_t deviceTimeStampNs,
    const TimeQueryOptions& timeQueryOptions) const {
  if (frames_.empty()) {
    XR_LOGW_EVERY_N_SEC(100, "no skeleton data available");
    return {};
  }

  auto iter = queryTimestampsMap<SkeletonFrame>(frames_, deviceTimeStampNs, timeQueryOptions);
  if (iter == frames_.end()) {
    fmt::print(
        "invalid query time for skeleton joints data. Query {}Ns, data range: [{}, {}]Ns\n",
        deviceTimeStampNs,
        frames_.begin()->first,
        frames_.rbegin()->first);
    return {};
  }
  return SkeletonFrameWithDt(iter->second, iter->first - deviceTimeStampNs);
}

const std::vector<std::pair<int, int>>& AriaDigitalTwinSkeletonProvider::getJointConnections() {
  static const std::vector<std::pair<int, int>> kJointConnections{
      {4, 3},   {3, 2},  {2, 1},   {1, 0},   {0, 43},  {43, 44}, {44, 45},
      {45, 46}, {0, 47}, {47, 48}, {48, 49}, {49, 50}, {2, 5},   {5, 6},
      {6, 7},   {7, 8},  {2, 24},  {24, 25}, {25, 26}, {26, 27}};
  return kJointConnections;
}

const std::vector<std::string>& AriaDigitalTwinSkeletonProvider::getJointLabels() {
  static const std::vector<std::string> kJointLabels{
      "Skeleton", "Ab",       "Chest",    "Neck",      "Head",    "LShoulder", "LUArm",
      "LFArm",    "LHand",    "LThumb1",  "LThumb2",   "LThumb3", "LIndex1",   "LIndex2",
      "LIndex3",  "LMiddle1", "LMiddle2", "LMiddle3",  "LRing1",  "LRing2",    "LRing3",
      "LPinky1",  "LPinky2",  "LPinky3",  "RShoulder", "RUArm",   "RFArm",     "RHand",
      "RThumb1",  "RThumb2",  "RThumb3",  "RIndex1",   "RIndex2", "RIndex3",   "RMiddle1",
      "RMiddle2", "RMiddle3", "RRing1",   "RRing2",    "RRing3",  "RPinky1",   "RPinky2",
      "RPinky3",  "LThigh",   "LShin",    "LFoot",     "LToe",    "RThigh",    "RShin",
      "RFoot",    "RToe"};
  return kJointLabels;
}

const std::vector<std::string>& AriaDigitalTwinSkeletonProvider::getMarkerLabels() {
  static const std::vector<std::string> kMarkerLabels{
      "LIAS", "LFTC", "LIPS", "RIPS", "RFTC", "RIAS", "CV7",  "TV2",  "TV7",  "SJN",
      "SXS",  "LAH",  "LPH",  "RPH",  "RAH",  "LHGT", "LCAJ", "LUA",  "LHLE", "LHME",
      "LUSP", "LRSP", "LHM2", "RHGT", "RCAJ", "RUA",  "RHLE", "RHME", "RUSP", "RRSP",
      "RHM2", "LTH",  "LFLE", "LFME", "LFAX", "LTTC", "LSK",  "LFAL", "LTAM", "LFCC",
      "LFM1", "LFM5", "LFM2", "LDP1", "RTH",  "RFLE", "RFME", "RFAX", "RTTC", "RSK",
      "RFAL", "RTAM", "RFCC", "RFM1", "RFM5", "RFM2", "RDP1"};
  return kMarkerLabels;
}

} // namespace projectaria::dataset::adt
