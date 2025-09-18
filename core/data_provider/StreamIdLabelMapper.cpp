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

#include <data_provider/StreamIdLabelMapper.h>

#include <stdexcept>

#include <logging/Checks.h>
#define DEFAULT_LOG_CHANNEL "StreamIdLabelMapper"
#include <logging/Log.h>

namespace projectaria::tools::data_provider {

StreamIdLabelMapper::StreamIdLabelMapper(
    const std::map<vrs::StreamId, std::string>& streamIdToLabel)
    : streamIdToLabel_(streamIdToLabel) {
  for (const auto& [streamId, label] : streamIdToLabel) {
    labelToStreamId_.emplace(label, streamId);
  }
}

std::optional<std::string> StreamIdLabelMapper::getLabelFromStreamId(
    const vrs::StreamId& streamId) const {
  auto maybeLabel = streamIdToLabel_.find(streamId);
  if (maybeLabel != streamIdToLabel_.end()) {
    return maybeLabel->second;
  } else {
    XR_LOGW(
        "stream id {} not found in Aria Device Model. You will not be able to get the label of this stream. ",
        streamId.getNumericName());
    return {};
  }
}

std::optional<vrs::StreamId> StreamIdLabelMapper::getStreamIdFromLabel(
    const std::string& label) const {
  auto maybeStreamId = labelToStreamId_.find(label);
  if (maybeStreamId != labelToStreamId_.end()) {
    return maybeStreamId->second;
  } else {
    XR_LOGW(
        "sensor label {} not found in Aria Device Model. You will not be able to get the stream id for this label. ",
        label);
    return {};
  }
}

namespace {

// Helper function to get a static streamId -> label mapping for Gen1
std::shared_ptr<StreamIdLabelMapper> getAriaGen1StreamIdLabelMapper() {
  const std::map<vrs::StreamId, std::string> kGen1StreamIdToLabel = {
      // streams below have calibration
      {vrs::StreamId::fromNumericName("1201-1"), "camera-slam-left"},
      {vrs::StreamId::fromNumericName("1201-2"), "camera-slam-right"},
      {vrs::StreamId::fromNumericName("1202-1"), "imu-right"},
      {vrs::StreamId::fromNumericName("1202-2"), "imu-left"},
      {vrs::StreamId::fromNumericName("214-1"), "camera-rgb"},
      {vrs::StreamId::fromNumericName("211-1"), "camera-et"},
      {vrs::StreamId::fromNumericName("1203-1"), "mag0"},
      {vrs::StreamId::fromNumericName("247-1"), "baro0"},
      {vrs::StreamId::fromNumericName("231-1"), "mic"},
      // streams below does not have calibration, but we still assign a label
      {vrs::StreamId::fromNumericName("281-1"), "gps"},
      {vrs::StreamId::fromNumericName("281-2"), "gps-app"},
      {vrs::StreamId::fromNumericName("282-1"), "wps"},
      {vrs::StreamId::fromNumericName("283-1"), "bluetooth"}};
  return std::make_shared<StreamIdLabelMapper>(kGen1StreamIdToLabel);
}

// Helper function to get stream Id -> label mapping for Gen2, which is dynamically read from VRS
std::shared_ptr<StreamIdLabelMapper> getAriaGen2StreamIdLabelMapper(
    std::shared_ptr<vrs::MultiRecordFileReader> reader) {
  // The following fields are static
  std::map<vrs::StreamId, std::string> streamIdToLabel = {
      // streams below have calibration
      {vrs::StreamId::fromNumericName("1201-1"), "slam-front-left"},
      {vrs::StreamId::fromNumericName("1201-2"), "slam-front-right"},
      {vrs::StreamId::fromNumericName("1201-3"), "slam-side-left"},
      {vrs::StreamId::fromNumericName("1201-4"), "slam-side-right"},
      {vrs::StreamId::fromNumericName("1202-1"), "imu-left"},
      {vrs::StreamId::fromNumericName("1202-2"), "imu-right"},
      {vrs::StreamId::fromNumericName("214-1"), "camera-rgb"},
      {vrs::StreamId::fromNumericName("211-1"), "camera-et-left"},
      {vrs::StreamId::fromNumericName("211-2"), "camera-et-right"},
      {vrs::StreamId::fromNumericName("1203-1"), "mag0"},
      {vrs::StreamId::fromNumericName("247-1"), "baro0"},
      {vrs::StreamId::fromNumericName("246-1"), "temperature"},
      {vrs::StreamId::fromNumericName("231-1"), "mic"},
      // streams below does not have calibration, but we still assign a label
      {vrs::StreamId::fromNumericName("281-1"), "gps"},
      {vrs::StreamId::fromNumericName("281-2"), "gps-app"},
      {vrs::StreamId::fromNumericName("282-1"), "wps"},
      {vrs::StreamId::fromNumericName("283-1"), "bluetooth"},
      {vrs::StreamId::fromNumericName("248-1"), "ppg"},
      {vrs::StreamId::fromNumericName("500-1"), "als"},
      // MPV Streams
      {vrs::StreamId::fromNumericName("373-1"), "eyegaze"},
  };

  // Hand pose and VIO streams can be dynamic
  const std::vector<vrs::StreamId> maybeHandPoseStreamIds =
      reader->getStreams(vrs::RecordableTypeId::PoseRecordableClass, "device/oatmeal/hand");
  if (maybeHandPoseStreamIds.size() == 1) {
    streamIdToLabel.emplace(maybeHandPoseStreamIds[0], "handtracking");
  } else if (maybeHandPoseStreamIds.size() > 1) {
    throw std::runtime_error("More than one hand pose stream found in the VRS file.");
  }

  const std::vector<vrs::StreamId> maybeVioStreamIds =
      reader->getStreams(vrs::RecordableTypeId::PoseRecordableClass, "device/oatmeal/vio");
  if (maybeVioStreamIds.size() == 1) {
    streamIdToLabel.emplace(maybeVioStreamIds[0], "vio");
  } else if (maybeVioStreamIds.size() > 1) {
    throw std::runtime_error("More than one vio stream found in the VRS file.");
  }

  const std::vector<vrs::StreamId> maybeVioHighFreqStreamIds = reader->getStreams(
      vrs::RecordableTypeId::PoseRecordableClass, "device/oatmeal/vio_high_frequency");
  if (maybeVioHighFreqStreamIds.size() == 1) {
    streamIdToLabel.emplace(maybeVioHighFreqStreamIds[0], "vio_high_frequency");
  } else if (maybeVioHighFreqStreamIds.size() > 1) {
    throw std::runtime_error("More than one vio high frequency stream found in the VRS file.");
  }
  return std::make_shared<StreamIdLabelMapper>(streamIdToLabel);
}
} // namespace

std::shared_ptr<StreamIdLabelMapper> getAriaStreamIdLabelMapper(
    const calibration::DeviceVersion& deviceVersion,
    std::shared_ptr<vrs::MultiRecordFileReader> reader) {
  if (deviceVersion == calibration::DeviceVersion::Gen1) {
    return getAriaGen1StreamIdLabelMapper();
  } else if (deviceVersion == calibration::DeviceVersion::Gen2) {
    if (!reader) {
      throw std::runtime_error("VRS reader needs to be provided for Gen2 StreamIdLabelMapper");
    }

    return getAriaGen2StreamIdLabelMapper(reader);
  } else {
    fmt::print(
        "WARNING: Unsupported device version for label to stream id mapping: {}",
        getName(deviceVersion));
    return nullptr;
  }
}
} // namespace projectaria::tools::data_provider
