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
    XR_LOGE(
        "stream id {} not found in Aria Device Model. Double check stream id. ",
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
    XR_LOGE("sensor label {} not found in Aria Device Model. Double check label. ", label);
    return {};
  }
}

std::shared_ptr<StreamIdLabelMapper> getAriaStreamIdLabelMapper() {
  const std::map<vrs::StreamId, std::string> kStreamIdToLabel = {
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
      {vrs::StreamId::fromNumericName("282-1"), "wps"},
      {vrs::StreamId::fromNumericName("283-1"), "bluetooth"}};
  return std::make_shared<StreamIdLabelMapper>(kStreamIdToLabel);
}
} // namespace projectaria::tools::data_provider
