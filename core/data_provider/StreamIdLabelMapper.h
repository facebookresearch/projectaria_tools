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

#include <map>
#include <memory>
#include <optional>

#include <vrs/StreamId.h>

namespace projectaria::tools::data_provider {
class StreamIdLabelMapper {
 public:
  StreamIdLabelMapper() = default;
  explicit StreamIdLabelMapper(const std::map<vrs::StreamId, std::string>& streamIdToLabel);

  /**
   * @brief retrieve the label of a vrs stream from its id.
   * Returns nullopt of the stream id does not exist
   */
  std::optional<std::string> getLabelFromStreamId(const vrs::StreamId& streamId) const;
  /**
   * @brief retrieve the ID of a vrs stream from its label.
   * Returns nullopt of the label does not exist
   */
  std::optional<vrs::StreamId> getStreamIdFromLabel(const std::string& label) const;

 private:
  std::map<std::string, vrs::StreamId> labelToStreamId_;
  std::map<vrs::StreamId, std::string> streamIdToLabel_;
};

/**
 * @brief Add default mapping for Aria from streamId to label and update interface to string only
 */
std::shared_ptr<StreamIdLabelMapper> getAriaStreamIdLabelMapper();

} // namespace projectaria::tools::data_provider
