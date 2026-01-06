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
#include <set>

#include <vrs/StreamId.h>

namespace projectaria::tools::data_provider {
/**
 * @brief Class for subselecting VRS streams from all streams available in an VRS file
 */
class SubstreamSelector {
 public:
  explicit SubstreamSelector(const std::set<vrs::StreamId>& allStreamIds);

  /* info about available streams */
  /**
   * @brief Returns the list of available stream ids
   */
  [[nodiscard]] std::set<vrs::StreamId> getStreamIds() const;
  /**
   * @brief Returns the list of available type ids
   */

  [[nodiscard]] std::set<vrs::RecordableTypeId> getTypeIds() const;
  /**
   * @brief Returns the list of stream ids of a specified type
   * @param typeId the ID of a VRS recordable type e.g. vrs::RecordableTypeId::SlamCameraData
   */

  [[nodiscard]] std::set<vrs::StreamId> getStreamIds(const vrs::RecordableTypeId& typeId) const;

  /* info about selected streams */

  /**
   * @brief Returns of a stream has been selected
   */

  [[nodiscard]] bool isActive(const vrs::StreamId& streamId) const;
  /**
   * @brief Returns all selected streams
   */

  [[nodiscard]] std::set<vrs::StreamId> getActiveStreamIds() const;

  /* toggle streams on or off */
  /**
   * @brief Turns on a VRS stream
   * @param streamId the ID of the VRS stream of interest
   * @ret Returns true if stream is turned on, false if stream is turned off
   */

  bool activateStream(const vrs::StreamId& streamId);
  /**
   * @brief Turns off a VRS stream
   * @param streamId the ID of the VRS stream of interest
   * @ret Returns true if stream is turned on, false if stream is turned off
   */

  bool deactivateStream(const vrs::StreamId& streamId);
  /**
   * @brief Toggles a VRS stream from on to off or from off to on
   * @param streamId the ID of the VRS stream of interest
   * @ret Returns true if stream is turned on, false if stream is turned off
   */

  bool toggleStream(const vrs::StreamId& streamId);

  /**
   * @brief Turns on all streams of a specific typeId, regardless of current state
   * @param typeId the ID of a VRS recordable type e.g. vrs::RecordableTypeId::SlamCameraData
   */

  void activateStream(const vrs::RecordableTypeId& typeId);
  /**
   * @brief Turns on all streams of a specific typeId, regardless of current state
   * @param typeId the ID of a VRS recordable type e.g. vrs::RecordableTypeId::SlamCameraData
   */

  void deactivateStream(const vrs::RecordableTypeId& typeId);

  /**
   * @debrif Turns on all streams available, regardless of current state
   */

  void activateStreamAll();
  /**
   * @debrif Turns off all streams available, regardless of current state
   */
  void deactivateStreamAll();

 private:
  std::set<vrs::StreamId> allStreamIds_;
  std::set<vrs::StreamId> selectedStreamIds_;

  std::set<vrs::RecordableTypeId> typeIds_;
  std::map<vrs::RecordableTypeId, std::set<vrs::StreamId>> typeIdToStreamIds_;
};

} // namespace projectaria::tools::data_provider
