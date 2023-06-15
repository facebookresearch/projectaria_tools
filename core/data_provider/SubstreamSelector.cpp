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

#include <data_provider/ErrorHandler.h>
#include <data_provider/SubstreamSelector.h>

namespace projectaria::tools::data_provider {
SubstreamSelector::SubstreamSelector(const std::set<vrs::StreamId>& allStreamIds)
    : allStreamIds_(allStreamIds), selectedStreamIds_(allStreamIds) {
  for (const auto& streamId : allStreamIds_) {
    auto typeId = streamId.getTypeId();
    typeIds_.insert(typeId);
    typeIdToStreamIds_[typeId].insert(streamId);
  }
}

/* info about available streams */
const std::set<vrs::StreamId> SubstreamSelector::getStreamIds() const {
  return allStreamIds_;
}

const std::set<vrs::RecordableTypeId> SubstreamSelector::getTypeIds() const {
  return typeIds_;
}

const std::set<vrs::StreamId> SubstreamSelector::getStreamIds(
    const vrs::RecordableTypeId& typeId) const {
  return typeIdToStreamIds_.at(typeId);
}

/* info about selected streams */
bool SubstreamSelector::isActive(const vrs::StreamId& streamId) const {
  checkAndThrow(allStreamIds_.count(streamId));
  return selectedStreamIds_.count(streamId);
}

const std::set<vrs::StreamId> SubstreamSelector::getActiveStreamIds() const {
  return selectedStreamIds_;
}

/* toggle streams on or off */
bool SubstreamSelector::activateStream(const vrs::StreamId& streamId) {
  checkAndThrow(allStreamIds_.count(streamId));
  selectedStreamIds_.insert(streamId);
  return true;
}

bool SubstreamSelector::deactivateStream(const vrs::StreamId& streamId) {
  checkAndThrow(allStreamIds_.count(streamId));
  selectedStreamIds_.erase(streamId);
  return false;
}

bool SubstreamSelector::toggleStream(const vrs::StreamId& streamId) {
  checkAndThrow(allStreamIds_.count(streamId));
  if (isActive(streamId)) {
    deactivateStream(streamId);
    return false;
  } else {
    activateStream(streamId);
    return true;
  }
}

void SubstreamSelector::activateStream(const vrs::RecordableTypeId& typeId) {
  for (const auto& streamId : typeIdToStreamIds_.at(typeId)) {
    activateStream(streamId);
  }
}

void SubstreamSelector::deactivateStream(const vrs::RecordableTypeId& typeId) {
  for (const auto& streamId : typeIdToStreamIds_.at(typeId)) {
    deactivateStream(streamId);
  }
}

void SubstreamSelector::activateStreamAll() {
  selectedStreamIds_ = allStreamIds_;
}

void SubstreamSelector::deactivateStreamAll() {
  selectedStreamIds_.clear();
}
} // namespace projectaria::tools::data_provider
