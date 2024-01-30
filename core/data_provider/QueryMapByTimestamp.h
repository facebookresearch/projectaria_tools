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

namespace projectaria::tools::data_provider {

/**
 * @brief helper function for querying an ordered map ordered by timestamps
 * @param tsMap some map ordered by int64_t timestamp with values of type T
 * @param ts query time where ts must have the same time units as tsMap
 * @param timeQueryOptions
 * (1) Closest: returns the closest timestamp. This will always return a value even if querying
 * before or after data range
 * (2) Before: returns the timestamp right before (or equal to) ts. If ts < start, then we return
 * invalid result
 * (3) After: returns the timestamp right after (or equal to) ts. If ts > end, we
 * return  an invalid result
 */
template <typename T>
typename std::map<int64_t, T>::const_iterator queryMapByTimestamp(
    const std::map<int64_t, T>& tsMap,
    int64_t ts,
    const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Closest) {
  // first check if exact match exists
  auto itExact = tsMap.find(ts);
  if (itExact != tsMap.end()) {
    return itExact;
  }

  // check if outside bounds
  if (ts < tsMap.begin()->first) {
    return timeQueryOptions == TimeQueryOptions::Before ? tsMap.end() : tsMap.begin();
  } else if (ts > tsMap.rbegin()->first) {
    return timeQueryOptions == TimeQueryOptions::After ? tsMap.end() : std::prev(tsMap.end());
  }

  // else get before and after
  auto iterAfter = tsMap.lower_bound(ts);
  auto iterBefore = std::prev(iterAfter);

  switch (timeQueryOptions) {
    case TimeQueryOptions::Closest:
      if (std::abs(iterBefore->first - ts) < std::abs(iterAfter->first - ts)) {
        return iterBefore;
      } else {
        return iterAfter;
      }
      break;
    case TimeQueryOptions::After:
      return iterAfter;
      break;
    case TimeQueryOptions::Before:
      return iterBefore;
      break;
    default:
      throw std::runtime_error{"invalid timeQueryOptions"};
      return tsMap.end();
  }
}

} // namespace projectaria::tools::data_provider
