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

#include <array>
#include <string>

namespace projectaria::tools::data_provider {

/**
 * @brief Enum class for different types of timestamps used in projectaria_tools
 */
enum class TimeDomain {
  RecordTime, /**< timestamp directly stored in vrs index, fast to access, but not guaranteed which
                 time domain*/
  DeviceTime, /**< capture time in device's timedomain, <b>accurate</b>. All sensors on the same
                 Aria glass share the same device time domain as they are issued from the same
                 clock. We <b>strongly recommend</b> to always work with the device timestamp when
                 dealing with <b>single-device</b> Aria data. */
  HostTime, /**< arrival time in host computer's timedomain, may not be accurate */
  TimeCode, /**< capture in TimeSync server's timedomain, accurate across devices in a
               <b>multi-device</b> data capture. */
};
constexpr size_t kNumTimeDomain = 4;
/**
 * @brief A helper function to return a descriptive name for a given TimeDomain enum
 * @return one of the followings: {"RecordTime", "DeviceTime", "HostTime", "Timecode"}
 */
inline std::string getName(const TimeDomain& domain) {
  std::array<std::string, kNumTimeDomain> domainNames{
      "RecordTime", "DeviceTime", "HostTime", "TimeCode"};
  return domainNames.at(static_cast<size_t>(domain));
}

/**
 * @brief Enum class that allows user to customize how to query data by timestamp `t_query`
 */
enum class TimeQueryOptions {
  Before, /**< the last valid data with `timestamp <= t_query` */
  After, /**< the first valid data with `timestamp >= t_query` */
  Closest /**< the data whose `|timestamp - t_query|` is smallest:
           * @parblock
           *   1. If two data of equal distance, we return the data before the queried timestamp.
           *   2. Although query from timecode domain is possible, we return the data closest in
           * the device domain. This is to maintain consistency when switching time domains.
           * @endparblock
           */
};
} // namespace projectaria::tools::data_provider
