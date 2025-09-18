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

#include <optional>
#include <string>

#include <sophus/se3.hpp>

#include <data_provider/data_types/FrontendTypes.h>
#include "nlohmann/json.hpp"

namespace projectaria::tools::json {

// Json reader and writer for OnlineCalibState in FrontendOutputBase
std::optional<data_provider::OnlineCalibState> onlineCalibStateFromJson(
    const nlohmann::json& jsonObj);
std::optional<data_provider::OnlineCalibState> onlineCalibStateFromJsonStr(
    const std::string& onlineCalibJsonStr);

nlohmann::json onlineCalibStateToJson(const data_provider::OnlineCalibState& onlineCalibState);
std::string onlineCalibStateToJsonStr(const data_provider::OnlineCalibState& onlineCalibState);

} // namespace projectaria::tools::json
