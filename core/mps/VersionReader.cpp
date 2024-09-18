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

#include <fstream>
#include <iostream>

#include <nlohmann/json.hpp>

#include "VersionReader.h"

namespace projectaria::tools::mps {

std::optional<std::string> readVersion(const std::string& summaryFilepath) {
  // Open the file
  std::ifstream file(summaryFilepath);
  if (!file.is_open()) {
    std::cerr << "[readVersion] Can't open the provided file path." << std::endl;
    return {};
  }

  // Parse the JSON
  nlohmann::json summaryJson;
  try {
    file >> summaryJson;
    if (summaryJson.contains("version")) {
      return summaryJson["version"].get<std::string>();
    } else {
      std::cerr << "[readVersion] No version found in the summary file." << std::endl;
    }
  } catch (const nlohmann::json::parse_error& ex) {
    std::cerr << "[readVersion] Error parsing json: " << ex.what() << std::endl;
  }

  return {};
}

} // namespace projectaria::tools::mps
