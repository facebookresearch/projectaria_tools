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

#include "PangolinColor.h"

#include <fmt/format.h>
#include <map>
#include <string>

namespace projectaria::tools::viz {

const std::map<std::string, Eigen::Vector3f> kColorMap = {
    {"red", Eigen::Vector3f(1.0f, 0.0f, 0.0f)},
    {"green", Eigen::Vector3f(0.0f, 1.0f, 0.0f)},
    {"blue", Eigen::Vector3f(0.0f, 0.0f, 1.0f)},
    {"yellow", Eigen::Vector3f(1.0f, 1.0f, 0.0f)},
    {"magenta", Eigen::Vector3f(1.0f, 0.0f, 1.0f)},
    {"cyan", Eigen::Vector3f(0.0f, 1.0f, 1.0f)},
    {"white", Eigen::Vector3f(1.0f, 1.0f, 1.0f)},
    {"black", Eigen::Vector3f(0.0f, 0.0f, 0.0f)},
    {"light_gray", Eigen::Vector3f(0.75f, 0.75f, 0.75f)},
    {"dark_gray", Eigen::Vector3f(0.1f, 0.1f, 0.1f)},
    {"light_sky_blue", Eigen::Vector3f(0.67f, 0.85f, 1.0f)},
    {"medium_slate_blue", Eigen::Vector3f(0.3f, 0.3f, 0.9f)},
    {"pale_green", Eigen::Vector3f(0.56f, 0.93f, 0.56f)},
    {"orchid", Eigen::Vector3f(1.0f, 0.4f, 1.0f)},
    {"orange_red", Eigen::Vector3f(1.0f, 0.25f, 0.0f)},
    {"beige", Eigen::Vector3f(0.8f, 0.8f, 0.6f)},
    {"forest_green", Eigen::Vector3f(0.1f, 0.6f, 0.1f)},
    {"plum", Eigen::Vector3f(0.8f, 0.5f, 0.8f)},
    {"lemon_yellow", Eigen::Vector3f(0.94f, 0.95f, 0.52f)}};

void setPlotColor(const std::string& colorName, float alpha) {
  // Try to find the color, and set its RGBA value
  auto it = kColorMap.find(colorName);
  if (it != kColorMap.end()) {
    const Eigen::Vector3f& color = it->second;
    glColor4f(color[0], color[1], color[2], alpha);
  } else {
    fmt::print(
        "WARNING: Color {} cannot be set, setting this to `white`. Please consider add this to the pre-defined color map, or set it by explicitly call `glColor4f(r,g,b,a);`\n",
        colorName);
    glColor3f(1.0f, 1.0f, 1.0f);
  }
}

} // namespace projectaria::tools::viz
