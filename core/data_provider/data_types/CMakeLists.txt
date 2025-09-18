# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

file(
  GLOB
  header_files
)

add_library(data_types INTERFACE)
target_sources(data_types INTERFACE ${header_files})
target_link_libraries(data_types INTERFACE Eigen3::Eigen Sophus::Sophus)
target_include_directories(data_types INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..)
