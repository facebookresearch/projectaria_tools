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

# players library
file(
  GLOB_RECURSE
  source_files
  *.cpp
)
file(
  GLOB_RECURSE
  header_files
  *.h
)

add_library(players ${source_files} ${header_files})
target_link_libraries(players PUBLIC data_layout image vrslib vrs_utils Eigen3::Eigen)
target_include_directories(players PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
