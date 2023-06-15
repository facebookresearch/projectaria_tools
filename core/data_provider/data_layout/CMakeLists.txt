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
  ./aria/*.h
)

add_library(data_layout INTERFACE)
target_sources(data_layout INTERFACE ${header_files})
target_link_libraries(data_layout INTERFACE vrslib)
target_include_directories(data_layout INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..)

if (NOT BUILD_PYTHON_BINDINGS)
  add_executable(AriaFileReader ./sample_app/AriaFileReader.cpp)
  target_link_libraries(AriaFileReader PRIVATE data_layout)
endif()
