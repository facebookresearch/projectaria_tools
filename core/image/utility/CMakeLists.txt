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

add_library(image_distort Distort.cpp Distort.h)
target_include_directories(image_distort PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_link_libraries(image_distort PUBLIC image dispenso)

add_library(image_debayer Debayer.cpp Debayer.h)
target_include_directories(image_debayer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_link_libraries(image_debayer PUBLIC image)
