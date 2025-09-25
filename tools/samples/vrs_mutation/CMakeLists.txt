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

add_library(vrs_image_mutation_interface INTERFACE)
target_sources(vrs_image_mutation_interface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/ImageMutationFilterCopier.h)
target_link_libraries(vrs_image_mutation_interface INTERFACE vrs_utils)
target_include_directories(vrs_image_mutation_interface INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)


find_package(fmt REQUIRED)
add_executable(vrs_mutation main.cpp)
target_link_libraries(vrs_mutation PRIVATE vrs_image_mutation_interface CLI11::CLI11 fmt::fmt)
target_include_directories(vrs_mutation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
