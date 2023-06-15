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

add_library(common INTERFACE)
target_sources(common INTERFACE Common.h IgnoreJetInfinitesimal.h)
target_link_libraries(common INTERFACE Eigen3::Eigen Sophus::Sophus)
target_include_directories(common INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>)

add_library(camera_models INTERFACE)
target_sources(camera_models INTERFACE Linear.h Spherical.h KannalaBrandtK3.h FisheyeRadTanThinPrism.h)
target_link_libraries(camera_models INTERFACE common)
target_include_directories(camera_models INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>)

add_library(camera_projection CameraProjection.cpp CameraProjection.h)
target_include_directories(camera_projection PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../..>)
target_link_libraries(camera_projection PUBLIC camera_models Eigen3::Eigen Sophus::Sophus)
