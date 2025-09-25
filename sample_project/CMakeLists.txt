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


cmake_minimum_required(VERSION 3.16)

project(
  SampleProject
  VERSION 1.0
  DESCRIPTION "Sample project that uses projectaria_tools c++ library"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add subdirectory with path to projectaria_tools
# in our case, the path is one directory up
# Set the binary directory to a projectaria subfolder in the cmake binary
# directory to avoid confusion with this projects binaries
add_subdirectory(.. ${CMAKE_BINARY_DIR}/projectaria EXCLUDE_FROM_ALL)

add_library(my_custom_camera_models lib/MyCustomCameraModel.cpp lib/MyCustomCameraModel.h)
target_link_libraries(my_custom_camera_models
  PUBLIC
    sensor_calibration
)

add_executable(my_custom_camera_projector my_custom_camera_projector.cpp)

target_link_libraries(my_custom_camera_projector
  PUBLIC
    my_custom_camera_models
)
