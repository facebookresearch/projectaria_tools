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

message("--- Compiling Project Aria Synthetic Environment Dataset tools.")

# Add Aria Synthetic Environment calibration provider Library
add_library(
    AriaSyntheticEnvironmentCalibrationProviderLib
        AseCalibrationProvider.cpp
        AseCalibrationProvider.h
)
target_link_libraries(
    AriaSyntheticEnvironmentCalibrationProviderLib
        PUBLIC
            Sophus::Sophus
            Eigen3::Eigen
            sensor_calibration
)
target_include_directories(
    AriaSyntheticEnvironmentCalibrationProviderLib
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            "${CMAKE_BINARY_DIR}/_deps"
)

if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif(BUILD_PYTHON_BINDINGS)
