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

if(BUILD_UNIT_TEST)
    add_subdirectory(test)
endif()

# Add Aria DigitalTwin data provider Library
add_library(
    AriaDigitalTwinDataProviderLib
        AriaDigitalTwinDataTypes.cpp
        AriaDigitalTwinDataTypes.h
        AriaDigitalTwinUtils.cpp
        AriaDigitalTwinUtils.h
        AriaDigitalTwinSkeletonProvider.cpp
        AriaDigitalTwinSkeletonProvider.h
        AriaDigitalTwinDataProvider.cpp
        AriaDigitalTwinDataProvider.h
        AriaDigitalTwinDataPathsProvider.cpp
        AriaDigitalTwinDataPathsProvider.h
        AriaDigitalTwinDataFileNames.h
        AriaDigitalTwinDataFileKeys.h
)
target_link_libraries(
    AriaDigitalTwinDataProviderLib
        PUBLIC
            sensor_data_type
            Sophus::Sophus
            Eigen3::Eigen
            eye_gaze
            mps
            vrslib
            vrs_data_provider
)
target_include_directories(
    AriaDigitalTwinDataProviderLib
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            "${CMAKE_BINARY_DIR}/_deps"
)
