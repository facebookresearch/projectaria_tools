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

# Add Aria Everyday Activities data provider Library
add_library(
    AriaEverydayActivitiesLib
        AriaEverydayActivitiesDataPathsProvider.cpp
        AriaEverydayActivitiesDataPathsProvider.h
        AriaEverydayActivitiesDataPathsFormat.h
        AriaEverydayActivitiesDataProvider.cpp
        AriaEverydayActivitiesDataProvider.h
        AriaEverydayActivitiesDataTypes.h
        AriaEverydayActivitiesFileKeys.h
        AriaEverydayActivitiesFileNames.h
        SpeechDataProvider.cpp
        SpeechDataProvider.h
)
target_link_libraries(
    AriaEverydayActivitiesLib
        PUBLIC
            mps
            vrslib
            vrs_data_provider
)
target_include_directories(
    AriaEverydayActivitiesLib
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            "${CMAKE_BINARY_DIR}/_deps"
)
