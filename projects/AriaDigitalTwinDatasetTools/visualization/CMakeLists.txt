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

message("---- Compiling Aria Digital Twin Viewer.")

find_package(Pangolin QUIET)
if (Pangolin_FOUND)
    set(BUILD_WITH_PANGOLIN ON)
else()
    message("Pangolin not found. Aria Digital Twin Viewer will not build")
    return()
endif()

# Add viewer lib
add_library(
    AriaDigitalTwinViewerLib
        AriaDigitalTwinViewer.cpp
        AriaDigitalTwinViewer.h
)
target_link_libraries(
    AriaDigitalTwinViewerLib
        PUBLIC
            AriaDigitalTwinDataProviderLib
            ${Pangolin_LIBRARIES}
)
target_include_directories(
    AriaDigitalTwinDataProviderLib
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            "${CMAKE_BINARY_DIR}/_deps"
)

# Add Aria DigitalTwin Viewer Executable
add_executable(
    AriaDigitalTwinViewer
        main_aria_digital_twin_viewer.cpp
)
target_link_libraries(
    AriaDigitalTwinViewer
        PRIVATE
            AriaDigitalTwinViewerLib CLI11::CLI11
)
