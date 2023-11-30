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

find_package(Boost REQUIRED)

message("--- Compiling mps_point_cloud_colorization.")
add_executable(mps_point_cloud_colorization
    main_PointCloudColorization.cpp
    GlobalPointCloudToPly.h
    PointCloudColorizer.h
    SLAMPointCloudColorizer.h
    RGBPointCloudColorizer.h)

target_link_libraries(mps_point_cloud_colorization
    PRIVATE
        CLI11::CLI11
        ${Boost_LIBRARIES}
        mps
        vrs_data_provider)
