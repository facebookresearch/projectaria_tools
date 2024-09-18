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

find_package(Boost REQUIRED COMPONENTS iostreams)

add_library(compressed_istream CompressedIStream.cpp CompressedIStream.h)
target_link_libraries(compressed_istream PUBLIC Boost::iostreams)

add_library(eye_gaze EyeGaze.h EyeGazeReader.cpp EyeGazeFormat.h EyeGazeReader.h)
target_link_libraries(eye_gaze PUBLIC device_calibration_json Sophus::Sophus)
add_dependencies(eye_gaze fast-cpp-csv-parser)
target_include_directories(eye_gaze
    PRIVATE
        "${CMAKE_BINARY_DIR}/_deps"
)

add_library(mps
    CachedDataProviders.h
    OnlineCalibration.h
    OnlineCalibrationFormat.h
    OnlineCalibrationsReader.h OnlineCalibrationsReader.cpp
    GlobalPointCloud.h
    GlobalPointCloudFormat.h
    GlobalPointCloudFilter.h
    GlobalPointCloudReader.h GlobalPointCloudReader.cpp
    HandTrackingFormat.h
    HandTrackingReader.h HandTrackingReader.cpp
    MpsDataPathsProvider.h MpsDataPathsProvider.cpp
    MpsDataProvider.h MpsDataProvider.cpp
    MpsDataPathsFormat.h
    PointObservation.h
    PointObservationFormat.h
    PointObservationReader.h PointObservationReader.cpp
    Trajectory.h
    TrajectoryFormat.h
    TrajectoryReaders.h TrajectoryReaders.cpp
    StaticCameraCalibration.h
    StaticCameraCalibrationFormat.h
    StaticCameraCalibrationReader.h StaticCameraCalibrationReader.cpp
    VersionReader.h VersionReader.cpp
)
target_link_libraries(mps
    PUBLIC
        compressed_istream
        device_calibration_json
        eye_gaze
        format
        Sophus::Sophus
    PRIVATE
        utils
        vrs_logging
        sensor_calibration_json)
add_dependencies(mps fast-cpp-csv-parser)
target_include_directories(mps
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR} "../"
    PRIVATE
        "${CMAKE_BINARY_DIR}/_deps"
)

if(BUILD_UNIT_TEST)
    add_subdirectory(test)
endif()
