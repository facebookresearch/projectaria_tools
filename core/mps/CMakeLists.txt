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

add_library(eye_gaze EyeGaze.h EyeGazeReader.cpp EyeGazeReader.h)
target_link_libraries(eye_gaze PUBLIC device_calibration_json)
add_dependencies(eye_gaze fast-cpp-csv-parser)
target_include_directories(eye_gaze
    PRIVATE
        "${CMAKE_BINARY_DIR}/_deps"
)

add_library(mps
    OnlineCalibration.h
    onlineCalibrationReader.h onlineCalibrationReader.cpp
    GlobalPointCloud.h
    GlobalPointCloudReader.h GlobalPointCloudReader.cpp
    PointObservation.h
    PointObservationReader.h PointObservationReader.cpp
    Trajectory.h
    TrajectoryReaders.h TrajectoryReaders.cpp
    StaticCameraCalibration.h
    StaticCameraCalibrationReader.h StaticCameraCalibrationReader.cpp
)
target_compile_definitions(mps PRIVATE CEREAL_RAPIDJSON_HAS_STDSTRING)
target_link_libraries(mps
    PUBLIC
        compressed_istream
        device_calibration_json
        eye_gaze
        Sophus::Sophus)
add_dependencies(mps fast-cpp-csv-parser)
target_include_directories(mps
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR} "../"
    PRIVATE
        "${CMAKE_BINARY_DIR}/_deps"
)
