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


cmake_policy(SET CMP0076 NEW) # Allow relative path in target_include_directories and INTERFACE

add_subdirectory(camera_projections)
add_subdirectory(loader)
add_subdirectory(utility)

add_library(device_version
    DeviceVersion.cpp DeviceVersion.h)
target_include_directories(device_version PUBLIC "../")

add_library(config_data
    ConfigData.cpp ConfigData.h)
target_include_directories(config_data PUBLIC "../")

add_library(config_file_loader
    ConfigFileLoader.cpp ConfigFileLoader.h)
target_include_directories(config_file_loader PUBLIC "../")
target_link_libraries(config_file_loader PUBLIC device_version PRIVATE format config_data)

add_library(camera_config_builder
    CameraConfigBuilder.cpp CameraConfigBuilder.h)
target_include_directories(camera_config_builder PUBLIC "../")
target_link_libraries(camera_config_builder PUBLIC device_version PRIVATE config_file_loader vrs_logging nlohmann_json::nlohmann_json)


add_library(sensor_calibration
    SensorCalibration.cpp SensorCalibration.h
    CameraCalibration.cpp CameraCalibration.h CameraCalibrationFormat.h
    ImuMagnetometerCalibration.cpp ImuMagnetometerCalibration.h ImuMagnetometerCalibrationFormat.h
    BarometerCalibration.cpp BarometerCalibration.h
    MicrophoneCalibration.cpp MicrophoneCalibration.h
)
target_include_directories(sensor_calibration PUBLIC "../")
target_link_libraries(sensor_calibration PUBLIC format error_handler camera_projection vrs_logging calibration_distort camera_config_builder)

add_library(device_cad_extrinsics DeviceCadExtrinsics.cpp DeviceCadExtrinsics.h)
target_include_directories(device_cad_extrinsics PUBLIC "../")
target_link_libraries(device_cad_extrinsics PUBLIC vrs_logging device_version config_file_loader Eigen3::Eigen Sophus::Sophus)

add_library(device_calibration DeviceCalibration.cpp DeviceCalibration.h)
target_include_directories(device_calibration PUBLIC "../")
target_link_libraries(device_calibration PUBLIC device_cad_extrinsics sensor_calibration Sophus::Sophus image)
