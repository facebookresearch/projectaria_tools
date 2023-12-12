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

add_library(aria_calib_rescale_and_crop AriaCalibRescaleAndCrop.cpp AriaCalibRescaleAndCrop.h)
target_include_directories(aria_calib_rescale_and_crop PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_link_libraries(aria_calib_rescale_and_crop PUBLIC device_calibration PRIVATE vrs_logging)

add_library(sensor_calibration_json SensorCalibrationJson.cpp SensorCalibrationJson.h)
target_include_directories(sensor_calibration_json PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_link_libraries(sensor_calibration_json PUBLIC nlohmann_json::nlohmann_json device_calibration PRIVATE Sophus::Sophus  vrslib vrs_logging)

add_library(device_calibration_json DeviceCalibrationJson.cpp DeviceCalibrationJson.h)
target_include_directories(device_calibration_json PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_link_libraries(device_calibration_json PUBLIC sensor_calibration_json device_calibration PRIVATE vrs_logging nlohmann_json::nlohmann_json)
