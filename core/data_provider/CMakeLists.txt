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

add_subdirectory(data_layout)
add_subdirectory(players)

if(BUILD_UNIT_TEST)
    add_subdirectory(test)
endif()

cmake_policy(SET CMP0076 NEW) # Allow relative path in target_include_directories and INTERFACE

add_library(aria_stream_ids INTERFACE)
target_sources(aria_stream_ids INTERFACE AriaStreamIds.h)
target_link_libraries(aria_stream_ids INTERFACE vrslib)
target_include_directories(aria_stream_ids INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

add_library(error_handler INTERFACE)
target_sources(error_handler INTERFACE ErrorHandler.h)
target_link_libraries(error_handler INTERFACE vrslib)
target_include_directories(error_handler INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

add_library(sensor_data_type STATIC SensorDataType.cpp TimeTypes.h SensorDataType.h)
target_include_directories(sensor_data_type PUBLIC "../")
target_link_libraries(sensor_data_type PUBLIC vrslib)

add_library(sensor_data STATIC SensorData.cpp SensorData.h)
target_include_directories(sensor_data PUBLIC "../")
target_link_libraries(sensor_data PUBLIC error_handler sensor_data_type vrslib players)

add_library(sensor_configuration STATIC SensorConfiguration.cpp SensorConfiguration.h)
target_include_directories(sensor_configuration PUBLIC "../")
target_link_libraries(sensor_configuration PUBLIC error_handler sensor_data_type vrslib players)

add_library(substream_selector STATIC SubstreamSelector.cpp SubstreamSelector.h)
target_include_directories(substream_selector PUBLIC "../")
target_link_libraries(substream_selector PUBLIC error_handler vrslib) #vrs_logging

add_library(deliver_queued_options STATIC DeliverQueuedOptions.cpp DeliverQueuedOptions.h)
target_include_directories(deliver_queued_options PUBLIC "../")
target_link_libraries(deliver_queued_options PUBLIC error_handler substream_selector vrslib)

add_library(streamid_label_mapper STATIC StreamIdLabelMapper.cpp StreamIdLabelMapper.h)
target_include_directories(streamid_label_mapper PUBLIC "../")
target_link_libraries(streamid_label_mapper PUBLIC vrslib)

add_library(timecode_mapper STATIC TimeCodeMapper.cpp TimeCodeMapper.h)
target_include_directories(timecode_mapper PUBLIC "../")
target_link_libraries(timecode_mapper PUBLIC error_handler vrslib players)

add_library(record_reader_interface STATIC RecordReaderInterface.cpp RecordReaderInterface.h)
target_include_directories(record_reader_interface PUBLIC "../")
target_link_libraries(record_reader_interface PUBLIC sensor_data)

add_library(timestamp_index_mapper STATIC TimestampIndexMapper.cpp TimestampIndexMapper.h)
target_include_directories(timestamp_index_mapper PUBLIC "../")
target_link_libraries(timestamp_index_mapper PUBLIC error_handler record_reader_interface)

add_library(streamid_configuration_mapper STATIC StreamIdConfigurationMapper.cpp StreamIdConfigurationMapper.h)
target_include_directories(streamid_configuration_mapper PUBLIC "../")
target_link_libraries(streamid_configuration_mapper PUBLIC sensor_configuration)

add_library(vrs_data_provider STATIC
        VrsDataProvider.cpp VrsDataProvider.h
        VrsDataProviderFactory.cpp
        SensorDataSequence.cpp SensorDataSequence.h)
target_include_directories(vrs_data_provider PUBLIC "../")
target_link_libraries(vrs_data_provider PUBLIC
        aria_stream_ids
        deliver_queued_options
        record_reader_interface
        sensor_configuration
        sensor_data
        streamid_configuration_mapper
        streamid_label_mapper
        timecode_mapper
        timestamp_index_mapper
        aria_calib_rescale_and_crop
        device_calibration_json
)
