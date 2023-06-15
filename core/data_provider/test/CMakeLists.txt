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

find_package(GTest)

add_executable(substream_selector_test SubstreamSelectorTest.cpp)
target_link_libraries(substream_selector_test
    PUBLIC
        substream_selector
        vrs_data_provider
        GTest::Main
)
gtest_discover_tests(substream_selector_test)
add_test(NAME substream_selector_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:substream_selector_test>)
target_compile_definitions(substream_selector_test
    PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../data/)

add_executable(vrs_data_provider_factory_test VrsDataProviderFactoryTest.cpp)
target_link_libraries(vrs_data_provider_factory_test
    PUBLIC
        vrs_data_provider
        GTest::Main
)
gtest_discover_tests(vrs_data_provider_factory_test)
add_test(NAME vrs_data_provider_factory_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:vrs_data_provider_factory_test>)
target_compile_definitions(vrs_data_provider_factory_test
    PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../data/)

add_executable(vrs_data_provider_calibration_test VrsDataProviderCalibrationTest.cpp)
target_link_libraries(vrs_data_provider_calibration_test
    PUBLIC
        vrs_data_provider
        GTest::Main
)
gtest_discover_tests(vrs_data_provider_calibration_test)
add_test(NAME vrs_data_provider_calibration_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:vrs_data_provider_factory_test>)
target_compile_definitions(vrs_data_provider_calibration_test
    PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../data/)

add_executable(vrs_data_provider_configuration_test VrsDataProviderConfigurationTest.cpp)
target_link_libraries(vrs_data_provider_configuration_test
    PUBLIC
        vrs_data_provider
        GTest::Main
)
gtest_discover_tests(vrs_data_provider_configuration_test)
add_test(NAME vrs_data_provider_configuration_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:vrs_data_provider_configuration_test>)
target_compile_definitions(vrs_data_provider_configuration_test
    PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../data/)

add_executable(vrs_data_provider_deliver_queued_test VrsDataProviderDeliverQueuedTest.cpp)
target_link_libraries(vrs_data_provider_deliver_queued_test
    PUBLIC
        vrs_data_provider
        GTest::Main
)
gtest_discover_tests(vrs_data_provider_deliver_queued_test)
add_test(NAME vrs_data_provider_deliver_queued_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:vrs_data_provider_deliver_queued_test>)
target_compile_definitions(vrs_data_provider_deliver_queued_test
    PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../data/)

add_executable(vrs_data_provider_get_data_by_time_test VrsDataProviderGetDataByTimeTest.cpp)
target_link_libraries(vrs_data_provider_get_data_by_time_test
    PUBLIC
        vrs_data_provider
        GTest::Main
)
gtest_discover_tests(vrs_data_provider_get_data_by_time_test)
add_test(NAME vrs_data_provider_get_data_by_time_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:vrs_data_provider_get_data_by_time_test>)
target_compile_definitions(vrs_data_provider_get_data_by_time_test
    PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../data/)

add_executable(vrs_data_provider_get_data_by_index_test VrsDataProviderGetDataByTimeTest.cpp)
target_link_libraries(vrs_data_provider_get_data_by_index_test
    PUBLIC
        vrs_data_provider
        GTest::Main
)
gtest_discover_tests(vrs_data_provider_get_data_by_index_test)
add_test(NAME vrs_data_provider_get_data_by_index_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:vrs_data_provider_get_data_by_index_test>)
target_compile_definitions(vrs_data_provider_get_data_by_index_test
    PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../data/)
