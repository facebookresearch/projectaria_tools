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

add_executable(test_periodic TestPeriodic.cpp)
target_link_libraries(test_periodic
    PUBLIC
    vrs_health_check
    GTest::Main
)
gtest_discover_tests(test_periodic)
add_test(NAME test_periodic WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:test_periodic>)
target_compile_definitions(test_periodic
    PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../data/)
