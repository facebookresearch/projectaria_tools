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

add_executable(aria_everyday_activities_data_provider_test AriaEverydayActivitiesDataProviderTest.cpp)
target_link_libraries(aria_everyday_activities_data_provider_test
    PUBLIC
        AriaEverydayActivitiesLib
        GTest::Main
)
gtest_discover_tests(aria_everyday_activities_data_provider_test)
add_test(NAME aria_everyday_activities_data_provider_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:aria_everyday_activities_data_provider_test>)
target_compile_definitions(aria_everyday_activities_data_provider_test
    PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../../data/aria_everyday_activities_test_data/)
