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

# Add a Unit test target for each file starting with Test*.cpp
file(GLOB TEST_FILES Test*.cpp)
foreach(X IN LISTS TEST_FILES)
    get_filename_component(filenameWithExt ${X} NAME)
    string(REGEX REPLACE "\\.[^.]*$" "" filenameWithoutExt ${filenameWithExt})

    add_executable(mps_${filenameWithoutExt} ${filenameWithExt})
    target_link_libraries(mps_${filenameWithoutExt}
    PUBLIC
        mps
        GTest::Main
    )
    gtest_discover_tests(mps_${filenameWithoutExt})
    add_test(NAME mps_${filenameWithoutExt} WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:mps_${filenameWithoutExt}>)
    target_compile_definitions(mps_${filenameWithoutExt}
        PRIVATE -DTEST_FOLDER=${CMAKE_CURRENT_SOURCE_DIR}/../../../data/)
endforeach()
