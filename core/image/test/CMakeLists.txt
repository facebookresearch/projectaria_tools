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

add_executable(image_test ImageTest.cpp)
target_link_libraries(image_test
    PUBLIC
        image
        GTest::Main
)
gtest_discover_tests(image_test)
add_test(NAME image_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:image_test>)

add_executable(managed_image_test ManagedImageTest.cpp)
target_link_libraries(managed_image_test
    PUBLIC
        image
        GTest::Main
)
gtest_discover_tests(managed_image_test)
add_test(NAME managed_image_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:managed_image_test>)

add_executable(image_variant_test ImageVariantTest.cpp)
target_link_libraries(image_variant_test
    PUBLIC
        image
        GTest::Main
)
gtest_discover_tests(image_variant_test)
add_test(NAME image_variant_test WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
             COMMAND $<TARGET_FILE:image_variant_test>)
