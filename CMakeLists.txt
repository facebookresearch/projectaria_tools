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

cmake_minimum_required(VERSION 3.12)

# Check for Windows platform and emit error
# TODO: remove this once we support building on Windows in 2.0+
if(WIN32)
  message(FATAL_ERROR "Building `projectaria_tools` 2.0+ is not yet supported on Windows. Please revert back to 1.6.0.")
endif()

project(projectaria_tools
  VERSION 0.1
  DESCRIPTION "Tools to process and use Project Aria data."
  LANGUAGES CXX)

# Set EIGEN alignment to OFF to avoid memory errors in avx instruction set
add_compile_definitions(
  EIGEN_DONT_VECTORIZE
  EIGEN_DONT_ALIGN
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set unit test data folders
set(GEN1_TEST_DATA_PATH "${CMAKE_SOURCE_DIR}/data/gen1/")
set(GEN2_TEST_DATA_PATH "${CMAKE_SOURCE_DIR}/data/gen2/")

# Build in Release mode by default
if(NOT CMAKE_BUILD_TYPE AND NOT MSVC)
  set(CMAKE_BUILD_TYPE "Release")
endif(NOT CMAKE_BUILD_TYPE AND NOT MSVC)

# Project CMAKE options
option(BUILD_PYTHON_BINDINGS "Build python binding." OFF)
option(BUILD_UNIT_TEST "Build tests." OFF)
option(PROJECTARIA_TOOLS_BUILD_PROJECTS "Build projects." OFF)
option(PROJECTARIA_TOOLS_BUILD_TOOLS "Build tools." OFF)

if(BUILD_UNIT_TEST)
  enable_testing()
endif(BUILD_UNIT_TEST)

# Configure MSVC compiler to correctly link libraries
if(MSVC AND BUILD_PYTHON_BINDINGS)
  set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:\"LIBCMT\"")
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

  add_compile_options(
    $<$<CONFIG:>:/MT> #---------|
    $<$<CONFIG:Debug>:/MTd> #---|-- Statically link the runtime libraries
    $<$<CONFIG:Release>:/MT> #--|
  )
endif()

# Additional CMake configuration to setup 3rd party libraries
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(${CMAKE_MODULE_PATH}/Setup3rdParty.cmake)

# Adding core libraries
message("- Compiling Core.")
add_subdirectory(core)

if(PROJECTARIA_TOOLS_BUILD_PROJECTS)
  message("- Compiling Projects.")
  add_subdirectory(projects)
endif()

if(PROJECTARIA_TOOLS_BUILD_TOOLS AND NOT BUILD_PYTHON_BINDINGS)
  message("- Compiling Tools.")
  add_subdirectory(tools)
endif()
