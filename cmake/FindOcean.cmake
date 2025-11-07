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

# FindOcean.cmake — fetch, build, and install Facebook Ocean as static libraries
# Place this in a folder on CMAKE_MODULE_PATH (e.g. cmake/modules)

# Prevent multiple inclusion
if(Ocean_FOUND)
  return()
endif()

# --------------------------
# 0) Standard install dirs
include(GNUInstallDirs)

# --------------------------
# 1) Fetch & build Ocean
include(FetchContent)
FetchContent_Declare(
  ocean
  GIT_REPOSITORY https://github.com/facebookresearch/ocean.git
  GIT_TAG        5fca1d27a9f37e868738b6db09a6b1e00723d80f # master branch, 2025.11.07
)
set(OCEAN_BUILD_MINIMAL ON)
set(OCEAN_BUILD_TEST    OFF)
message(STATUS "⤷ FetchContent: pulling and building Facebook Ocean…")
FetchContent_MakeAvailable(ocean)

# --------------------------
# 2) Normalize component include dirs to avoid build-tree paths
foreach(_lib IN ITEMS ocean_base ocean_cv ocean_math)
  # Clear any pre-existing include dirs
  set_target_properties(${_lib} PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ""
  )
  # Add proper GENERATOR_EXPR-based includes
  target_include_directories(${_lib} INTERFACE
    $<BUILD_INTERFACE:${ocean_SOURCE_DIR}/impl>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
endforeach()

# --------------------------
# 3) Bundle into a single INTERFACE target
add_library(ocean INTERFACE)
target_link_libraries(ocean INTERFACE
  ocean_base
  ocean_cv
  ocean_math
)
add_library(Ocean::Ocean ALIAS ocean)

# --------------------------
# 4) Installation
install(
  TARGETS
    ocean_base
    ocean_cv
    ocean_math
    ocean
  EXPORT OceanTargets
  ARCHIVE      DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES     DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
  DIRECTORY ${ocean_SOURCE_DIR}/impl/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h"
)
install(
  EXPORT OceanTargets
  FILE   FindOceanTargets.cmake
  NAMESPACE Ocean::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Ocean
)

# --------------------------
# 5) In-tree find_package support
set(Ocean_FOUND     TRUE)
set(Ocean_LIBRARIES Ocean::Ocean)
set(Ocean_INCLUDE_DIRS "${ocean_SOURCE_DIR}/impl")
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ocean
  REQUIRED_VARS Ocean_LIBRARIES
)
