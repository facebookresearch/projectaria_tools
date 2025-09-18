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

# - Find Opus
# Find the Opus Interactive Audio Codec library
#
# Opus_INCLUDE_DIRS - where to find opus.h, etc.
# Opus_LIBRARIES - List of libraries when using opus.
# Opus_FOUND - True if Opus found.
#
# As this library is not mandatory, we set:
# BUILD_WITH_OPUS to ON if found, OFF otherwise
#

find_path(Opus_INCLUDE_DIRS NAMES opus.h PATH_SUFFIXES opus)
find_library(Opus_LIBRARIES NAMES opus)

mark_as_advanced(Opus_LIBRARIES Opus_INCLUDE_DIRS)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Opus DEFAULT_MSG Opus_LIBRARIES Opus_INCLUDE_DIRS)

if (Opus_FOUND AND NOT (TARGET Opus::opus))
  add_library(Opus::opus UNKNOWN IMPORTED)
  set_target_properties(Opus::opus
    PROPERTIES
      IMPORTED_LOCATION ${Opus_LIBRARIES}
      INTERFACE_INCLUDE_DIRECTORIES ${Opus_INCLUDE_DIRS}
  )
  set(BUILD_WITH_OPUS ON)
else ()
  set(BUILD_WITH_OPUS OFF)
endif()
