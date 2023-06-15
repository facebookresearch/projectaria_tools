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

include(FetchContent) # once in the project to include the module

FetchContent_Declare(
  vrs
  GIT_REPOSITORY  https://github.com/facebookresearch/vrs.git
  GIT_TAG        00b345cbe30e1de4450642c38e47a296982b6555 # master Apr 28, 2023
)
# Override config for vrs
option(UNIT_TESTS OFF)
option(BUILD_SAMPLES OFF)
option(BUILD_TOOLS OFF)

message("Pulling deps: {vrs}")
FetchContent_MakeAvailable(vrs)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY      https://gitlab.com/libeigen/eigen.git
  GIT_TAG          19cacd3ecb9dab73c2dd7bc39d9193e06ba92bdd # 3.4.90
)
FetchContent_Declare(
  cereal
  GIT_REPOSITORY      https://github.com/USCiLab/cereal.git
  GIT_TAG         ebef1e929807629befafbb2918ea1a08c7194554 # v1.3.2
)
FetchContent_Declare(
  Sophus
  GIT_REPOSITORY      https://github.com/strasdat/Sophus.git
  GIT_TAG          de0f8d3d92bf776271e16de56d1803940ebccab9 # 1.22.10
)

# Override config for Cereal
option(SKIP_PERFORMANCE_COMPARISON ON)
option(BUILD_SANDBOX OFF)
option(BUILD_DOC OFF)
# Override config for Sophus
option(BUILD_SOPHUS_TESTS OFF)
option(BUILD_SOPHUS_EXAMPLES OFF)
add_definitions("-DSOPHUS_DISABLE_ENSURES")

FetchContent_Declare(
  cli11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11
  GIT_TAG        v2.3.2
)

set(dependencies cereal cli11 eigen Sophus)
foreach(X IN LISTS dependencies)
  message("Pulling deps: {${X}}")
  FetchContent_MakeAvailable(${X})
endforeach()

include(ExternalProject)
ExternalProject_Add(
  fast-cpp-csv-parser
  GIT_REPOSITORY https://github.com/ben-strasser/fast-cpp-csv-parser.git
  GIT_TAG        origin/master
  SOURCE_DIR     "${CMAKE_BINARY_DIR}/_deps/fast-cpp-csv-parser"
  # disable following, since it is not needed
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_DIR ""
  INSTALL_COMMAND ""
)
