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

set(SOURCE_FILES
    MpvCsvWriter.cpp
    OnlineCalibrationJson.cpp
)
set(HEADER_FILES
    MpvCsvWriter.h
    OnlineCalibrationJson.h
)

# Add the library target
add_library(mp_csv_exporter_file_io ${SOURCE_FILES} ${HEADER_FILES})

# Link libraries
target_link_libraries(mp_csv_exporter_file_io PUBLIC json_io vrs_data_provider)

# Include directories
target_include_directories(mp_csv_exporter_file_io
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../../
)
