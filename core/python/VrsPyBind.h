/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vrs/utils/AudioTrackExtractor.h>
#include <vrs/utils/FilteredFileReader.h>

namespace projectaria::tools::vrspybind {

namespace py = pybind11;

namespace {

inline void declareVrsAudioToWav(py::module& m) {
  m.def(
      "extract_audio_track",
      [](const std::string& vrsFilePath, const std::string& wavFilePath) {
        ::vrs::utils::FilteredFileReader filteredReader;
        // Initialize VRS Reader and filters
        filteredReader.setSource(vrsFilePath);
        filteredReader.openFile();
        filteredReader.applyFilters({});

        return vrs::utils::extractAudioTrack(filteredReader, wavFilePath);
      },
      "Extract the audio stream of a VRS file into a WAV file");
}

} // namespace

inline void exportVrs(py::module& m) {
  declareVrsAudioToWav(m);
}
} // namespace projectaria::tools::vrspybind
