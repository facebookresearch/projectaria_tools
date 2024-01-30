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

#include "AriaEverydayActivitiesDataPathsFormat.h"
#include "AriaEverydayActivitiesDataPathsProvider.h"
#include "AriaEverydayActivitiesDataProvider.h"

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace projectaria::dataset::aea {

inline void exportAriaEverydayActivities(py::module& m) {
  // For module documentation, see: projectaria_tools/projectaria_tools/projects/aea/__init__.py

  // pybind data types
  py::class_<WordData>(
      m,
      "WordData",
      "A simple struct to store data associated with one word from the speech to text data")
      .def(py::init<>())
      .def_readwrite(
          "start_timestamp_ns",
          &WordData::startTimestampNs,
          "Start time for the word in nanoseconds")
      .def_readwrite(
          "end_timestamp_ns", &WordData::endTimestampNs, "End time for the word in nanoseconds")
      .def_readwrite(
          "word",
          &WordData::word,
          "String which contains the word that was detected in the speech to text")
      .def_readwrite(
          "confidence", &WordData::confidence, "Confidence level associated with this word");

  py::class_<SentenceData>(
      m,
      "SentenceData",
      "A simple struct to store data associated with one sentence from the speech to text data. Sentences are broken up by punctuation marks including: .  !  ?")
      .def(py::init<>())
      .def_readwrite(
          "start_timestamp_ns",
          &SentenceData::startTimestampNs,
          "Start time for the sentence in nanoseconds. This is the start time of the first word")
      .def_readwrite(
          "end_timestamp_ns",
          &SentenceData::endTimestampNs,
          "End time for the sentence in nanoseconds. This is the end time of the last word")
      .def_readwrite(
          "words",
          &SentenceData::words,
          "Ordered map of word data, where the words are ordered based on start times")
      .def(
          "to_string",
          &SentenceData::toString,
          "Return the full sentence as a single string. Same as calling str() on the object")
      .def("__str__", [](const SentenceData& d) { return d.toString(); });

  // Data paths and data provider
  py::class_<AriaEverydayActivitiesDataPaths>(
      m,
      "AriaEverydayActivitiesDataPaths",
      "A struct that includes the file paths of all AEA data files for one sequence")
      .def_readwrite("aria_vrs", &AriaEverydayActivitiesDataPaths::ariaVrs, "Aria vrs")
      .def_readwrite("speech", &AriaEverydayActivitiesDataPaths::speech, "Speech csv")
      .def_readwrite("metadata", &AriaEverydayActivitiesDataPaths::metadata, "Speech csv")
      .def_readwrite("mps", &AriaEverydayActivitiesDataPaths::mps, "MPS file paths")
      .def("__repr__", [](AriaEverydayActivitiesDataPaths const& self) {
        return fmt::to_string(self);
      });

  py::class_<AriaEverydayActivitiesDataPathsProvider>(
      m,
      "AriaEverydayActivitiesDataPathsProvider",
      "This class is to load all data file paths from AEA data structure given a sequence path. "
      "Each AEA collection sequence can only contain one Aria device and its associated data: \n"
      "├── sequencePath\n"
      "│   ├── metadata.json\n"
      "│   ├── recording.vrs\n"
      "│   ├── speech.csv\n"
      "│   ├── mps\n"
      "│   │   ├── {SEE MpsDataPathsProvider}\n"
      "This class allows you use dataset root to query all data associated with a single device "
      "recording.")
      .def(py::init<const std::string&>())
      .def(
          "get_data_paths",
          &AriaEverydayActivitiesDataPathsProvider::getDataPaths,
          "Get the resulting data paths which can be fed to the AEA data provider to load the data")
      .def(
          "get_location_number",
          &AriaEverydayActivitiesDataPathsProvider::getLocationNumber,
          "Get the location number. This number is found in the metadata json file, and is also "
          "embedded in the sequence name")
      .def(
          "get_script_number",
          &AriaEverydayActivitiesDataPathsProvider::getScriptNumber,
          "Get the script number. This number is found in the metadata json file, and is also "
          "embedded in the sequence name")
      .def(
          "get_sequence_number",
          &AriaEverydayActivitiesDataPathsProvider::getSequenceNumber,
          "Get the sequence number. This number is found in the metadata json file, and is also "
          "embedded in the sequence name")
      .def(
          "get_recording_number",
          &AriaEverydayActivitiesDataPathsProvider::getRecordingNumber,
          "Get the recording number. This number is found in the metadata json file, and is also "
          "embedded in the sequence name")
      .def(
          "get_concurrent_recordings",
          &AriaEverydayActivitiesDataPathsProvider::getConcurrentRecordings,
          "Get the name(s) of the recording(s) that were collected at the same time and location "
          "as the current recording. This data can be found in the metadata json file")
      .def(
          "get_dataset_name",
          &AriaEverydayActivitiesDataPathsProvider::getDatasetName,
          "Get the name of the current dataset (AEA)")
      .def(
          "get_dataset_version",
          &AriaEverydayActivitiesDataPathsProvider::getDatasetVersion,
          "Get the version of the current dataset (AEA)");

  py::class_<AriaEverydayActivitiesDataProvider>(
      m,
      "AriaEverydayActivitiesDataProvider",
      "This is the core data loader that provide all assets for an AEA sequence")
      .def(py::init<const AriaEverydayActivitiesDataPaths&>())
      .def(py::init<const std::string&>())
      .def("has_aria_data", &AriaEverydayActivitiesDataProvider::hasAriaData)
      .def("has_speech_data", &AriaEverydayActivitiesDataProvider::hasSpeechData)
      .def("has_mps_data", &AriaEverydayActivitiesDataProvider::hasMpsData)
      .def_readwrite(
          "vrs", &AriaEverydayActivitiesDataProvider::vrs, "Access to the Aria VRS data provider")
      .def_readwrite(
          "speech",
          &AriaEverydayActivitiesDataProvider::speech,
          "Access to the speech data provider")
      .def_readwrite(
          "mps", &AriaEverydayActivitiesDataProvider::mps, "Access to the MPS data provider");

  py::class_<SpeechDataProvider, std::shared_ptr<SpeechDataProvider>>(
      m,
      "SpeechDataProvider",
      "Class for reading and querying speech data generated in the AEA sequence")
      .def(py::init<const std::string&>())
      .def(
          "get_sentence_data_by_timestamp_ns",
          &SpeechDataProvider::getSentenceDataByTimestampNs,
          "Get sentence data given a query timestamp. A sentence is a series of words, where each "
          "sentence has a start and end timestamp, and each word has a start and end timestamp."
          "Note: TimeQueryOptions is ignored if there is a sentence that contains the query device "
          "timestamp.",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def(
          "get_word_data_by_timestamp_ns",
          &SpeechDataProvider::getWordDataByTimestampNs,
          "Get word data given a query timestamp. A word has a start and end timestamp and "
          "confidence level. Note: TimeQueryOptions is ignored if there is a word that contains "
          "the query device timestamp.",
          py::arg("device_timestamp_ns"),
          py::arg("time_query_options") = TimeQueryOptions::Closest)
      .def("empty", &SpeechDataProvider::empty);
}

} // namespace projectaria::dataset::aea
