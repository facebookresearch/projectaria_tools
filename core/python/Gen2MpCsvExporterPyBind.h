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

#include <gen2_mp_csv_exporter/Gen2MpCsvExporter.h>

namespace py = pybind11;

namespace projectaria::tools::mp_csv_exporter {

inline void exportGen2MpCsvExporter(py::module& m) {
  m.def(
      "run_gen2_mp_csv_exporter",
      &runGen2MPCsvExporter,
      py::arg("vrs_path"),
      py::arg("output_folder"),
      py::arg("vio_high_freq_subsample_rate") = 1,
      "Run Gen2 MP csv exporter. ");
}

} // namespace projectaria::tools::mp_csv_exporter
