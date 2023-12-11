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

#include <vrs_health_check/Utils.h>
#include <vrs_health_check/VrsHealthCheck.h>

namespace py = pybind11;

namespace projectaria::tools::vrs_check {

inline void exportVrsHealthCheck(py::module& m) {
  py::class_<Settings>(m, "Settings", "Vrs health check settings.")
      .def(py::init<>())
      .def_readwrite("max_imu_skip_us", &Settings::maxImuSkipUs)
      .def_readwrite("max_frame_drop_us", &Settings::maxFrameDropUs)
      .def_readwrite("physical_accel_threshold", &Settings::physicalAccelThreshold)
      .def_readwrite("max_non_physical_accel", &Settings::maxNonPhysicalAccel)
      .def_readwrite(
          "max_allowed_rotation_accel_rad_per_s2", &Settings::maxAllowedRotationAccel_radPerS2)
      .def_readwrite("default_imu_period_us", &Settings::defaultImuPeriodUs)
      .def_readwrite("min_imu_score", &Settings::minImuScore)
      .def_readwrite("min_baro_score", &Settings::minBaroScore)
      .def_readwrite("min_temp", &Settings::minTemp)
      .def_readwrite("max_temp", &Settings::maxTemp)
      .def_readwrite("min_camera_score", &Settings::minCameraScore)
      .def_readwrite("min_camera_gain", &Settings::minCameraGain)
      .def_readwrite("max_camera_gain", &Settings::maxCameraGain)
      .def_readwrite("min_camera_exposure_ms", &Settings::minCameraExposureMs)
      .def_readwrite("max_camera_exposure_ms", &Settings::maxCameraExposureMs)
      .def_readwrite("min_time_domain_mapping_score", &Settings::minTimeDomainMappingScore)
      .def_readwrite("min_audio_score", &Settings::minAudioScore)
      .def_readwrite("min_alignment_score", &Settings::minAlignmentScore)
      .def_readwrite("ignore_gps", &Settings::ignoreGps)
      .def_readwrite("min_gps_accuracy", &Settings::minGpsAccuracy)
      .def_readwrite("default_gps_rate_hz", &Settings::defaultGpsRateHz)
      .def_readwrite("ignore_audio", &Settings::ignoreAudio)
      .def_readwrite("ignore_bluetooth", &Settings::ignoreBluetooth)
      .def_readwrite("is_interactive", &Settings::isInteractive);

  m.def(
      "run",
      [](const std::string& path,
         const std::string& json_out_filename,
         Settings settings,
         const std::string& dropped_out_filename,
         const bool print_stats,
         const bool disable_logging) {
        return Utils::runVrsHealthCheck(
            path,
            json_out_filename,
            std::move(settings),
            dropped_out_filename,
            print_stats,
            disable_logging);
      },
      py::arg("path"),
      py::arg("json_out_filename") = "",
      py::arg("settings") = Settings{},
      py::arg("dropped_out_filename") = "",
      py::arg("print_stats") = false,
      py::arg("disable_logging") = false,
      "Run vrs health check.");
}

} // namespace projectaria::tools::vrs_check
