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

#include "DeviceCalibrationPyBind.h"
#include "ImagePyBind.h"
#include "MpsPyBind.h"
#include "SensorDataPyBind.h"
#include "StreamIdPyBind.h"
#include "VrsDataProviderPyBind.h"
#include "VrsHealthCheckPybind.h"
#include "VrsPyBind.h"

#include "sophus/SophusPyBind.h"

namespace py = pybind11;
using namespace projectaria::tools;

PYBIND11_MODULE(_core_pybinds, m) {
  // For module documentation, see: projectaria_tools/projectaria_tools/core/__init__.py

  py::module sophus = m.def_submodule("sophus");
  sophus::exportSophus(sophus);

  py::module image = m.def_submodule("image");
  image::exportImage(image);

  py::module calibration = m.def_submodule("calibration");
  calibration::exportDeviceCalibration(calibration);

  py::module StreamId = m.def_submodule("stream_id");
  data_provider::exportStreamId(StreamId);

  py::module sensorData = m.def_submodule("sensor_data");
  data_provider::exportSensorData(sensorData);

  py::module dataProvider = m.def_submodule("data_provider");
  data_provider::exportVrsDataProvider(dataProvider);

  py::module mps = m.def_submodule("mps");
  mps::exportMps(mps);

  py::module vrs = m.def_submodule("vrs");
  vrspybind::exportVrs(vrs);

  py::module vrsHealthCheck = m.def_submodule("vrs_health_check");
  vrs_check::exportVrsHealthCheck(vrsHealthCheck);
}
