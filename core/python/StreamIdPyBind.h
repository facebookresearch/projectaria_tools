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

#include <vrs/StreamId.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace projectaria::tools::data_provider {

namespace py = pybind11;

namespace {
inline void declareStreamId(py::module& m) {
  // RecordableType Id for aria only
  py::class_<vrs::StreamId>(m, "StreamId")
      .def(py::init([](uint16_t recordableTypeId, uint16_t instanceId) {
        return vrs::StreamId(static_cast<vrs::RecordableTypeId>(recordableTypeId), instanceId);
      }))
      .def(py::init([](const std::string& numericName) {
        return vrs::StreamId::fromNumericName(numericName);
      }))
      .def("get_name", &vrs::StreamId::getName, "Returns the name of the StreamId")
      .def("get_type_name", &vrs::StreamId::getTypeName, "Returns the type name of the StreamId")
      .def("is_valid", &vrs::StreamId::isValid, "Returns if a stream is valid")
      .def(
          "get_type_id", &vrs::StreamId::getTypeId, "Returns the RecordeableTypeId of the StreamId")
      .def(
          "get_instance_id",
          &vrs::StreamId::getInstanceId,
          "Returns the instance id of the Stream, range in 1...N")
      .def(
          "__eq__",
          [](const vrs::StreamId& self, const vrs::StreamId& another) { return self == another; },
          "Compares two StreamIds")
      .def("__repr__", [](const vrs::StreamId& streamId) { return streamId.getNumericName(); });
}

inline void declareRecordableTypeId(py::module& m) {
  // RecordableType Id for aria only
  py::enum_<vrs::RecordableTypeId>(
      m, "RecordableTypeId", "Recordable Type Id, e.g. SLAM_CAMERA_DATA")
      // image type
      .value("SLAM_CAMERA_DATA", vrs::RecordableTypeId::SlamCameraData)
      .value("EYE_CAMERA_RECORDABLE_CLASS", vrs::RecordableTypeId::EyeCameraRecordableClass)
      .value("RGB_CAMERA_RECORDABLE_CLASS", vrs::RecordableTypeId::RgbCameraRecordableClass)
      // imu
      .value("SLAM_IMU_DATA", vrs::RecordableTypeId::SlamImuData)
      .value("IMU_RECORDABLE_CLASS", vrs::RecordableTypeId::ImuRecordableClass)
      // mag
      .value("SLAM_MAGNETOMETER_DATA", vrs::RecordableTypeId::SlamMagnetometerData)
      // baro
      .value("BAROMETER_RECORDABLE_CLASS", vrs::RecordableTypeId::BarometerRecordableClass)
      // gps
      .value("GPS_RECORDABLE_CLASS", vrs::RecordableTypeId::GpsRecordableClass)
      // wifi
      .value("WIFI_BEACON_RECORDABLE_CLASS", vrs::RecordableTypeId::WifiBeaconRecordableClass)
      // bluetooth
      .value(
          "BLUETOOTH_BEACON_RECORDABLE_CLASS",
          vrs::RecordableTypeId::BluetoothBeaconRecordableClass)
      // audio
      .value("STEREO_AUDIO_RECORDABLE_CLASS", vrs::RecordableTypeId::StereoAudioRecordableClass)
      // timecode
      .value("TIME_RECORDABLE_CLASS", vrs::RecordableTypeId::TimeRecordableClass)
      .export_values();
}
} // namespace

inline void exportStreamId(py::module& m) {
  m.doc() = "A pybind11 binding for Aria Data Tools streamIds";

  declareRecordableTypeId(m);
  declareStreamId(m);
}
} // namespace projectaria::tools::data_provider
