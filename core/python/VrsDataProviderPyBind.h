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

#include <data_provider/VrsDataProvider.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "SensorDataSequence.h"

namespace projectaria::tools::data_provider {

namespace py = pybind11;
namespace {
inline void declareSubstreamSelector(py::module& m) {
  py::class_<SubstreamSelector>(
      m,
      "SubstreamSelector",
      "Class for subselecting VRS streams from all streams available in an VRS file.")
      .def(py::init<const std::set<vrs::StreamId>&>())
      .def(
          "get_stream_ids",
          [](const SubstreamSelector& self) {
            auto streams = self.getStreamIds();
            return std::vector<vrs::StreamId>{streams.begin(), streams.end()};
          },
          "Returns the list of available stream ids.")
      .def(
          "get_type_ids",
          [](const SubstreamSelector& self) {
            auto types = self.getTypeIds();
            return std::vector<vrs::RecordableTypeId>{types.begin(), types.end()};
          },
          "Returns the list of available type ids.")
      .def(
          "get_stream_ids",
          [](const SubstreamSelector& self, const vrs::RecordableTypeId& typeId) {
            auto streams = self.getStreamIds(typeId);
            return std::vector<vrs::StreamId>{streams.begin(), streams.end()};
          },
          "Returns the list of stream ids of a specified type.")
      .def(
          "is_active",
          &SubstreamSelector::isActive,
          py::arg("stream_id"),
          "Returns true if a stream has been selected.")
      .def(
          "get_active_stream_ids",
          [](const SubstreamSelector& self) {
            auto streams = self.getActiveStreamIds();
            return std::vector<vrs::StreamId>{streams.begin(), streams.end()};
          },
          "Returns all selected streams.")
      .def(
          "activate_stream",
          [](SubstreamSelector& self, const vrs::StreamId& streamId) {
            return self.activateStream(streamId);
          },
          "Activate a VRS stream (turn on).")
      .def(
          "deactivate_stream",
          [](SubstreamSelector& self, const vrs::StreamId& streamId) {
            return self.deactivateStream(streamId);
          },
          "Deactivate a VRS stream (turn off).")
      .def(
          "toggle_stream",
          [](SubstreamSelector& self, const vrs::StreamId& streamId) {
            return self.toggleStream(streamId);
          },
          "Toggles a VRS stream from on to off or from off to on.")
      .def(
          "activate_stream",
          [](SubstreamSelector& self, const vrs::StreamId& typeId) {
            return self.activateStream(typeId);
          },
          "Turns on all streams of a specific typeId, regardless of current state.")
      .def(
          "deactivate_stream",
          [](SubstreamSelector& self, const vrs::StreamId& typeId) {
            return self.deactivateStream(typeId);
          },
          "Turns on all streams of a specific typeId, regardless of current state.")
      .def(
          "activate_stream_all",
          &SubstreamSelector::activateStreamAll,
          "Turns on all available streams, regardless of current state.")
      .def(
          "deactivate_stream_all",
          &SubstreamSelector::deactivateStreamAll,
          "Turns off all available streams, regardless of current state.");
}

inline void declareDeliverQueued(py::module& m) {
  py::class_<DeliverQueuedOptions, SubstreamSelector>(
      m,
      "DeliverQueuedOptions",
      "Options for delivering sensor data of multiple streams sorted in device timestamps.")
      .def(py::init<const int64_t, const int64_t, const std::map<vrs::StreamId, size_t>&>())
      .def(
          "get_truncate_first_device_time_ns",
          &DeliverQueuedOptions::getTruncateFirstDeviceTimeNs,
          "Returns how many nanoseconds to skip from the beginning of the vrs recording.")
      .def(
          "get_truncate_last_device_time_ns",
          &DeliverQueuedOptions::getTruncateLastDeviceTimeNs,
          "Returns how many nanoseconds to skip before the end of the vrs recording.")
      .def(
          "get_subsample_rate",
          &DeliverQueuedOptions::getSubsampleRate,
          py::arg("stream_id"),
          "Returns how many times the frame rate is downsampled in a stream.")
      .def(
          "set_truncate_first_device_time_ns",
          &DeliverQueuedOptions::setTruncateFirstDeviceTimeNs,
          py::arg("time_ns"),
          "Sets how much time to skip from the beginning of the recording.")
      .def(
          "set_truncate_last_device_time_ns",
          &DeliverQueuedOptions::setTruncateLastDeviceTimeNs,
          py::arg("time_ns"),
          "Sets how much time to skip from the end of the recording.")
      .def(
          "set_subsample_rate",
          &DeliverQueuedOptions::setSubsampleRate,
          py::arg("stream_id"),
          py::arg("rate"),
          "Sets how many times the frame rate is downsampled in a stream i.e, after a data is played, rate - 1 data are skipped.");

  py::class_<SensorDataIterator>(
      m, "SensorDataIterator", "Forward iterator for a sensor data container")
      .def(py::init<>());
  py::class_<SensorDataSequence>(
      m,
      "SensorDataSequence",
      "Interface for delivering sensor data sorted by timestamps, with iterator support.")
      .def(py::init<VrsDataProvider*, const DeliverQueuedOptions&>());
}

inline void declareVrsDataProvider(py::module& m) {
  py::class_<VrsDataProvider, std::shared_ptr<VrsDataProvider>>(
      m,
      "VrsDataProvider",
      "Given a vrs file that contains data collected from Aria devices, createVrsDataProvider will create and return a new VrsDataProvider object. A VrsDataProvider object can be used to access sensor data from a vrs file including image data, IMU data, calibration data and more.")
      .def(py::init<
           const std::shared_ptr<RecordReaderInterface>&,
           const std::shared_ptr<StreamIdConfigurationMapper>&,
           const std::shared_ptr<TimeSyncMapper>&,
           const std::shared_ptr<StreamIdLabelMapper>&,
           const std::optional<calibration::DeviceCalibration>&>())
      .def(
          "get_all_streams",
          [](const VrsDataProvider& self) {
            auto streams = self.getAllStreams();
            return std::vector<vrs::StreamId>{streams.begin(), streams.end()};
          },
          "Get all available streams from the vrs file.")
      .def(
          "get_file_tags",
          [](const VrsDataProvider& self) { return self.getFileTags(); },
          "Get the tags map from the vrs file.")
      .def(
          "get_sensor_data_type",
          &VrsDataProvider::getSensorDataType,
          py::arg("stream_id"),
          "Get sensor_data_type from stream_id.")
      .def(
          "get_label_from_stream_id",
          &VrsDataProvider::getLabelFromStreamId,
          py::arg("stream_id"),
          "Get label from stream_id as opposed to get_stream_id_from_label().")
      .def(
          "get_stream_id_from_label",
          &VrsDataProvider::getStreamIdFromLabel,
          py::arg("label"),
          "Get stream_id from label as opposed to get_label_from_stream_id().")
      .def(
          "check_stream_is_active",
          &VrsDataProvider::checkStreamIsActive,
          py::arg("stream_id"),
          "Check, if a stream with provided ID is active.")
      .def(
          "check_stream_is_type",
          &VrsDataProvider::checkStreamIsType,
          py::arg("stream_id"),
          py::arg("type"),
          "Checks, if a stream with provided ID is of expected type.")
      .def(
          "get_device_calibration",
          &VrsDataProvider::getDeviceCalibration,
          "Get calibration of the device.")
      .def(
          "get_sensor_calibration",
          &VrsDataProvider::getSensorCalibration,
          py::arg("stream_id"),
          "Get calibration of a sensor from the device.")
      .def(
          "get_configuration",
          &VrsDataProvider::getConfiguration,
          py::arg("stream_id"),
          "Get configuration of a specific stream.")
      .def(
          "get_nominal_rate_hz",
          &VrsDataProvider::getNominalRateHz,
          py::arg("stream_id"),
          "Gets the nominal frame rate in Hz of a specific stream.")
      .def(
          "get_nominalRateHz",
          [](const VrsDataProvider& self, const vrs::StreamId& streamId) -> double {
            auto warnings = pybind11::module::import("warnings");
            warnings.attr("warn")(
                "get_nominalRateHz(stream_id) is deprecated, use get_nominal_rate_hz(stream_id) instead.");
            return self.getNominalRateHz(streamId);
          },
          py::arg("stream_id"),
          "Gets the nominal frame rate in Hz of a specific stream.")
      .def(
          "get_default_deliver_queued_options",
          &VrsDataProvider::getDefaultDeliverQueuedOptions,
          "Default options that delivers all sensor data in vrs from start to end in TimeDomain.DEVICE_TIME.")
      .def(
          "deliver_queued_sensor_data",
          [](VrsDataProvider& self) {
            auto iterPair = self.makeIterator();
            return py::make_iterator(iterPair.first, iterPair.second);
          },
          py::keep_alive<0, 1>(),
          "Delivers data from all sensors in the entire vrs file sorted by TimeDomain.DEVICE_TIME.")
      .def(
          "deliver_queued_sensor_data",
          [](VrsDataProvider& self, const DeliverQueuedOptions& options) {
            auto iterPair = self.makeIterator(options);
            return py::make_iterator(iterPair.first, iterPair.second);
          },
          py::keep_alive<0, 1>(),
          "Delivers data from vrs file with options sorted by TimeDomain.DEVICE_TIME.")
      .def(
          "get_num_data",
          &VrsDataProvider::getNumData,
          py::arg("stream_id"),
          "Return number of collected sensor data of a stream.")
      .def(
          "get_sensor_data_by_index",
          &VrsDataProvider::getSensorDataByIndex,
          py::arg("stream_id"),
          py::arg("index"),
          "Return the N-th data of a stream, return SensorData of NOT_VALID if out of range. see SensorData for more details on how sensor data are represented.")
      .def(
          "supports_time_domain",
          &VrsDataProvider::supportsTimeDomain,
          py::arg("stream_id"),
          py::arg("time_domain"),
          "Check if a stream contains timestamp of a specific time domain specifically, Audio, Barometer, and GPS data does not have host timestamps. if the vrs does not contain a timesync stream with timecode mode, then timecode is not supported.")
      .def(
          "get_first_time_ns",
          &VrsDataProvider::getFirstTimeNs,
          py::arg("stream_id"),
          py::arg("time_domain"),
          "Get first timestamp in nanoseconds of a stream_id at a particular timeDomain.")
      .def(
          "get_last_time_ns",
          &VrsDataProvider::getLastTimeNs,
          py::arg("stream_id"),
          py::arg("time_domain"),
          "Get last timestamp in nanoseconds of a stream_id at a particular timeDomain.")
      .def(
          "get_first_time_ns_all_streams",
          &VrsDataProvider::getFirstTimeNsAllStreams,
          py::arg("time_domain"),
          "Get first timestamp in nanoseconds of all stream_ids at a particular timeDomain.")
      .def(
          "get_last_time_ns_all_streams",
          &VrsDataProvider::getLastTimeNsAllStreams,
          py::arg("time_domain"),
          "Get last timestamp in nanoseconds of all stream_ids at a particular timeDomain.")
      .def(
          "get_sensor_data_by_time_ns",
          &VrsDataProvider::getSensorDataByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before,
          "Get sensorData from a specific timestamp in nanosecond from sensor stream_id.")
      .def(
          "get_index_by_time_ns",
          &VrsDataProvider::getIndexByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before,
          "Get index of a the data from query timestamp in nanoseconds.")
      .def(
          "get_timestamps_ns",
          &VrsDataProvider::getTimestampsNs,
          py::arg("stream_id"),
          py::arg("time_domain"),
          "Get all timestamps in nanoseconds as a vector for stream_id of a particular timeDomain.")
      .def(
          "convert_from_timecode_to_device_time_ns",
          &VrsDataProvider::convertFromTimeCodeToDeviceTimeNs,
          py::arg("timecode_time_ns"),
          "Convert TIME_CODE timestamp into DEVICE_TIME in nanoseconds.")
      .def(
          "convert_from_device_time_to_timecode_ns",
          &VrsDataProvider::convertFromDeviceTimeToTimeCodeNs,
          py::arg("device_time_ns"),
          "Convert DEVICE_TIME timestamp into TIME_CODE in nanoseconds.")
      .def(
          "convert_from_device_time_to_synctime_ns",
          &VrsDataProvider::convertFromDeviceTimeToSyncTimeNs,
          py::arg("device_time_ns"),
          py::arg("mode"),
          "Convert DeviceTime timestamp into synchronized timestamp in nanoseconds.")
      .def(
          "convert_from_synctime_to_device_time_ns",
          &VrsDataProvider::convertFromSyncTimeToDeviceTimeNs,
          py::arg("sync_time_ns"),
          py::arg("mode"),
          "Convert sync timestamp into synchronized timestamp in nanoseconds.")

      /* Get data configuration*/
      .def("get_image_configuration", &VrsDataProvider::getImageConfiguration, py::arg("stream_id"))
      .def("get_imu_configuration", &VrsDataProvider::getImuConfiguration, py::arg("stream_id"))
      .def("get_gps_configuration", &VrsDataProvider::getGpsConfiguration, py::arg("stream_id"))
      .def("get_wps_configuration", &VrsDataProvider::getWpsConfiguration, py::arg("stream_id"))
      .def("get_audio_configuration", &VrsDataProvider::getAudioConfiguration, py::arg("stream_id"))
      .def(
          "get_barometer_configuration",
          &VrsDataProvider::getBarometerConfiguration,
          py::arg("stream_id"))
      .def(
          "get_bluetooth_configuration",
          &VrsDataProvider::getBluetoothConfiguration,
          py::arg("stream_id"))
      .def(
          "get_magnetometer_configuration",
          &VrsDataProvider::getMagnetometerConfiguration,
          py::arg("stream_id"))

      /* Get data from index */
      .def(
          "get_image_data_by_index",
          &VrsDataProvider::getImageDataByIndex,
          py::arg("stream_id"),
          py::arg("index"))
      .def(
          "get_imu_data_by_index",
          &VrsDataProvider::getImuDataByIndex,
          py::arg("stream_id"),
          py::arg("index"))
      .def(
          "get_gps_data_by_index",
          &VrsDataProvider::getGpsDataByIndex,
          py::arg("stream_id"),
          py::arg("index"))
      .def(
          "get_wps_data_by_index",
          &VrsDataProvider::getWpsDataByIndex,
          py::arg("stream_id"),
          py::arg("index"))
      .def(
          "get_audio_data_by_index",
          &VrsDataProvider::getAudioDataByIndex,
          py::arg("stream_id"),
          py::arg("index"))
      .def(
          "get_barometer_data_by_index",
          &VrsDataProvider::getBarometerDataByIndex,
          py::arg("stream_id"),
          py::arg("index"))
      .def(
          "get_bluetooth_data_by_index",
          &VrsDataProvider::getBluetoothDataByIndex,
          py::arg("stream_id"),
          py::arg("index"))
      .def(
          "get_magnetometer_data_by_index",
          &VrsDataProvider::getMagnetometerDataByIndex,
          py::arg("stream_id"),
          py::arg("index"))

      /* Get data before timestamp in nanoseconds*/
      .def(
          "get_image_data_by_time_ns",
          &VrsDataProvider::getImageDataByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before)
      .def(
          "get_imu_data_by_time_ns",
          &VrsDataProvider::getImuDataByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before)
      .def(
          "get_gps_data_by_time_ns",
          &VrsDataProvider::getGpsDataByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before)
      .def(
          "get_wps_data_by_time_ns",
          &VrsDataProvider::getWpsDataByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before)
      .def(
          "get_audio_data_by_time_ns",
          &VrsDataProvider::getAudioDataByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before)
      .def(
          "get_barometer_data_by_time_ns",
          &VrsDataProvider::getBarometerDataByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before)
      .def(
          "get_bluetooth_data_by_time_ns",
          &VrsDataProvider::getBluetoothDataByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before)
      .def(
          "get_magnetometer_data_by_time_ns",
          &VrsDataProvider::getMagnetometerDataByTimeNs,
          py::arg("stream_id"),
          py::arg("time_ns"),
          py::arg("time_domain"),
          py::arg("time_query_options") = TimeQueryOptions::Before);
}
} // namespace

inline void exportVrsDataProvider(py::module& m) {
  // For submodule documentation, see: projectaria_tools/projectaria_tools/core/data_provider.py

  m.def(
      "create_vrs_data_provider",
      &createVrsDataProvider,
      py::arg("vrs_filename"),
      "Factory class to create a VrsDataProvider class.");

  declareSubstreamSelector(m);
  declareDeliverQueued(m);
  declareVrsDataProvider(m);
}

} // namespace projectaria::tools::data_provider
