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

#include <data_provider/ErrorHandler.h>
#include <data_provider/SensorConfiguration.h>
#include <data_provider/SensorData.h>
#include <data_provider/SensorDataType.h>
#include <data_provider/TimeTypes.h>

#include "ImageDataHelper.h"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace projectaria::tools::data_provider {

namespace py = pybind11;

namespace {
inline void declareTimeEnums(py::module& m) {
  py::enum_<TimeDomain>(
      m, "TimeDomain", "Enum class for different types of timestamps used in projectaria_tools")
      .value(
          "RECORD_TIME",
          TimeDomain::RecordTime,
          "timestamp directly stored in vrs index, fast to access, but not guaranteed which time domain")
      .value(
          "DEVICE_TIME",
          TimeDomain::DeviceTime,
          "capture time in device's timedomain, <b>accurate</b>. All sensors on the same Aria glass share the same device time domain as they are issued from the same clock. We <b>strongly recommend</b> to always work with the device timestamp when dealing with <b>single-device</b> Aria data.")
      .value(
          "HOST_TIME",
          TimeDomain::HostTime,
          "arrival time in host computer's timedomain, may not be accurate")
      .value(
          "TIME_CODE",
          TimeDomain::TimeCode,
          "capture in TimeSync server's timedomain, accurate across devices in a <b>multi-device</b> data capture.")
      .export_values();

  m.def(
      "get_time_domain_name",
      [](const TimeDomain& domain) { return getName(domain); },
      "A helper function to return a descriptive name for a given TimeDomain enum");

  py::enum_<TimeQueryOptions>(m, "TimeQueryOptions")
      .value("BEFORE", TimeQueryOptions::Before, "the last valid data with `timestamp <= t_query")
      .value("AFTER", TimeQueryOptions::After, "the first valid data with `timestamp >= t_query")
      .value(
          "CLOSEST",
          TimeQueryOptions::Closest,
          "the data whose `|timestamp - t_query|` is smallest")
      .export_values();
}

inline void declareSensorDataType(py::module& m) {
  py::enum_<SensorDataType>(
      m,
      "SensorDataType",
      "Enum class for different types of sensor data used in projectaria_tools")
      .value("NOT_VALID", SensorDataType::NotValid)
      .value("IMAGE", SensorDataType::Image, "camera image streams")
      .value(
          "IMU",
          SensorDataType::Imu,
          "Inertial measurement unit (IMU) data streams, including accelerometer and gyroscope, note that magnetometer is a different stream")
      .value("GPS", SensorDataType::Gps, "Global positioning system (GPS) data streams")
      .value("WPS", SensorDataType::Wps, "Wifi beacon data streams")
      .value("AUDIO", SensorDataType::Audio, "Audio data streams")
      .value("BAROMETER", SensorDataType::Barometer, "Barometer data streams")
      .value("BLUETOOTH", SensorDataType::Bluetooth, "Bluetooth data streams")
      .value("MAGNETOMETER", SensorDataType::Magnetometer, "Magnetometer data streams")
      .export_values();

  // output sensorDataType string
  m.def(
      "get_sensor_data_type_name",
      [](const SensorDataType type) { return getName(type); },
      "converts the enum to readable string");
  m.def(
      "supports_host_time_domain",
      &supportsHostTimeDomain,
      py::arg("type"),
      "checks if host time domain is supported by a type. Note we encourage user to avoid using host time domains as arrival timestamps are inaccurate.");
  m.def(
      "has_calibration",
      &hasCalibration,
      py::arg("type"),
      "checks if calibration exists for a specific stream");
}

inline void declareImageDataRecord(py::module& m) {
  py::class_<vrs::utils::PixelFrame, std::shared_ptr<vrs::utils::PixelFrame>>(m, "PixelFrame")
      .def("get_buffer", &vrs::utils::PixelFrame::getBuffer, "Get image data buffer")
      .def("get_width", &vrs::utils::PixelFrame::getWidth, "Return number of columns in image")
      .def("get_height", &vrs::utils::PixelFrame::getHeight, "Return number of rows in image")
      .def(
          "normalize_frame",
          [](const std::shared_ptr<vrs::utils::PixelFrame>& self, bool grey16supported) {
            std::shared_ptr<vrs::utils::PixelFrame> outFrame;
            vrs::utils::PixelFrame::normalizeFrame(self, outFrame, grey16supported);
            return outFrame;
          },
          "Normalize an input frame if possible and as necessary");

  py::class_<ImageData>(m, "ImageData")
      .def(py::init<>())
      .def_readwrite(
          "pixel_frame",
          &ImageData::pixelFrame,
          py::return_value_policy::reference,
          "Returns PixelFrame representation of the image.")
      .def("get_width", &ImageData::getWidth, "Returns number of columns in image")
      .def("get_height", &ImageData::getHeight, "Returns number of rows in image")
      .def("is_valid", &ImageData::isValid, "Returns if image is empty")
      .def(
          "to_numpy_array",
          [](const ImageData& self) -> image::PyArrayVariant {
            checkAndThrow(self.isValid());
            return image::toPyArrayVariant(self.imageVariant().value());
          },
          "Converts to numpy array")
      .def(
          "at",
          [](const ImageData& self, int x, int y, int channel) -> image::PixelValueVariant {
            checkAndThrow(self.isValid());
            return at(self.imageVariant().value(), x, y, channel);
          },
          py::arg("x"),
          py::arg("y"),
          py::arg("channel") = 0,
          "Returns pixel value at (x, y, channel)");
  ;

  py::class_<ImageConfigRecord>(m, "ImageConfigRecord")
      .def(py::init<>())
      .def_readwrite("device_type", &ImageConfigRecord::deviceType, "type of the device")
      .def_readwrite(
          "device_version", &ImageConfigRecord::deviceVersion, "OS version on the device")
      .def_readwrite("device_serial", &ImageConfigRecord::deviceSerial, "serial of the device")
      .def_readwrite("camera_id", &ImageConfigRecord::cameraId, "ID of the camera, 0 to N")
      .def_readwrite("sensor_model", &ImageConfigRecord::sensorModel, "model of the camera sensor")
      .def_readwrite(
          "sensor_serial", &ImageConfigRecord::sensorSerial, "serial of the camera sensor")
      .def_readwrite(
          "nominal_rate_hz", &ImageConfigRecord::nominalRateHz, "number of frames per second")
      .def_readwrite("image_width", &ImageConfigRecord::imageWidth, "number of columns")
      .def_readwrite("image_height", &ImageConfigRecord::imageHeight, "number of rows")
      .def_readwrite("image_stride", &ImageConfigRecord::imageStride, "number of bytes per row")
      .def_readwrite("pixel_format", &ImageConfigRecord::pixelFormat, "format of the pixel")
      .def_readwrite(
          "exposure_duration_min",
          &ImageConfigRecord::exposureDurationMin,
          "longest exposure time allowed by the camera")
      .def_readwrite(
          "exposure_duration_max",
          &ImageConfigRecord::exposureDurationMax,
          "shortest exposure time allowed by the camera")
      .def_readwrite(
          "gain_min", &ImageConfigRecord::gainMin, "lowest gain setting allowed by the camera")
      .def_readwrite(
          "gain_max", &ImageConfigRecord::gainMax, "highest gain setting allowed by the camera")
      .def_readwrite("gamma_factor", &ImageConfigRecord::gammaFactor, "gamma correction factor")
      .def_readwrite("factory_calibration", &ImageConfigRecord::factoryCalibration)
      .def_readwrite("online_calibration", &ImageConfigRecord::onlineCalibration)
      .def_readwrite("description", &ImageConfigRecord::description);
  py::class_<ImageDataRecord>(m, "ImageDataRecord")
      .def(py::init<>())
      .def_readwrite("camera_id", &ImageDataRecord::cameraId, "ID of the camera, 0 to N")
      .def_readwrite("group_id", &ImageDataRecord::groupId)
      .def_readwrite("group_mask", &ImageDataRecord::groupMask)
      .def_readwrite("frame_number", &ImageDataRecord::frameNumber, "index of the frame")
      .def_readwrite(
          "exposure_duration", &ImageDataRecord::exposureDuration, "length of exposure time")
      .def_readwrite("gain", &ImageDataRecord::gain, "gain settings")
      .def_readwrite(
          "capture_timestamp_ns",
          &ImageDataRecord::captureTimestampNs,
          "capture time in device domain")
      .def_readwrite(
          "arrival_timestamp_ns",
          &ImageDataRecord::arrivalTimestampNs,
          "arrival time in device domain")
      .def_readwrite(
          "temperature", &ImageDataRecord::temperature, "temperature on the sensor, may be NAN");
}

inline void declareMotionDataRecord(py::module& m) {
  // motionData includes imu or mag
  py::class_<MotionConfigRecord>(m, "MotionConfigRecord")
      .def(py::init<>())
      .def_readwrite("stream_index", &MotionConfigRecord::streamIndex, "ID of the VRS stream")
      .def_readwrite("device_type", &MotionConfigRecord::deviceType, "type of the device")
      .def_readwrite("device_serial", &MotionConfigRecord::deviceSerial, "OS version on the device")
      .def_readwrite("device_id", &MotionConfigRecord::deviceId, "ID of the IMU, 0 to N")
      .def_readwrite("sensor_model", &MotionConfigRecord::sensorModel, "model of the IMU sensor")
      .def_readwrite(
          "nominal_rate_hz", &MotionConfigRecord::nominalRateHz, "number of frames per second")
      .def_readwrite(
          "has_accelerometer",
          &MotionConfigRecord::hasAccelerometer,
          "if the sensor contains a accelerometer")
      .def_readwrite(
          "has_gyroscope", &MotionConfigRecord::hasGyroscope, "if the sensor contains a gyroscope")
      .def_readwrite(
          "has_magnetometer",
          &MotionConfigRecord::hasMagnetometer,
          "if the sensor contains a magnetometer")
      .def_readwrite("factory_calibration", &MotionConfigRecord::factoryCalibration)
      .def_readwrite("online_calibration", &MotionConfigRecord::onlineCalibration)
      .def_readwrite("description", &MotionConfigRecord::description);

  py::class_<MotionData>(m, "MotionData")
      .def(py::init<>())
      .def_readwrite(
          "accel_valid", &MotionData::accelValid, "if the data contains accelerometer data")
      .def_readwrite("gyro_valid", &MotionData::gyroValid, "if the data contains gyroscope data")
      .def_readwrite("mag_valid", &MotionData::magValid, "if the data contains magnetometer data")
      .def_readwrite("temperature", &MotionData::temperature, "temperature in celsius degrees")
      .def_readwrite(
          "capture_timestamp_ns",
          &MotionData::captureTimestampNs,
          "capture time in device time domain")
      .def_readwrite(
          "arrival_timestamp_ns",
          &MotionData::arrivalTimestampNs,
          "arrival time in host time domain")
      .def_readwrite("accel_msec2", &MotionData::accelMSec2, "accelerometer data in m/sec2")
      .def_readwrite("gyro_radsec", &MotionData::gyroRadSec, "gyroscope data in rad/sec2")
      .def_readwrite("mag_tesla", &MotionData::magTesla, "magnetometer data in Tesla");
}

inline void declareGpsDataRecord(py::module& m) {
  // gpsData
  py::class_<GpsConfigRecord>(m, "GpsConfigRecord", "Gps sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &GpsConfigRecord::streamId, "ID of the VRS stream, 0 to N")
      .def_readwrite(
          "sample_rate_hz",
          &GpsConfigRecord::sampleRateHz,
          "the number of data collected per second");
  py::class_<GpsData>(
      m, "GpsData", "Gps data type, note that GPS sensor data are already rectified")
      .def(py::init<>())
      .def_readwrite(
          "capture_timestamp_ns", &GpsData::captureTimestampNs, "capture time in device domain")
      .def_readwrite("utc_time_ms", &GpsData::utcTimeMs, "capture time in UTC domain")
      .def_readwrite("provider", &GpsData::provider, "GPS provider")
      .def_readwrite(
          "latitude",
          &GpsData::latitude,
          "latitude of the position in Degrees Minutes Seconds (DMS)")
      .def_readwrite(
          "longitude",
          &GpsData::longitude,
          "longitude of the position in Degrees Minutes Seconds (DMS)")
      .def_readwrite("altitude", &GpsData::altitude, "altitude of the position")
      .def_readwrite("accuracy", &GpsData::accuracy)
      .def_readwrite("speed", &GpsData::speed)
      .def_readwrite("raw_data", &GpsData::rawData);
}

inline void declareWpsDataRecord(py::module& m) {
  // wpsData
  py::class_<WifiBeaconConfigRecord>(m, "WifiBeaconConfigRecord")
      .def(py::init<>())
      .def_readwrite("stream_id", &WifiBeaconConfigRecord::streamId, "ID of the VRS stream");
  py::class_<WifiBeaconData>(m, "WifiBeaconData")
      .def(py::init<>())
      .def_readwrite(
          "system_timestamp_ns",
          &WifiBeaconData::systemTimestampNs,
          "capture time of the data in host domain")
      .def_readwrite(
          "board_timestamp_ns",
          &WifiBeaconData::boardTimestampNs,
          "capture time of the data in device domain")
      .def_readwrite(
          "board_scan_request_start_timestamp_ns",
          &WifiBeaconData::boardScanRequestStartTimestampNs,
          "the device time the request starts")
      .def_readwrite(
          "board_scan_request_complete_timestamp_ns",
          &WifiBeaconData::boardScanRequestCompleteTimestampNs,
          "the device time the request starts")
      .def_readwrite("ssid", &WifiBeaconData::ssid, "id of the Wi-Fi source")
      .def_readwrite("bssid_mac", &WifiBeaconData::bssidMac, "mac id of the Wi-Fi source")
      .def_readwrite("rssi", &WifiBeaconData::rssi, "sensor readout in dBm")
      .def_readwrite("freq_mhz", &WifiBeaconData::freqMhz, "frequency of the data")
      .def_readwrite("rssi_per_antenna", &WifiBeaconData::rssiPerAntenna);
}

inline void declareAudioDataRecord(py::module& m) {
  // audioData
  py::class_<AudioData>(m, "AudioData", "Audio sensor data type: the audio value")
      .def(py::init<>())
      .def_readwrite("data", &AudioData::data, "raw data, length = nChannels * nSamples");
  py::class_<AudioConfig>(m, "AudioConfig", "Audio sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &AudioConfig::streamId, "ID of the VRS stream")
      .def_readwrite("num_channels", &AudioConfig::numChannels, "number of microphones used")
      .def_readwrite("sample_rate", &AudioConfig::sampleRate, "number of timestamps per second")
      .def_readwrite("sample_format", &AudioConfig::sampleFormat, "format of the Audio data");
  py::class_<AudioDataRecord>(m, "AudioDataRecord", "Audio meta data")
      .def(py::init<>())
      .def_readwrite(
          "capture_timestamps_ns",
          &AudioDataRecord::captureTimestampsNs,
          "timestamps in device time domain")
      .def_readwrite("audio_muted", &AudioDataRecord::audioMuted, "set 1 for muted, 0 otherwise");
}

inline void declareBluetoothDataRecord(py::module& m) {
  // bluetoothData
  py::class_<BluetoothBeaconConfigRecord>(
      m, "BluetoothBeaconConfigRecord", "Bluetooth sensor configuration type")
      .def(py::init<>())
      .def_readwrite("streamId", &BluetoothBeaconConfigRecord::streamId, "ID of the VRS stream")
      .def_readwrite(
          "sample_rate_hz",
          &BluetoothBeaconConfigRecord::sampleRateHz,
          "number of times the device request data from bluetooth");
  py::class_<BluetoothBeaconData>(m, "BluetoothBeaconData")
      .def(py::init<>())
      .def_readwrite(
          "system_timestamp_ns",
          &BluetoothBeaconData::systemTimestampNs,
          "capture time of the data in host domain")
      .def_readwrite(
          "board_timestamp_ns",
          &BluetoothBeaconData::boardTimestampNs,
          "capture time of the data in device domain")
      .def_readwrite(
          "board_scan_request_start_timestamp_ns",
          &BluetoothBeaconData::boardScanRequestStartTimestampNs,
          "the device time the request starts")
      .def_readwrite(
          "board_scan_request_complete_timestamp_ns",
          &BluetoothBeaconData::boardScanRequestCompleteTimestampNs,
          "the device time the request starts")
      .def_readwrite("unique_id", &BluetoothBeaconData::uniqueId, "id of the bluetooth source")
      .def_readwrite(
          "tx_power",
          &BluetoothBeaconData::txPower,
          "the range of the bluetooth signal to transmit the beacon")
      .def_readwrite("rssi", &BluetoothBeaconData::rssi, "bluetooth data readout in dBm")
      .def_readwrite("freq_mhz", &BluetoothBeaconData::freqMhz, "frequency of the data");
}

inline void declareBarometerDataRecord(py::module& m) {
  // barometerData
  py::class_<BarometerConfigRecord>(
      m, "BarometerConfigRecord", "Barometer sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &BarometerConfigRecord::streamId, "ID of the VRS stream")
      .def_readwrite("sensor_model_name", &BarometerConfigRecord::sensorModelName, "sensor model")
      .def_readwrite(
          "sample_rate", &BarometerConfigRecord::sampleRate, "number of samples per second");
  py::class_<BarometerData>(m, "BarometerData")
      .def(py::init<>())
      .def_readwrite(
          "capture_timestamp_ns",
          &BarometerData::captureTimestampNs,
          "the timestamp when the data is captured")
      .def_readwrite(
          "temperature",
          &BarometerData::temperature,
          "temperature of the sensor in degrees Celsius")
      .def_readwrite(
          "pressure", &BarometerData::pressure, "raw sensor readout of pressure in Pascal");
}

inline void declareSensorConfiguration(py::module& m) {
  py::class_<SensorConfiguration>(
      m,
      "SensorConfiguration",
      "Configuration of a sensor stream, such as stream id, nominal frame rate")
      .def(
          py::init<const SensorConfiguration::SensorConfigurationVariant&, const SensorDataType&>())
      .def(
          "sensor_data_type",
          &SensorConfiguration::sensorDataType,
          "Returns the type of sensor data ")
      .def(
          "image_configuration",
          &SensorConfiguration::imageConfiguration,
          "Returns the sensor configuration as ImageConfigRecord")
      .def(
          "motion_configuration",
          &SensorConfiguration::imuConfiguration,
          "Returns the sensor configuration as MotionConfigRecord")
      .def(
          "gps_configuration",
          &SensorConfiguration::gpsConfiguration,
          "Returns the sensor configuration as GpsConfigRecord")
      .def(
          "wps_configuration",
          &SensorConfiguration::wpsConfiguration,
          "Returns the sensor configuration as WifiBeaconConfigRecord")
      .def(
          "audio_configuration",
          &SensorConfiguration::audioConfiguration,
          "Returns the sensor configuration as AudioConfig")
      .def(
          "barometer_configuration",
          &SensorConfiguration::barometerConfiguration,
          "Returns the sensor configuration as BarometerConfigRecord")
      .def(
          "bluetooth_configuration",
          &SensorConfiguration::bluetoothConfiguration,
          "Returns the sensor configuration as Bluetooth")
      .def(
          "magnetometer_configuration",
          &SensorConfiguration::magnetometerConfiguration,
          "Returns the sensor configuration as MotionConfigRecord")
      .def(
          "get_nominal_rate_hz",
          &SensorConfiguration::getNominalRateHz,
          "Returns the nominal frame rate of the sensor");
}

inline void declareSensorData(py::module& m) {
  declareImageDataRecord(m);
  declareMotionDataRecord(m);
  declareGpsDataRecord(m);
  declareWpsDataRecord(m);
  declareAudioDataRecord(m);
  declareBluetoothDataRecord(m);
  declareBarometerDataRecord(m);

  declareSensorConfiguration(m);

  // SensorData Class
  py::class_<SensorData>(m, "SensorData")
      .def(py::init<
           const vrs::StreamId&,
           const SensorData::SensorDataVariant&,
           const SensorDataType&,
           const int64_t,
           const int64_t>())
      .def("stream_id", &SensorData::streamId)
      .def("sensor_data_type", &SensorData::sensorDataType)
      .def("image_data_and_record", &SensorData::imageDataAndRecord)
      .def("imu_data", &SensorData::imuData)
      .def("gps_data", &SensorData::gpsData)
      .def("wps_data", &SensorData::wpsData)
      .def("audio_data_and_record", &SensorData::audioDataAndRecord)
      .def("bluetooth_data", &SensorData::bluetoothData)
      .def("barometer_data", &SensorData::barometerData)
      .def("magnetometer_data", &SensorData::magnetometerData)
      .def("get_time_ns", &SensorData::getTimeNs, py::arg("time_domain"));
}
} // namespace

void exportSensorData(py::module& m) {
  // For submodule documentation, see: projectaria_tools/projectaria_tools/core/sensor_data.py

  declareTimeEnums(m);
  declareSensorDataType(m);
  declareSensorData(m);
}

} // namespace projectaria::tools::data_provider
