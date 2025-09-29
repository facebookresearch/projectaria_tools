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
#include <data_provider/data_types/FrontendOutput.h>

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
      .value(
          "TIC_SYNC",
          TimeDomain::TicSync,
          "capture in TimeSync server's timedomain where the server can be an Aria device, accurate across devices in a <b>multi-device</b> data capture")
      .value(
          "SUBGHZ",
          TimeDomain::SubGhz,
          "capture in SubGhz timedomain, accurate across devices in a <b>multi-device</b> data")
      .value("UTC", TimeDomain::Utc, "capture in UTC timedomain, captured at 1 sample per minute")
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

  py::enum_<TimeSyncMode>(m, "TimeSyncMode")
      .value("TIME_CODE", TimeSyncMode::TIMECODE, "TIMECODE mode")
      .value("TIC_SYNC", TimeSyncMode::TIC_SYNC, "TIC_SYNC mode")
      .value("SUBGHZ", TimeSyncMode::SUBGHZ, "SUBGHZ mode")
      .value("UTC", TimeSyncMode::UTC, "UTC mode")
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
      .value("PPG", SensorDataType::Ppg, "Photoplethysmogram (PPG) data streams")
      .value("ALS", SensorDataType::Als, "Ambient Light Sensor (ALS) data streams")
      .value("EYE_GAZE", SensorDataType::EyeGaze, "EyeGaze data streams")
      .value("HAND_POSE", SensorDataType::HandPose, "HandPose data streams")
      .value("VIO_HIGH_FREQ", SensorDataType::VioHighFreq, "Vio high frequency data streams")
      .value("VIO", SensorDataType::Vio, "Vio data streams")
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
      .def(py::init<>())
      .def(
          py::init([](uint8_t imageFormat,
                      uint8_t pixelFormat,
                      uint32_t width,
                      uint32_t height,
                      uint32_t stride,
                      uint32_t stride2,
                      std::string codecName,
                      uint8_t codecQuality,
                      double keyFrameTimestamp,
                      uint32_t keyFrameIndex,
                      std::vector<uint8_t>&& frameBytes) {
            if (imageFormat >= static_cast<uint8_t>(vrs::ImageFormat::COUNT)) {
              throw std::invalid_argument(
                  "Invalid ImageFormat value: " + std::to_string(imageFormat));
            }
            if (pixelFormat >= static_cast<uint8_t>(vrs::PixelFormat::COUNT)) {
              throw std::invalid_argument(
                  "Invalid PixelFormat value: " + std::to_string(pixelFormat));
            }

            auto spec = vrs::ImageContentBlockSpec(
                static_cast<vrs::ImageFormat>(imageFormat),
                static_cast<vrs::PixelFormat>(pixelFormat),
                width,
                height,
                stride,
                stride2,
                std::move(codecName),
                codecQuality,
                keyFrameTimestamp,
                keyFrameIndex);
            return std::make_shared<vrs::utils::PixelFrame>(std::move(spec), std::move(frameBytes));
          }),
          "Initialize a PixelFrame with the complete information.")
      .def("swap", &vrs::utils::PixelFrame::swap, "Swap the image data")
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
      .def(
          "get_pixel_format",
          [](const ImageData& self) -> uint8_t {
            return static_cast<uint8_t>(self.getPixelFormat());
          },
          "Returns the format of the pixel")
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
          "the number of data collected per second")
      .def_readwrite(
          "provider", &GpsConfigRecord::provider, "Provider of the GPS data, e.g. 'app', 'gps'.");
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
      .def_readwrite("verticalAccuracy", &GpsData::verticalAccuracy)
      .def_readwrite("speed", &GpsData::speed)
      .def_readwrite("raw_data", &GpsData::rawData)
      .def_readwrite(
          "raw_measurements",
          &GpsData::rawMeasurements,
          "satellite measurements (such as pseudo-range)")
      .def_readwrite(
          "navigation_messages",
          &GpsData::navigationMessages,
          "message (which includes Ephemeris data) stored in text format")
      .def_readwrite(
          "constellations_enabled",
          &GpsData::constellationsEnabled,
          "a list of constellations and frequency bands used for this GNSS result");
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
      .def_readwrite("data", &AudioData::data, "raw data, length = nChannels * nSamples")
      .def_readwrite("max_amplitude", &AudioData::maxAmplitude, "max amplitude");
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

inline void declarePpgDataRecord(py::module& m) {
  py::class_<PpgConfiguration>(m, "PpgConfiguration", "PPG sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &PpgConfiguration::streamId, "ID of the VRS stream")
      .def_readwrite("sensor_model", &PpgConfiguration::sensorModel, "sensor model name")
      .def_readwrite("device_id", &PpgConfiguration::deviceId, "device ID for ppg sensor")
      .def_readwrite(
          "nominal_rate_hz", &PpgConfiguration::nominalRateHz, "number of frames per second")
      .def_readwrite(
          "description",
          &PpgConfiguration::description,
          "description of the PPG sensor"); // TODO: update this after checking with os team.

  py::class_<PpgData>(m, "PpgData", "PPG data type")
      .def(py::init<>())
      .def_readwrite(
          "capture_timestamp_ns",
          &PpgData::captureTimestampNs,
          "Timestamp of capturing this sample")
      .def_readwrite(
          "value",
          &PpgData::value,
          "Raw sensor light exposure measurement, there is no unit for this value")
      .def_readwrite("led_current_ma", &PpgData::ledCurrentMa, "LED current in mA")
      .def_readwrite(
          "integration_time_us", &PpgData::integrationTimeUs, "PPG integration time in us");
}

inline void declareAlsDataRecord(py::module& m) {
  py::class_<AlsConfiguration>(m, "AlsConfiguration", "ALS sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &AlsConfiguration::streamId, "ID of the VRS stream")
      .def_readwrite("device_id", &AlsConfiguration::deviceId, "device ID for ALS sensor")
      .def_readwrite(
          "nominal_rate_hz", &AlsConfiguration::nominalRateHz, "number of frames per second")
      .def_readwrite("sensor_model", &AlsConfiguration::sensorModel, "sensor model name");

  py::class_<AlsData>(m, "AlsData", "ALS data type")
      .def(py::init<>())
      .def_readwrite(
          "capture_timestamp_ns",
          &AlsData::captureTimestampNs,
          "Timestamp of capturing this sample")
      .def_readwrite(
          "red_channel_normalized", &AlsData::redChannelNormalized, "Normalized red channel value")
      .def_readwrite(
          "green_channel_normalized",
          &AlsData::greenChannelNormalized,
          "Normalized green channel value")
      .def_readwrite(
          "blue_channel_normalized",
          &AlsData::blueChannelNormalized,
          "Normalized blue channel value")
      .def_readwrite(
          "uv_channel_normalized", &AlsData::uvChannelNormalized, "Normalized UV channel value")
      .def_readwrite(
          "ir_channel_normalized", &AlsData::irChannelNormalized, "Normalized IR channel value")
      .def_readwrite(
          "clear_channel_normalized",
          &AlsData::clearChannelNormalized,
          "Normalized clear channel value")
      .def_readwrite(
          "uv_flux_watt_per_square_meter",
          &AlsData::uvFluxWattPerSquareMeter,
          "UV flux in watts per square meter")
      .def_readwrite(
          "ir_flux_watt_per_square_meter",
          &AlsData::irFluxWattPerSquareMeter,
          "IR flux in watts per square meter")
      .def_readwrite(
          "clear_flux_watt_per_square_meter",
          &AlsData::clearFluxWattPerSquareMeter,
          "Clear flux in watts per square meter")
      .def_readwrite("gain_red", &AlsData::gainRed, "Red channel gain")
      .def_readwrite("gain_green", &AlsData::gainGreen, "Green channel gain")
      .def_readwrite("gain_blue", &AlsData::gainBlue, "Blue channel gain")
      .def_readwrite("gain_uv", &AlsData::gainUv, "UV channel gain")
      .def_readwrite("gain_ir", &AlsData::gainIr, "IR channel gain")
      .def_readwrite("gain_clear", &AlsData::gainClear, "Clear channel gain")
      .def_readwrite("exposure_time_us", &AlsData::exposureTimeUs, "Exposure time in microseconds")
      .def_readwrite("cct", &AlsData::cct, "Correlated color temperature")
      .def_readwrite("lux", &AlsData::lux, "Illuminance in lux");
}

inline void declareTemperatureDataRecord(py::module& m) {
  py::class_<TemperatureConfiguration>(
      m, "TemperatureConfiguration", "Temperature sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &TemperatureConfiguration::streamId, "ID of the VRS stream")
      .def_readwrite("sensor_model", &TemperatureConfiguration::sensorModel, "sensor model name")
      .def_readwrite(
          "device_id", &TemperatureConfiguration::deviceId, "device ID for temperature sensor")
      .def_readwrite(
          "nominal_rate_hz",
          &TemperatureConfiguration::nominalRateHz,
          "number of frames per second");

  py::class_<TemperatureData>(m, "TemperatureData", "Temperature data type")
      .def(py::init<>())
      .def_readwrite(
          "capture_timestamp_ns",
          &TemperatureData::captureTimestampNs,
          "Timestamp of capturing this sample")
      .def_readwrite(
          "temperature_celsius",
          &TemperatureData::temperatureCelsius,
          "Temperature measurement in Celsius")
      .def_readwrite("sensor_name", &TemperatureData::sensorName, "Name of the temperature sensor");
}

// Pybinding for classes in `FrontendTypes.h`
inline void declareFrontendTypes(py::module& m) {
  // Vio status related enums
  py::enum_<VioStatus>(m, "VioStatus")
      .value("VALID", VioStatus::VALID)
      .value("FILTER_NOT_INITIALIZED", VioStatus::FILTER_NOT_INITIALIZED)
      .value("INVALID", VioStatus::INVALID)
      .export_values();
  py::enum_<TrackingQuality>(m, "TrackingQuality")
      .value("UNKNOWN", TrackingQuality::UNKNOWN)
      .value("GOOD", TrackingQuality::GOOD)
      .value("BAD", TrackingQuality::BAD)
      .value("UNRECOVERABLE", TrackingQuality::UNRECOVERABLE)
      .export_values();
  py::enum_<VisualTrackingQuality>(m, "VisualTrackingQuality")
      .value("UNKNOWN", VisualTrackingQuality::UNKNOWN)
      .value("BAD", VisualTrackingQuality::BAD)
      .value("GOOD", VisualTrackingQuality::GOOD)
      .export_values();

  // FrontendOutput::online calibration for cameras
  py::class_<ProjectionModelParameters<float>>(m, "ProjectionModelParametersFloat")
      .def(py::init<>())
      .def_readwrite(
          "type", &ProjectionModelParameters<float>::type, "The type of projection model.")
      .def_readwrite(
          "intrinsics",
          &ProjectionModelParameters<float>::intrinsics,
          "Projection intrinsics within the context of VIO.")
      .def_readwrite(
          "readout_time_sec",
          &ProjectionModelParameters<float>::readoutTimeSec,
          "The readout time of the image, in seconds.")
      .def_readwrite(
          "max_radius_squared",
          &ProjectionModelParameters<float>::maxRadiusSquared,
          "Squared radius of the circle that contains valid pixels.")
      .def_readwrite(
          "image_size", &ProjectionModelParameters<float>::imageSize, "Image size in pixels.");

  // FrontendOutput::online calibration for imus
  py::class_<ImuMeasurementModelParameters<float>>(m, "ImuMeasurementModelParametersFloat")
      .def(py::init<>())
      .def_readwrite(
          "gyro_scale_vec",
          &ImuMeasurementModelParameters<float>::gyroScaleVec,
          "The scale of each of the gyro axes.")
      .def_readwrite(
          "accel_scale_vec",
          &ImuMeasurementModelParameters<float>::accelScaleVec,
          "The scale of each of the accel axes.")
      .def_readwrite(
          "accel_bias_m_sec2",
          &ImuMeasurementModelParameters<float>::accelBiasMSec2,
          "Additive bias, m/sec^2.")
      .def_readwrite(
          "gyro_bias_rad_sec",
          &ImuMeasurementModelParameters<float>::gyroBiasRadSec,
          "Additive bias, rad/sec.")
      .def_readwrite(
          "accel_nonorth",
          &ImuMeasurementModelParameters<float>::accelNonorth,
          "Nonorthogonality of accel axes.")
      .def_readwrite(
          "gyro_nonorth",
          &ImuMeasurementModelParameters<float>::gyroNonorth,
          "Nonorthogonality of gyro axes.")
      .def_readwrite(
          "gyro_g_sensitivity_rad_sec_per_m_sec2",
          &ImuMeasurementModelParameters<float>::gyroGSensitivityRadSecPerMSec2,
          "Gyro g sensitivity.")
      .def_readwrite(
          "dt_reference_accel_sec",
          &ImuMeasurementModelParameters<float>::dtReferenceAccelSec,
          "Time offset for accelerometer.")
      .def_readwrite(
          "dt_reference_gyro_sec",
          &ImuMeasurementModelParameters<float>::dtReferenceGyroSec,
          "Time offset for gyroscope.")
      .def_readwrite(
          "t_imu_body_imu",
          &ImuMeasurementModelParameters<float>::T_Imu_BodyImu,
          "IMU from BodyImu transform.")
      .def_readwrite(
          "nominal_sampling_period_sec",
          &ImuMeasurementModelParameters<float>::nominalSamplingPeriodSec,
          "Nominal sample period, seconds.")
      .def_readwrite(
          "accel_saturation_threshold_m_sec2",
          &ImuMeasurementModelParameters<float>::accelSaturationThresholdMSec2,
          "Saturation threshold for the accel.")
      .def_readwrite(
          "gyro_saturation_threshold_rad_sec",
          &ImuMeasurementModelParameters<float>::gyroSaturationThresholdRadSec,
          "Saturation threshold for the gyro.")
      .def("reset", &ImuMeasurementModelParameters<float>::reset, "Reset to default values.");

  // FrontendOutput::online calibration state
  py::class_<OnlineCalibState>(m, "OnlineCalibState")
      .def(py::init<>())
      .def_readwrite(
          "cam_parameters",
          &OnlineCalibState::camParameters,
          "Camera intrinsic, camera model parameters.")
      .def_readwrite("t_cam_body_imu", &OnlineCalibState::T_Cam_BodyImu, "T_camera_imu extrinsics.")
      .def_readwrite(
          "dt_ref_cam",
          &OnlineCalibState::dt_Ref_Cam,
          "Time offset between the reference and the camera in nanoseconds.")
      .def_readwrite(
          "imu_model_parameters", &OnlineCalibState::imuModelParameters, "IMU state parameters.")
      .def("reset", &OnlineCalibState::reset, "Reset the calibration state.")
      .def("num_cameras", &OnlineCalibState::numCameras, "Get the number of cameras.");
}

// Pybinding for FrontendOutput
inline void declareFrontendOutput(py::module& m) {
  py::class_<FrontendOutput>(m, "FrontendOutput", R"docdelimiter(
        FrontendOutput class holds the output data from the on-device VIO stream.
        It includes session identifiers, timestamps, status, and various pose and velocity data.
    )docdelimiter")
      .def(py::init<>())
      .def_property(
          "frontend_session_uid",
          [](FrontendOutput& self) { return self.frontendSessionUid.toString(); },
          [](FrontendOutput& self, const std::string& uidString) {
            self.frontendSessionUid = FrontendSessionUid::createFromString(uidString).first;
          },
          "Uid tag for the Frontend session. Regenerate on Frontend reset.")
      .def_readwrite("frame_id", &FrontendOutput::frameID, "ID of this frame set.")
      .def_readwrite(
          "capture_timestamp_ns",
          &FrontendOutput::captureTimestampNs,
          "Center capture time of the frame set.")
      .def_readwrite(
          "unix_timestamp_ns", &FrontendOutput::unixTimestampNs, "Unix timestamp of the frame.")
      .def_readwrite(
          "status",
          &FrontendOutput::status,
          "Status of the Frontend, whether there is pose result available.")
      .def_readwrite(
          "pose_quality",
          &FrontendOutput::poseQuality,
          "VIO tracking quality: quality of the pose results.")
      .def_readwrite(
          "visual_tracking_quality",
          &FrontendOutput::visualTrackingQuality,
          "Visual-only tracking quality.")
      .def_readwrite("online_calib", &FrontendOutput::onlineCalib, "Online calibration estimate.")
      .def_readwrite("camera_serials", &FrontendOutput::cameraSerials, "Camera serial numbers.")
      .def_readwrite(
          "gravity_in_odometry",
          &FrontendOutput::gravityInOdometry,
          "Gravity vector in odometry frame.")
      .def_readwrite(
          "transform_odometry_bodyimu",
          &FrontendOutput::T_Odometry_BodyImu,
          "This frame's pose in odometry reference.")
      .def_readwrite(
          "transform_bodyimu_device",
          &FrontendOutput::T_BodyImu_Device,
          "Pose of the device frame.")
      .def_readwrite(
          "linear_velocity_in_odometry",
          &FrontendOutput::linearVelocityInOdometry,
          "This frame's linear velocity in odometry reference.")
      .def_readwrite(
          "angular_velocity_in_bodyimu",
          &FrontendOutput::angularVelocityInBodyImu,
          "This frame's angular velocity in the body IMU frame.");
}

inline void declareOnDeviceMpConfiguration(py::module& m) {
  py::class_<EyeGazeConfiguration>(m, "EyeGazeConfiguration", "Eye gaze sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &EyeGazeConfiguration::streamId, "ID of the VRS stream")
      .def_readwrite(
          "nominal_rate_hz", &EyeGazeConfiguration::nominalRateHz, "number of frames per second")
      .def_readwrite(
          "user_calibrated",
          &EyeGazeConfiguration::userCalibrated,
          "indicates if the eye tracking has been calibrated before the recording")
      .def_readwrite(
          "user_calibration_error",
          &EyeGazeConfiguration::userCalibrationError,
          "indicates the accuracy of the eye tracking calibration. Lower is better");

  py::class_<HandPoseConfiguration>(
      m, "HandPoseConfiguration", "Hand pose sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &HandPoseConfiguration::streamId, "ID of the VRS stream")
      .def_readwrite(
          "nominal_rate_hz", &HandPoseConfiguration::nominalRateHz, "number of frames per second")
      .def_readwrite(
          "is_wrist_palm_only",
          &HandPoseConfiguration::isWristPalmOnly,
          "Whether the result is limited to wrist and palm joints")
      .def_readwrite(
          "user_profile",
          &HandPoseConfiguration::userProfile,
          "User profile associated with the hand pose configuration");

  py::class_<VioConfiguration>(m, "VioConfiguration", "Vio sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &VioConfiguration::streamId, "ID of the VRS stream")
      .def_readwrite(
          "nominal_rate_hz", &VioConfiguration::nominalRateHz, "number of frames per second")
      .def_readwrite(
          "message_version",
          &VioConfiguration::messageVersion,
          "Payload version to indicate the VIO message serialization format. Formatted in string as Major.minor");

  py::class_<VioHighFreqConfiguration>(
      m, "VioHighFreqConfiguration", "Vio high frequency sensor configuration type")
      .def(py::init<>())
      .def_readwrite("stream_id", &VioHighFreqConfiguration::streamId, "ID of the VRS stream")
      .def_readwrite(
          "nominal_rate_hz",
          &VioHighFreqConfiguration::nominalRateHz,
          "number of frames per second")
      .def_readwrite(
          "message_version",
          &VioHighFreqConfiguration::messageVersion,
          "Payload version to indicate the VIO message serialization format. Formatted in string as Major.minor");
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
          "ppg_configuration",
          &SensorConfiguration::ppgConfiguration,
          "Returns the sensor configuration as PpgConfiguration")
      .def(
          "als_configuration",
          &SensorConfiguration::alsConfiguration,
          "Returns the sensor configuration as AlsConfiguration")
      .def(
          "temperature_configuration",
          &SensorConfiguration::temperatureConfiguration,
          "Returns the sensor configuration as TemperatureConfiguration")
      .def("eye_gaze_configuration", &SensorConfiguration::eyeGazeConfiguration)
      .def("hand_pose_configuration", &SensorConfiguration::handPoseConfiguration)
      .def("vio_configuration", &SensorConfiguration::vioConfiguration)
      .def("vio_high_freq_configuration", &SensorConfiguration::vioHighFreqConfiguration)
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
  declarePpgDataRecord(m);
  declareAlsDataRecord(m);
  declareTemperatureDataRecord(m);
  declareFrontendTypes(m);
  declareFrontendOutput(m);
  declareOnDeviceMpConfiguration(m);
  declareSensorConfiguration(m);

  // SensorData Class
  py::class_<SensorData>(m, "SensorData")
      .def(py::init<
           const vrs::StreamId&,
           const SensorData::SensorDataVariant&,
           const SensorDataType&,
           const int64_t,
           const std::map<TimeSyncMode, int64_t>&>())
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
      .def("ppg_data", &SensorData::ppgData)
      .def("als_data", &SensorData::alsData)
      .def("temperature_data", &SensorData::temperatureData)
      .def("vio_high_freq_data", &SensorData::vioHighFreqData)
      .def("eye_gaze_data", &SensorData::eyeGazeData)
      .def("hand_pose_data", &SensorData::handPoseData)
      .def("vio_data", &SensorData::vioData)
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
