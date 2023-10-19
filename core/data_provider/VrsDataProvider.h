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

#include <mutex>
#include <optional>
#include <string>

#include <calibration/DeviceCalibration.h>
#include <data_provider/RecordReaderInterface.h>
#include <data_provider/SensorConfiguration.h>
#include <data_provider/SensorDataSequence.h>
#include <data_provider/StreamIdConfigurationMapper.h>
#include <data_provider/StreamIdLabelMapper.h>
#include <data_provider/TimeCodeMapper.h>
#include <data_provider/TimestampIndexMapper.h>

namespace projectaria::tools::data_provider {
class VrsDataProvider;

/**
 * @brief Factory class to create a VrsDataProvider class.
 * @param vrsFilename Single vrs file to read from.
 */
std::shared_ptr<VrsDataProvider> createVrsDataProvider(const std::string& vrsFilename);

/**
 * @brief Given a vrs file that contains data collected from Aria devices, createVrsDataProvider
 * will create and return a new VrsDataProvider object. A VrsDataProvider object can be used to
 * access sensor data from a vrs file including image data, IMU data, calibration data and more.
 */
class VrsDataProvider {
 public:
  /**
   * @brief Get all available streams from the vrs file.
   * @return A set of streamIds.
   */
  const std::set<vrs::StreamId> getAllStreams() const;

  /**
   * @brief Get SensorDataType from streamId.
   * @param streamId The ID of a sensor's stream.
   * @return An entry of SensorDataType assigned for streamId, if stream with this ID exists in vrs,
   * otherwise returns SensorDataType::NotValid.
   */
  SensorDataType getSensorDataType(const vrs::StreamId& streamId) const;

  /**
   * @brief Get label from streamId as oppose to getStreamIdFromLabel().
   * @param streamId The ID of a sensor's stream.
   * @return Label corresponds to the streamId, return nullopt if streamId cannot be found
   */
  std::optional<std::string> getLabelFromStreamId(const vrs::StreamId& streamId) const;

  /**
   * @brief Get streamId from label as oppose to getLabelFromStreamId().
   * @param label Label of a sensor.
   * @return StreamId corresponds to the sensor label, return nullopt if sensor label cannot be
   * found.
   */
  std::optional<vrs::StreamId> getStreamIdFromLabel(const std::string& label) const;

  /**
   * @brief Check, if a stream with provided ID is active.
   * @param streamId The ID of a sensor's stream.
   * @return True, if streamId is activated and its data can be accessed.
   */
  bool checkStreamIsActive(const vrs::StreamId& streamId) const;

  /**
   * @brief Checks, if a stream with provided ID is of expected type.
   * @param streamId The ID of a sensor's stream.
   * @param type The type of Sensor's data as defined by SensorDataType.
   * @return True, if provided SensorDataType matches actual SensorDataType set for the stream;
   * false otherwise.
   */
  bool checkStreamIsType(const vrs::StreamId& streamId, SensorDataType type) const;

  /**
   * @brief Get calibration of the device.
   * @return The Optional DeviceCalibration, that contains calibration of all sensors.
   */
  std::optional<calibration::DeviceCalibration> getDeviceCalibration() const;
  /**
   * @brief Get calibration of a sensor from the device
   * @param streamId The ID of a sensor's stream.
   * @return The optional calibration of the particular sensor.
   */
  std::optional<calibration::SensorCalibration> getSensorCalibration(
      const vrs::StreamId& streamId) const;

  /**
   * @brief Get configuration of a specific stream.
   * @param streamId The ID of a sensor's stream.
   * @return SensorConfiguration.
   */
  SensorConfiguration getConfiguration(const vrs::StreamId& streamId) const;
  /**
   * @brief Gets the nominal frame rate in Hz of a specific stream.
   * @param streamId The ID of a sensor's stream.
   * @return Nominal rate in Hz.
   */
  double getNominalRateHz(const vrs::StreamId& streamId) const;

  /**
   * @brief Delivers data from all sensors in the entire vrs file sorted by TimeDomain::DeviceTime.
   * @return SensorDataSequence object that contains .begin() .end() iterator that iterate through
   * all sensor data.
   */
  SensorDataSequence deliverQueuedSensorData();

  /**
   * @brief Delivers data from vrs file with options sorted by TimeDomain::DeviceTime.
   * @param options Options that specify truncated start/end time of the iterator, a subset of
   * streamIds (not all streamIds) with sample rates.
   * @return SensorDataSequence object that contains .begin() .end() iterator that iterate through
   * all sensor data specified by options.
   */
  SensorDataSequence deliverQueuedSensorData(DeliverQueuedOptions options);

  //////////////////////Pybind11 utilities//////////////////////
  /**
   * @brief Create iterator pair as a private variable for pybind11.
   * @return A pair of .begin() .end() iterators.
   */
  std::pair<SensorDataIterator, SensorDataIterator> makeIterator() {
    auto dataSequence = deliverQueuedSensorData();
    dataIterPair_.first = dataSequence.begin();
    dataIterPair_.second = dataSequence.end();
    return dataIterPair_;
  }

  /**
   * @brief Create iterator pair as private variable for pybind11 with options.
   * @param options Options that specify truncated start/end time of the iterator, a subset of
   * streamIds (not all streamIds) with sample rates.
   * @return A pair of .begin() .end() iterators.
   */
  std::pair<SensorDataIterator, SensorDataIterator> makeIterator(DeliverQueuedOptions options) {
    auto dataSequence = deliverQueuedSensorData(options);
    dataIterPair_.first = dataSequence.begin();
    dataIterPair_.second = dataSequence.end();
    return dataIterPair_;
  }
  //////////////////////////////////////////////////////////////

  /**
   * @brief Default options that delivers all sensor data in vrs from start to end in
   * TimeDomain::DeviceTime.
   * @return DeliverQueuedOptions.
   */
  DeliverQueuedOptions getDefaultDeliverQueuedOptions() const;

  /**
   * @brief Return number of collected sensor data of a stream.
   * @param streamId The ID of a sensor's stream.
   * @return Number of data in a stream.
   */
  size_t getNumData(const vrs::StreamId& streamId) const;

  /**
   * @brief Return the N-th data of a stream, return SensorData of kNotValid if out of range. see
   * SensorData for more details on how sensor data are represented.
   * @param streamId StreamId of the sensor stream.
   * @param index Index in range of 0 - getNumData(streamId). Note this index is not the globalIndex
   * of a data in the entire vrs file.
   * @return SensorData. If no data is available, SensorData.sensorDataType() will be
   * SensorDataType::NotValid.
   */
  SensorData getSensorDataByIndex(const vrs::StreamId& streamId, const int index);

  /**
   * @brief Check if a stream contains timestamp of a specific time domain specifically, Audio,
   * Barometer, and GPS data does not have host timestamps. if the vrs does not contain a timesync
   * stream with timecode mode, then timecode is not supported.
   * @param streamId StreamId of the sensor stream.
   * @param timeDomain An enum class TimeDomain.
   * @return True if timeDomain is supported for this streamId.
   */
  bool supportsTimeDomain(const vrs::StreamId& streamId, const TimeDomain& timeDomain) const;
  /**
   * @brief Get all timestamps in nanoseconds as a vector for streamId of a particular timeDomain.
   * @param streamId StreamId of the sensor stream.
   * @param timeDomain An enum class TimeDomain.
   * @return Vector of timestamps in nanoseconds.
   */
  std::vector<int64_t> getTimestampsNs(const vrs::StreamId& streamId, const TimeDomain& timeDomain);

  /**
   * @brief Get first timestamp in nanoseconds of a streamId at a particular timeDomain.
   * @param streamId StreamId of the sensor stream.
   * @param timeDomain An enum class TimeDomain. Default to be DeviceTime.
   * @return Timestamp in nanosecond.
   */
  int64_t getFirstTimeNs(
      const vrs::StreamId& streamId,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime) const;
  /**
   * @brief Get last timestamp in nanoseconds of a streamId at a particular timeDomain.
   * @param streamId StreamId of the sensor stream.
   * @param timeDomain An enum class TimeDomain. Default to be DeviceTime.
   * @return Timestamp in nanosecond.
   */
  int64_t getLastTimeNs(
      const vrs::StreamId& streamId,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime) const;
  /**
   * @brief Get first timestamp in nanoseconds of all streamIds at a particular timeDomain.
   * @param timeDomain An enum class TimeDomain. Default to be DeviceTime.
   * @return Timestamp in nanosecond.
   */
  int64_t getFirstTimeNsAllStreams(const TimeDomain& timeDomain = TimeDomain::DeviceTime) const;
  /**
   * @brief Get last timestamp in nanoseconds of all streamIds at a particular timeDomain.
   * @param timeDomain An enum class TimeDomain. Default to be DeviceTime.
   * @return Timestamp in nanosecond.
   */
  int64_t getLastTimeNsAllStreams(const TimeDomain& timeDomain = TimeDomain::DeviceTime) const;

  /**
   * @brief Get sensorData from a specific timestamp in nanosecond from sensor streamId.
   * @param timeNs Query timestamp in nanoseconds.
   * @param streamId StreamId of the sensor stream.
   * @param timeDomain An enum class TimeDomain. Default to be DeviceTime.
   * @param timeQueryOptions Options that specify the output sensorData timestamp and query
   * timestamp relationship. Default is before.
   *
   * Case 1: Query timestamp in bound
   * Before output data with timestamp <= timeNs
   * After output data with timestamp >= timeNs
   * Closest output data with timestamp thats closest to timeNs from either left or right
   *
   * Case 2: Query timestamp < first timestamp
   * Before output SensorDataType::NotValid data
   * After output first timestamp
   * Closest output first timestamp
   *
   * Case 3: Query timestamp > last timestamp
   * Before output last timestamp
   * After output SensorDataType::NotValid data
   * Closest output last timestamp
   *
   * @return SensorData. If no data is available, SensorData.sensorDataType() will be
   * SensorDataType::NotValid.
   */
  SensorData getSensorDataByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);

  /**
   * @brief Get index of a the data from query timestamp in nanoseconds.
   * @param timeNs Query timestamp in nanoseconds.
   * @param streamId StreamId of the sensor stream.
   * @param timeDomain An enum class TimeDomain. Default to be DeviceTime.
   * @param timeQueryOptions Options that specify the output sensorData timestamp and query
   * timestamp relationship. Default is before.
   *
   * Case 1: Query timestamp in bound
   * Before output data index with timestamp <= timeNs
   * After output data index with timestamp >= timeNs
   * Closest output data index with timestamp thats closest to timeNs from either left or right
   *
   * Case 2: Query timestamp < first timestamp
   * Before output data index = -1
   * After output dta index of the first timestamp
   * Closest output data index of the first timestamp
   *
   * Case 3: Query timestamp > last timestamp
   * Before output data index of the last timestamp
   * After output data index = -1
   * Closest output data index of the last timestamp
   *
   * @return index (within range [0, getNumData(streamId)) or -1 if not found) of corresponds to the
   * query timestamp.
   */
  int getIndexByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);

  /**
   * @brief Convert TimeCode timestamp into DeviceTime in nanoseconds.
   * @param timecodeTimeNs Timestamp in nanoseconds from TimeCode.
   * @return Timestamp in nanosecond from DeviceTime.
   */
  int64_t convertFromTimeCodeToDeviceTimeNs(const int64_t timecodeTimeNs) const;
  /**
   * @brief Convert DeviceTime timestamp into TimeCode in nanoseconds.
   * @param deviceTimeNs Timestamp in nanoseconds from DeviceTime.
   * @return Timestamp in nanoseconds from TimeCode.
   */
  int64_t convertFromDeviceTimeToTimeCodeNs(const int64_t deviceTimeNs) const;

  /*
    call the functions below if you know the modality of the streamId
    This avoid conversion among the variant and the specific type, e.g. between SensorData and
    ImageData
  */

  // access sensor configuration specific modalities
  ImageConfigRecord getImageConfiguration(const vrs::StreamId& streamId) const;
  MotionConfigRecord getImuConfiguration(const vrs::StreamId& streamId) const;
  GpsConfigRecord getGpsConfiguration(const vrs::StreamId& streamId) const;
  WifiBeaconConfigRecord getWpsConfiguration(const vrs::StreamId& streamId) const;
  AudioConfig getAudioConfiguration(const vrs::StreamId& streamId) const;
  BarometerConfigRecord getBarometerConfiguration(const vrs::StreamId& streamId) const;
  BluetoothBeaconConfigRecord getBluetoothConfiguration(const vrs::StreamId& streamId) const;
  MotionConfigRecord getMagnetometerConfiguration(const vrs::StreamId& streamId) const;

  // retrieve by index for specific modalities
  ImageDataAndRecord getImageDataByIndex(const vrs::StreamId& streamId, const int index);
  MotionData getImuDataByIndex(const vrs::StreamId& streamId, const int index);
  GpsData getGpsDataByIndex(const vrs::StreamId& streamId, const int index);
  WifiBeaconData getWpsDataByIndex(const vrs::StreamId& streamId, const int index);
  AudioDataAndRecord getAudioDataByIndex(const vrs::StreamId& streamId, const int index);
  BarometerData getBarometerDataByIndex(const vrs::StreamId& streamId, const int index);
  BluetoothBeaconData getBluetoothDataByIndex(const vrs::StreamId& streamId, const int index);
  MotionData getMagnetometerDataByIndex(const vrs::StreamId& streamId, const int index);

  // retrieve by timestamp for specific modalities
  ImageDataAndRecord getImageDataByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);
  MotionData getImuDataByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);
  GpsData getGpsDataByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);
  WifiBeaconData getWpsDataByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);
  AudioDataAndRecord getAudioDataByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);
  BarometerData getBarometerDataByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);
  BluetoothBeaconData getBluetoothDataByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);
  MotionData getMagnetometerDataByTimeNs(
      const vrs::StreamId& streamId,
      const int64_t timeNs,
      const TimeDomain& timeDomain = TimeDomain::DeviceTime,
      const TimeQueryOptions& timeQueryOptions = TimeQueryOptions::Before);

  VrsDataProvider(
      const std::shared_ptr<RecordReaderInterface>& interface,
      const std::shared_ptr<StreamIdConfigurationMapper>& configMap,
      const std::shared_ptr<TimeCodeMapper>& timeCodeMapper,
      const std::shared_ptr<StreamIdLabelMapper>& streamIdLabelMapper,
      const std::optional<calibration::DeviceCalibration>& maybeDeviceCalib);

  virtual ~VrsDataProvider() = default; // Add a virtual destructor

 private:
  // assert if a streamId is not active
  void assertStreamIsActive(const vrs::StreamId& streamId) const;
  // assert of a streamId is not of an expected type
  void assertStreamIsType(const vrs::StreamId& streamId, SensorDataType type) const;

 private:
  const std::shared_ptr<RecordReaderInterface> interface_;
  const std::shared_ptr<StreamIdConfigurationMapper> configMap_;
  const std::shared_ptr<TimestampIndexMapper> timeQuery_;
  const std::shared_ptr<TimeCodeMapper> timeCodeMapper_;
  const std::shared_ptr<StreamIdLabelMapper> streamIdLabelMapper_;
  std::optional<calibration::DeviceCalibration> maybeDeviceCalib_;

  // pybind11 requires variable to attach to VrsDataProvider class
  // in order to keep the iterator alive
  std::pair<SensorDataIterator, SensorDataIterator> dataIterPair_;
};

} // namespace projectaria::tools::data_provider
