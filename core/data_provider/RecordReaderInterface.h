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

#include <condition_variable>
#include <map>
#include <mutex>
#include <set>

#include <calibration/DeviceVersion.h>
#include <data_provider/SensorData.h>
#include <data_provider/TimeSyncMapper.h>
#include <data_provider/VrsMetadata.h>
#include <data_provider/players/AlsPlayer.h>
#include <data_provider/players/EyeGazePlayer.h>
#include <data_provider/players/HandPosePlayer.h>
#include <data_provider/players/PpgPlayer.h>
#include <data_provider/players/TemperaturePlayer.h>
#include <data_provider/players/VioHighFrequencyPlayer.h>
#include <vrs/MultiRecordFileReader.h>

namespace projectaria::tools::data_provider {

/*
  read a data record from vrs cached in corresponding players
*/
class RecordReaderInterface {
 public:
  RecordReaderInterface(
      std::shared_ptr<vrs::MultiRecordFileReader> reader,
      std::map<vrs::StreamId, std::shared_ptr<ImageSensorPlayer>>& imagePlayers,
      std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>>& motionPlayers,
      std::map<vrs::StreamId, std::shared_ptr<GpsPlayer>>& gpsPlayers,
      std::map<vrs::StreamId, std::shared_ptr<WifiBeaconPlayer>>& wpsPlayers,
      std::map<vrs::StreamId, std::shared_ptr<AudioPlayer>>& audioPlayers,
      std::map<vrs::StreamId, std::shared_ptr<BarometerPlayer>>& barometerPlayers,
      std::map<vrs::StreamId, std::shared_ptr<BatteryStatusPlayer>>& batteryStatusPlayers,
      std::map<vrs::StreamId, std::shared_ptr<BluetoothBeaconPlayer>>& bluetoothPlayers,
      std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>>& magnetometerPlayers,
      std::map<vrs::StreamId, std::shared_ptr<PpgPlayer>>& ppgPlayers,
      std::map<vrs::StreamId, std::shared_ptr<AlsPlayer>>& alsPlayers,
      std::map<vrs::StreamId, std::shared_ptr<TemperaturePlayer>>& temperaturePlayers,
      std::map<vrs::StreamId, std::shared_ptr<VioPlayer>>& vioPlayers,
      std::map<vrs::StreamId, std::shared_ptr<VioHighFrequencyPlayer>>& vioHighFreqPlayers,
      std::map<vrs::StreamId, std::shared_ptr<EyeGazePlayer>>& eyegazePlayers,
      std::map<vrs::StreamId, std::shared_ptr<HandPosePlayer>>& handPosePlayers,
      const std::shared_ptr<TimeSyncMapper>& timeSyncMapper);

  std::set<vrs::StreamId> getStreamIds() const;
  [[nodiscard]] std::map<std::string, std::string> getFileTags() const;
  [[nodiscard]] std::optional<VrsMetadata> getMetadata() const;
  SensorDataType getSensorDataType(const vrs::StreamId& streamId) const;

  size_t getNumData(const vrs::StreamId& streamId) const;

  std::map<vrs::StreamId, std::vector<const vrs::IndexRecord::RecordInfo*>>
  getStreamIdToDataRecords();

  // try to read the data record at (streamId, index)(
  // if read fails return null
  // if read is successful, lock the player's corresponding mutex
  // the mutex can only be unlocked if the corresponding getLastCached*Data() is called
  const vrs::IndexRecord::RecordInfo* readRecordByIndex(const vrs::StreamId& streamId, int index);

  /* read the last cached sensor data in player */
  SensorData getLastCachedSensorData(const vrs::StreamId& streamId);

  ImageDataAndRecord getLastCachedImageData(const vrs::StreamId& streamId);
  MotionData getLastCachedImuData(const vrs::StreamId& streamId);
  GpsData getLastCachedGpsData(const vrs::StreamId& streamId);
  WifiBeaconData getLastCachedWpsData(const vrs::StreamId& streamId);
  AudioDataAndRecord getLastCachedAudioData(const vrs::StreamId& streamId);
  BluetoothBeaconData getLastCachedBluetoothData(const vrs::StreamId& streamId);
  BarometerData getLastCachedBarometerData(const vrs::StreamId& streamId);
  BatteryStatusData getLastCachedBatteryStatusData(const vrs::StreamId& streamId);
  MotionData getLastCachedMagnetometerData(const vrs::StreamId& streamId);
  PpgData getLastCachedPpgData(const vrs::StreamId& streamId);
  AlsData getLastCachedAlsData(const vrs::StreamId& streamId);
  TemperatureData getLastCachedTemperatureData(const vrs::StreamId& streamId);

  /* read the last cached on-device MP data in player */
  FrontendOutput getLastCachedVioData(const vrs::StreamId& streamId);
  OnDeviceVioHighFreqData getLastCachedVioHighFreqData(const vrs::StreamId& streamId);
  OnDeviceEyeGazeData getLastCachedEyeGazeData(const vrs::StreamId& streamId);
  OnDeviceHandPoseData getLastCachedHandPoseData(const vrs::StreamId& streamId);

  void setReadImageContent(vrs::StreamId streamId, bool readContent);
  // ISP tuning version for RGB images, 0: output image is color corrected, 1: output image is not
  // color corrected.
  [[nodiscard]] uint32_t getRgbIspTuningVersion() const;

  [[nodiscard]] std::optional<MetadataTimeSyncMode> getTimeSyncMode() const;

  // Explicit function to reset AudioPlayer's Opus decoder. This is needed to support random access
  // for Audio data
  void resetOpusDecoder(const vrs::StreamId& streamId);

 private:
  // Parameter to control the "pre-roll" length for encoded audio signal.
  // 5 samples stands for approx 100ms, which is enough for pre-roll
  static constexpr int kAudioDecodingPrerollLength = 5;

  // Track last read index per audio stream for Opus pre-roll detection
  std::map<vrs::StreamId, int> audioStreamLastReadIndex_;

  // Cache which audio streams use Opus encoding (determined during initialization)
  std::map<vrs::StreamId, bool> audioStreamIsOpus_;

  // Helper method to determine if audio stream needs Opus pre-roll
  bool needsOpusPreroll(const vrs::StreamId& streamId, int targetIndex);

  // Helper method to perform Opus pre-roll
  void performOpusPreroll(const vrs::StreamId& streamId, int targetIndex);

  // Check if audio stream uses Opus encoding (cached lookup)
  bool isOpusAudioStream(const vrs::StreamId& streamId);

  // Initialize Opus detection cache during construction
  void initializeOpusDetection();

  // Update Opus detection cache for a specific stream
  void updateOpusDetection(const vrs::StreamId& streamId, bool isOpus);

  std::shared_ptr<vrs::MultiRecordFileReader> reader_;

  calibration::DeviceVersion deviceVersion_;

  std::set<vrs::StreamId> streamIds_;
  std::map<vrs::StreamId, SensorDataType> streamIdToSensorDataType_;
  std::map<std::string, std::string> fileTags_;
  std::optional<VrsMetadata> vrsMetadata_;

  std::map<vrs::StreamId, std::shared_ptr<ImageSensorPlayer>> imagePlayers_;
  std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>> motionPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<GpsPlayer>> gpsPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<WifiBeaconPlayer>> wpsPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<AudioPlayer>> audioPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<BarometerPlayer>> barometerPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<BatteryStatusPlayer>> batteryStatusPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<BluetoothBeaconPlayer>> bluetoothPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<MotionSensorPlayer>> magnetometerPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<PpgPlayer>> ppgPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<AlsPlayer>> alsPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<TemperaturePlayer>> temperaturePlayers_;

  std::map<vrs::StreamId, std::shared_ptr<VioPlayer>> vioPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<VioHighFrequencyPlayer>> vioHighFreqPlayers_;
  std::map<vrs::StreamId, std::shared_ptr<EyeGazePlayer>> eyegazePlayers_;
  std::map<vrs::StreamId, std::shared_ptr<HandPosePlayer>> handPosePlayers_;

  std::shared_ptr<TimeSyncMapper> timeSyncMapper_;

  std::unique_ptr<std::mutex> readerMutex_;
  std::map<vrs::StreamId, std::unique_ptr<std::mutex>> streamIdToPlayerMutex_;
  std::map<vrs::StreamId, std::unique_ptr<std::condition_variable>> streamIdToCondition_;
  std::map<vrs::StreamId, const vrs::IndexRecord::RecordInfo*> streamIdToLastReadRecord_;
};

} // namespace projectaria::tools::data_provider
