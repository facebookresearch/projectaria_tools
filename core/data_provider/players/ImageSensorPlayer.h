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

#include <data_layout/ImageSensorMetadata.h>
#include <image/FromPixelFrame.h>
#include <image/ImageVariant.h>
#include <vrs/utils/VideoRecordFormatStreamPlayer.h>

namespace projectaria::tools::data_provider {

/**
 * @brief Image data type: the pixels
 */
struct ImageData {
 public:
  /** @brief Returns ImageVariant type of the image */
  std::optional<projectaria::tools::image::ImageVariant> imageVariant() const;

  /** @brief Returns the format of the image */
  vrs::PixelFormat getPixelFormat() const;

  /** @brief Returns number of columns in image */
  uint32_t getWidth() const;
  /** @brief Returns number of rows in image */
  uint32_t getHeight() const;
  /** @brief Returns number of bytes per row in image */
  size_t getStride() const;
  /** @brief Returns if image is empty */
  bool isValid() const;

 public:
  std::shared_ptr<vrs::utils::PixelFrame> pixelFrame;
};

/**
 * @brief Image sensor configuration type
 */
struct ImageConfigRecord {
  std::string deviceType; ///< @brief type of the device
  std::string deviceVersion; ///< @brief OS version on the device
  std::string deviceSerial; ///< @brief serial of the device
  uint32_t cameraId; ///< @brief ID of the camera, 0 to N
  std::string sensorModel; ///< @brief model of the camera sensor
  std::string sensorSerial; ///< @brief serial of the camera sensor
  double nominalRateHz; ///< @brief number of frames per second
  uint32_t imageWidth; ///< @brief number of columns
  uint32_t imageHeight; ///< @brief number of height
  uint32_t imageStride; ///< @brief number of bytes per row
  uint32_t pixelFormat; ///< @brief format of the pixel
  double exposureDurationMin; ///< @brief longest exposure time allowed by the camera
  double exposureDurationMax; ///< @brief shortest exposure time allowed by the camera
  double gainMin; ///< @brief lowest gain setting allowed by the camera
  double gainMax; ///< @brief highest gain setting allowed by the camera
  double gammaFactor; ///< @brief gamma correction factor

  // Optional field, not filled when use with Aria
  std::string factoryCalibration;
  std::string onlineCalibration;
  std::string description;
};

/**
 * @brief Image data type: meta data
 */
struct ImageDataRecord {
  uint32_t cameraId; ///< @brief ID of the camera, 0 to N
  uint64_t groupId;
  uint64_t groupMask;
  uint64_t frameNumber; ///< @brief index of the frame
  double exposureDuration; ///< @brief length of exposure time
  double gain; ///< @brief gain settings
  int64_t captureTimestampNs; ///< @brief capture time in device domain
  int64_t arrivalTimestampNs; ///< @brief arrival time in host domain
  double temperature; ///< @brief capture temperature on the sensor, may be NAN
};

using ImageCallback = std::function<bool(
    const ImageData& data,
    const ImageDataRecord& record,
    const ImageConfigRecord& config,
    bool verbose)>;

class ImageSensorPlayer : public vrs::utils::VideoRecordFormatStreamPlayer {
 public:
  explicit ImageSensorPlayer(vrs::StreamId streamId) : streamId_(streamId) {}
  ImageSensorPlayer(const ImageSensorPlayer&) = delete;
  ImageSensorPlayer& operator=(const ImageSensorPlayer&) = delete;
  ImageSensorPlayer& operator=(ImageSensorPlayer&) = delete;
  ImageSensorPlayer(ImageSensorPlayer&&) = default;

  void setCallback(ImageCallback callback) {
    callback_ = callback;
  }

  const ImageData& getData() const {
    return data_;
  }

  const ImageConfigRecord& getConfigRecord() const {
    return configRecord_;
  }

  const ImageDataRecord& getDataRecord() const {
    return dataRecord_;
  }

  const vrs::StreamId& getStreamId() const {
    return streamId_;
  }

  double getNextTimestampSec() const {
    return nextTimestampSec_;
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

  void setReadContent(bool readContent) {
    readContent_ = readContent;
  }

  // An API to allow users to skip decoding the image content.
  void setSkipImageDecoding(bool skipImageDecoding) {
    skipImageDecoding_ = skipImageDecoding;
  }

  /**
   * @brief Enable empty frame mode for greatly accelerated processing.
   *
   * When enabled, this creates PixelFrame objects with proper dimensions and format
   * but without reading actual image data from the VRS file. This dramatically
   * speeds up processing when only metadata or frame structure is needed.
   *
   * Difference from readContent_:
   * - readContent_=false: Skips reading content blocks entirely, no callback invocation
   * - emptyFrameMode_=true: Reads metadata, creates empty frames, invokes callbacks
   *
   * @param emptyFrameMode If true, creates empty frames without image data
   */
  void setEmptyFrameMode(bool emptyFrameMode) {
    emptyFrameMode_ = emptyFrameMode;
  }

 private:
  bool onDataLayoutRead(const vrs::CurrentRecord& r, size_t blockIndex, vrs::DataLayout& dl)
      override;
  bool onImageRead(const vrs::CurrentRecord& r, size_t /*idx*/, const vrs::ContentBlock& cb)
      override;

  // Helper methods for different image processing modes
  bool handleEmptyFrameMode(const vrs::ContentBlock& cb);
  bool handleSkipImageDecoding(
      const vrs::CurrentRecord& r,
      const vrs::ContentBlock& cb,
      size_t blockSize);
  bool handleNormalImageProcessing(
      const vrs::CurrentRecord& r,
      const vrs::ContentBlock& cb,
      const vrs::ImageContentBlockSpec& imageSpec);
  bool handleYuv420Processing(
      const vrs::CurrentRecord& r,
      const vrs::ContentBlock& cb,
      const vrs::ImageContentBlockSpec& imageSpec);
  void invokeCallbackAndCache();

  const vrs::StreamId streamId_;
  ImageCallback callback_ =
      [](const ImageData&, const ImageDataRecord&, const ImageConfigRecord&, bool) { return true; };

  ImageData data_;
  ImageConfigRecord configRecord_;
  ImageDataRecord dataRecord_;
  int64_t cachedCaptureTimestampNs_ = -1;

  double nextTimestampSec_ = 0;
  bool verbose_ = false;
  bool readContent_ = true;
  bool skipImageDecoding_ = false;
  bool emptyFrameMode_ = false;
};

} // namespace projectaria::tools::data_provider
