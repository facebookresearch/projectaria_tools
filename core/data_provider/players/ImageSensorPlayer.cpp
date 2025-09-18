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

#include <cmath>
#include <optional>

#include "ImageSensorPlayer.h"

#include <data_provider/players/RgbImageFrameConverter.h>
#include <vrs/MultiRecordFileReader.h>
#include <vrs/RecordFormat.h>

namespace projectaria::tools::data_provider {
std::optional<projectaria::tools::image::ImageVariant> ImageData::imageVariant() const {
  if (pixelFrame->getSpec().getImageFormat() == vrs::ImageFormat::JPG) {
    std::shared_ptr<vrs::utils::PixelFrame> normalizedFrame;
    pixelFrame->normalizeFrame(normalizedFrame, true);
    if (!normalizedFrame) {
      const_cast<ImageData*>(this)->pixelFrame = nullptr;
    }
    const_cast<ImageData*>(this)->pixelFrame = normalizedFrame;
  }

  if (pixelFrame) {
    return image::fromPixelFrame(pixelFrame);
  } else {
    return std::nullopt;
  }
}

vrs::PixelFormat ImageData::getPixelFormat() const {
  return pixelFrame ? pixelFrame->getPixelFormat() : vrs::PixelFormat::UNDEFINED;
}
uint32_t ImageData::getWidth() const {
  return pixelFrame ? pixelFrame->getWidth() : 0;
}
uint32_t ImageData::getHeight() const {
  return pixelFrame ? pixelFrame->getHeight() : 0;
}
size_t ImageData::getStride() const {
  return pixelFrame ? pixelFrame->getStride() : 0;
}
bool ImageData::isValid() const {
  return pixelFrame != nullptr;
}

bool ImageSensorPlayer::onDataLayoutRead(
    const vrs::CurrentRecord& r,
    size_t blockIndex,
    vrs::DataLayout& dl) {
  if (r.recordType == vrs::Record::Type::CONFIGURATION) {
    auto& config = getExpectedLayout<datalayout::ImageSensorConfigRecordMetadata>(dl, blockIndex);
    configRecord_.deviceType = config.deviceType.get();
    configRecord_.deviceVersion = config.deviceVersion.get();
    configRecord_.deviceSerial = config.deviceSerial.get();
    configRecord_.cameraId = config.cameraId.get();
    configRecord_.sensorModel = config.sensorModel.get();
    configRecord_.sensorSerial = config.sensorSerial.get();
    configRecord_.nominalRateHz = config.nominalRateHz.get();
    configRecord_.imageWidth = config.imageWidth.get();
    configRecord_.imageHeight = config.imageHeight.get();
    configRecord_.imageStride = config.imageStride.get();
    configRecord_.pixelFormat = config.pixelFormat.get();
    configRecord_.exposureDurationMin = config.exposureDurationMin.get();
    configRecord_.exposureDurationMax = config.exposureDurationMax.get();
    configRecord_.gainMin = config.gainMin.get();
    configRecord_.gainMax = config.gainMax.get();
    configRecord_.gammaFactor = config.gammaFactor.get();
    configRecord_.factoryCalibration = config.factoryCalibration.get();
    configRecord_.onlineCalibration = config.onlineCalibration.get();
    configRecord_.description = config.description.get();
  } else if (r.recordType == vrs::Record::Type::DATA) {
    auto& data = getExpectedLayout<datalayout::ImageSensorDataRecordMetadata>(dl, blockIndex);
    dataRecord_.cameraId = configRecord_.cameraId;
    dataRecord_.groupId = data.groupId.get();
    dataRecord_.groupMask = data.groupMask.get();
    dataRecord_.frameNumber = data.frameNumber.get();
    dataRecord_.exposureDuration = data.exposureDuration.get();
    dataRecord_.gain = data.gain.get();
    dataRecord_.captureTimestampNs = data.captureTimestampNs.get();
    dataRecord_.arrivalTimestampNs = data.arrivalTimestampNs.get();
    // only read temperature if it is not NaN
    if (!std::isnan(data.temperature.get())) {
      dataRecord_.temperature = data.temperature.get();
    }
    nextTimestampSec_ = std::nextafter(r.timestamp, std::numeric_limits<double>::max());
  }
  return readContent_;
}

bool ImageSensorPlayer::onImageRead(
    const vrs::CurrentRecord& r,
    size_t /* idx */,
    const vrs::ContentBlock& cb) {
  const auto& imageSpec = cb.image();
  size_t blockSize = cb.getBlockSize();

  // Skip duplicate frames for encoded streams
  if (imageSpec.getImageFormat() == vrs::ImageFormat::VIDEO &&
      dataRecord_.captureTimestampNs == cachedCaptureTimestampNs_) {
    return true;
  }

  bool success = false;

  // Fast path: Create empty frames for greatly accelerated processing
  // This dramatically speeds up workflows that only need metadata or frame structure
  if (emptyFrameMode_) {
    success = handleEmptyFrameMode(cb);
  } else if (skipImageDecoding_) {
    success = handleSkipImageDecoding(r, cb, blockSize);
  } else {
    success = handleNormalImageProcessing(r, cb, imageSpec);
  }

  if (success) {
    invokeCallbackAndCache();
  } else {
    return false;
  }

  if (verbose_) {
    fmt::print(
        "{:.3f} {} [{}]: {}, {} bytes.\n",
        r.timestamp,
        r.streamId.getName(),
        r.streamId.getNumericName(),
        imageSpec.asString(),
        blockSize);
  }

  return true;
}

bool ImageSensorPlayer::handleEmptyFrameMode(const vrs::ContentBlock& cb) {
  // Create empty frame for accelerated processing - skips reading image data
  data_.pixelFrame = std::make_shared<vrs::utils::PixelFrame>(cb.image());
  return true;
}

bool ImageSensorPlayer::handleSkipImageDecoding(
    const vrs::CurrentRecord& r,
    const vrs::ContentBlock& cb,
    size_t blockSize) {
  // Allocate buffer and read raw bytes without decoding
  data_.pixelFrame = std::make_shared<vrs::utils::PixelFrame>(cb.image());

  int readErrorCode = r.reader->read(data_.pixelFrame->wdata(), blockSize);
  if (readErrorCode != 0) {
    fmt::print("Failed to read image data of size {}: error code {} \n", blockSize, readErrorCode);
    return false;
  }
  return true;
}

bool ImageSensorPlayer::handleNormalImageProcessing(
    const vrs::CurrentRecord& r,
    const vrs::ContentBlock& cb,
    const vrs::ImageContentBlockSpec& imageSpec) {
  // Handle YUV420 special case
  if (imageSpec.getPixelFormat() == vrs::PixelFormat::YUV_I420_SPLIT) {
    return handleYuv420Processing(r, cb, imageSpec);
  }

  // Handle all other pixel formats
  data_.pixelFrame = std::make_shared<vrs::utils::PixelFrame>(cb.image());
  return readFrame(*data_.pixelFrame, r, cb);
}

bool ImageSensorPlayer::handleYuv420Processing(
    const vrs::CurrentRecord& r,
    const vrs::ContentBlock& cb,
    const vrs::ImageContentBlockSpec& imageSpec) {
  // Create temporary buffer for YUV420 image
  vrs::utils::PixelFrame yuvFrame = vrs::utils::PixelFrame(cb.image());

  // Decode image to YUV420
  if (!readFrame(yuvFrame, r, cb)) {
    return false;
  }

  // Convert YUV420 to RGB8
  uint32_t width = imageSpec.getWidth();
  uint32_t height = imageSpec.getHeight();
  uint32_t stride = width * 3; // RGB8 stride
  data_.pixelFrame =
      std::make_shared<vrs::utils::PixelFrame>(vrs::PixelFormat::RGB8, width, height, stride);
  convertDecodedYuv420ToRgb8(yuvFrame, *data_.pixelFrame, width, height);
  return true;
}

void ImageSensorPlayer::invokeCallbackAndCache() {
  callback_(data_, dataRecord_, configRecord_, verbose_);
  cachedCaptureTimestampNs_ = dataRecord_.captureTimestampNs;
}

} // namespace projectaria::tools::data_provider
