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
    nextTimestampSec_ = std::nextafter(r.timestamp, std::numeric_limits<double>::max());
  }
  return readContent_;
}

bool ImageSensorPlayer::onImageRead(
    const vrs::CurrentRecord& r,
    size_t /*idx*/,
    const vrs::ContentBlock& cb) {
  // the image data was not read yet: allocate your own buffer & read!
  auto& imageSpec = cb.image();
  size_t blockSize = cb.getBlockSize();
  // Synchronously read the image data, which is jpg compressed with Aria
  if (imageSpec.getImageFormat() == vrs::ImageFormat::JPG) {
    vrs::utils::PixelFrame::readJpegFrame(data_.pixelFrame, r.reader, cb.getBlockSize());
    callback_(r, data_.pixelFrame->getBuffer(), verbose_);
  } else if (imageSpec.getImageFormat() == vrs::ImageFormat::RAW) {
    vrs::utils::PixelFrame::readRawFrame(data_.pixelFrame, r.reader, imageSpec);
    callback_(r, data_.pixelFrame->getBuffer(), verbose_);
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
  return true; // read next blocks, if any
}

} // namespace projectaria::tools::data_provider
