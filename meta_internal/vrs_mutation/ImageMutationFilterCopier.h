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

#include <vrs/RecordFormat.h>
#include <vrs/StreamId.h>
#include <vrs/helpers/Strings.h>
#include <vrs/utils/FilteredFileReader.h>
#include <vrs/utils/PixelFrame.h>

namespace vrs::utils {

// Abstract PixelFrame mutator to be used in conjunction of
// ImageMutationFilter to modify VRS frame when performing a VRS copy.
class UserDefinedImageMutator {
 public:
  virtual ~UserDefinedImageMutator() {}
  // Operator allowing to modify a given PixelFrame (according to its timestamp and streamId)
  // -> Note the image size (Width, Height, Stride) must be left unchanged
  virtual bool operator()(
      double, // timestamp
      const vrs::StreamId&, // streamId
      vrs::utils::PixelFrame* // frame
      ) = 0;
};

class ImageMutationFilter : public RecordFilterCopier {
 public:
  ImageMutationFilter(
      vrs::RecordFileReader& fileReader,
      vrs::RecordFileWriter& fileWriter,
      vrs::StreamId id,
      const CopyOptions& copyOptions,
      UserDefinedImageMutator* userDefinedImageMutator)
      : RecordFilterCopier(fileReader, fileWriter, id, copyOptions),
        userDefinedImageMutator_(userDefinedImageMutator) {}

  bool shouldCopyVerbatim(const CurrentRecord& record) override {
    auto tupleId = tuple<Record::Type, uint32_t>(record.recordType, record.formatVersion);
    const auto& verbatimCopyIter = verbatimCopy_.find(tupleId);
    if (verbatimCopyIter != verbatimCopy_.end()) {
      return verbatimCopyIter->second;
    }
    const auto& decoders = readers_.find(tuple<StreamId, Record::Type, uint32_t>(
        record.streamId, record.recordType, record.formatVersion));
    bool verbatimCopy = decoders == readers_.end() ||
        (record.recordType == Record::Type::DATA &&
         decoders->second.recordFormat.getBlocksOfTypeCount(ContentType::IMAGE) == 0);
    verbatimCopy_[tupleId] = verbatimCopy;
    return verbatimCopy;
  }
  void filterImage(
      const CurrentRecord& record,
      size_t /*blockIndex*/,
      const ContentBlock& cb,
      vector<uint8_t>& pixels) override {
    const auto& imageSpec = cb.image();

    // Synchronously read the image data, which is jpg compressed with Aria
    if (imageSpec.getImageFormat() == vrs::ImageFormat::JPG) {
      std::shared_ptr<vrs::utils::PixelFrame> frame = std::make_shared<PixelFrame>();
      frame->readJpegFrame(pixels, cb.getBlockSize());

      // Call user defined mutator on the frame
      if (userDefinedImageMutator_) {
        if (!userDefinedImageMutator_->operator()(record.timestamp, record.streamId, &(*frame))) {
          // If mutator not successful, we return an empty black frame
          if (!pixels.empty()) {
            memset(pixels.data(), 0, pixels.size());
          }
        }
      }
      // Re-encode to JPG
      frame->jpgCompress(pixels, 90);
    } else if (imageSpec.getImageFormat() == vrs::ImageFormat::RAW) {
      // Directly process pixels buffer as you wish
    }
  }

 protected:
  map<tuple<Record::Type, uint32_t>, bool> verbatimCopy_;
  UserDefinedImageMutator* userDefinedImageMutator_;
};

} // namespace vrs::utils
