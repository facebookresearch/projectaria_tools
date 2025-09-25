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

#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <fmt/core.h>
#include <vrs/StreamId.h>
#include <vrs/utils/FilterCopy.h>
#include <vrs/utils/PixelFrame.h>
#include <vrs/utils/RecordFileInfo.h>

#include "ImageMutationFilterCopier.h"

#include <CLI/CLI.hpp>

namespace {

// Demonstration on how to implement a variant of the abstract class "UserDefinedImageMutator"
// The operator is setting row to be black if the row index is % 2
struct NullifyEvenRowsImageMutator : public vrs::utils::UserDefinedImageMutator {
  bool operator()(
      double /*timestamp*/,
      const vrs::StreamId& /*streamId*/,
      vrs::utils::PixelFrame* frame) override {
    // Perform the image operation IFF the frame is valid
    if (!frame) {
      return false;
    }
    const size_t lineLength = frame->getStride();
    uint32_t top = 0;
    uint32_t bottom = frame->getHeight() - 1;
    while (top < bottom) {
      if (top % 2 == 0) {
        uint8_t* rowPixels = frame->wdata() + top * frame->getStride();
        memset(rowPixels, 0, lineLength);
      }
      top++;
    }
    return true;
  }
};

// Demonstration on how to implement a variant of the abstract class "UserDefinedImageMutator"
// to reload frame exported by `VRS export`
struct VrsExportLoader : public vrs::utils::UserDefinedImageMutator {
  std::string folderPath_;
  std::string extension_;
  std::string filenamePostfix_;
  std::map<std::string, uint64_t> frameCounter_;

  explicit VrsExportLoader(const std::string& folderPath, const std::string& extension = "jpg")
      : folderPath_(folderPath), extension_(extension) {}

  bool operator()(double timestamp, const vrs::StreamId& streamId, vrs::utils::PixelFrame* frame)
      override {
    if (!frame) {
      return false;
    }
    // Initialize frame counter or get back frame counter value
    if (frameCounter_.count(streamId.getNumericName()) == 0) {
      frameCounter_[streamId.getNumericName()] = 1;
    }
    const uint64_t imageCounter = frameCounter_[streamId.getNumericName()];

    // Build a path format that will match files exported by `vrs export`
    const std::string path = fmt::format(
        "{}/{}-{:05}-{:.3f}{}.{}",
        folderPath_,
        streamId.getNumericName(), // i.e 214-1
        imageCounter, // counter in your Mutator (see comment below)
        timestamp, // timestamp of the frame
        filenamePostfix_, // default -> ""
        extension_); // default -> "jpg"

    // Do the necessary work on the image (load and replace pixel data in the Pixel Frame)
    frame->readJpegFrameFromFile(path, true);

    // increment the frameCounter for this streamId
    ++frameCounter_[streamId.getNumericName()];
    return true;
  }
};

} // namespace

int main(int argc, const char* argv[]) {
  std::string mutationType;
  std::string vrsPathIn;
  std::string vrsPathOut;
  std::string vrsExportPath;

  CLI::App app{"VRS file Mutation example by using VRS Copy + Filter mechanism"};

  app.add_option("-i,--in", vrsPathIn, "VRS input")->required();
  app.add_option("-o,--out", vrsPathOut, "VRS output")->required();
  app.add_option(
         "-m,--mutationType",
         mutationType,
         "Mutation type:\n"
         "  - 'NULLIFY_EVEN_ROWS' or 'VRS_EXPORT_REIMPORT'\n"
         "  If mutation type is VRS_EXPORT_REIMPORT and parameter --exportPath must be specified\n"
         "  Note: NULLIFY_EVEN_ROWS is shared as a DEMO example only")
      ->required();
  app.add_option("-e,--exportPath", vrsExportPath, "VRS export output path");

  CLI11_PARSE(app, argc, argv);

  static const std::set<std::string> mutationsType = {"NULLIFY_EVEN_ROWS", "VRS_EXPORT_REIMPORT"};

  if (mutationsType.find(mutationType) == mutationsType.end()) {
    std::cerr << "Invalid <MUTATOR_TYPE> option." << std::endl;
    return EXIT_FAILURE;
  }
  if (vrsPathIn == vrsPathOut) {
    std::cerr << " <VRS_IN> <VRS_OUT> paths must be different." << std::endl;
    return EXIT_FAILURE;
  }

  vrs::utils::FilteredFileReader filteredReader;
  // Initialize VRS Reader and filters
  filteredReader.setSource(vrsPathIn);
  filteredReader.openFile();
  filteredReader.applyFilters({});

  // Configure Copy Filter and initialize the copy
  const std::string targetPath = vrsPathOut;
  vrs::utils::CopyOptions copyOptions;
  copyOptions.setCompressionPreset(vrs::CompressionPreset::Default);

  // Functor to perform image processing/conversion
  // See here the some example of Mutator (Vertical flip, Nullify, Reload VRSexport frame)
  std::shared_ptr<vrs::utils::UserDefinedImageMutator> imageMutator;
  if (mutationType == "NULLIFY_EVEN_ROWS") {
    imageMutator = std::make_shared<NullifyEvenRowsImageMutator>();
  } else if (mutationType == "VRS_EXPORT_REIMPORT") {
    imageMutator = std::make_shared<VrsExportLoader>(vrsExportPath);
  }

  auto copyMakeStreamFilterFunction = [&imageMutator](
                                          vrs::RecordFileReader& fileReader,
                                          vrs::RecordFileWriter& fileWriter,
                                          vrs::StreamId streamId,
                                          const vrs::utils::CopyOptions& copyOptions)
      -> std::unique_ptr<vrs::utils::RecordFilterCopier> {
    auto imageMutatorFilter = std::make_unique<vrs::utils::ImageMutationFilter>(
        fileReader, fileWriter, streamId, copyOptions, imageMutator.get());
    return imageMutatorFilter;
  };

  const int statusCode =
      filterCopy(filteredReader, targetPath, copyOptions, copyMakeStreamFilterFunction);

  return statusCode;
}
