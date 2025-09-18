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

#include <CLI/CLI.hpp>

#include <gen2_mp_csv_exporter/Gen2MpCsvExporter.h>

using namespace projectaria::tools::mp_csv_exporter;

struct InputArgs {
  std::string vrsPath;
  std::string outputFolder;
  int vioHighFreqSubsampleRate = 1;
};

int main(int argc, const char* argv[]) {
  CLI::App app{"Nebula export MPV data to csv"};

  InputArgs args;

  app.add_option("--vrs", args.vrsPath, "Path to the source vrs file")
      ->check(CLI::ExistingPath)
      ->required();
  app.add_option("--output-folder", args.outputFolder, "Folder to output MP csv files")
      ->check(CLI::ExistingPath);
  app.add_option(
      "--vio-high-freq-subsample-rate",
      args.vioHighFreqSubsampleRate,
      "Subsample rate for VIO high freq data. Default is 1");

  CLI11_PARSE(app, argc, argv);

  runGen2MPCsvExporter(args.vrsPath, args.outputFolder, args.vioHighFreqSubsampleRate);

  return 0;
}
