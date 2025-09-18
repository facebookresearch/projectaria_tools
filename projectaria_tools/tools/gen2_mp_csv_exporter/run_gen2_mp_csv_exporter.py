# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

from projectaria_tools.core import gen2_mp_csv_exporter


def main():
    # Parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vrs-path",
        type=str,
        help="Path to vrs file.",
    )
    parser.add_argument(
        "--output-folder",
        default="",
        type=str,
        help="Folder to output MP csv files.",
    )
    parser.add_argument(
        "--vio-high-freq-subsample-rate",
        default=1,
        type=int,
        help="Subsample rate for VIO high freq data. Default is 1",
    )

    args = parser.parse_args()

    gen2_mp_csv_exporter.run_gen2_mp_csv_exporter(
        args.vrs_path, args.output_folder, args.vio_high_freq_subsample_rate
    )


if __name__ == "__main__":
    main()
