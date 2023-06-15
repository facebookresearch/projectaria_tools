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
import json
import os
import zipfile


def main():
    parser = argparse.ArgumentParser("Compress submissions for evaluation.")
    parser.add_argument(
        "--phase",
        help="The phase codename for which this submission is prepared. Choose from phase1, phase2, phase3, phase4",
        default="phase1",
    )
    parser.add_argument(
        "--submission-folder",
        help="The path to the submission folder. The folder should contain a list of {sequence_name}.csv files. Refer to the evaluation_info.py or eval.ai website for the full list of sequences per phase.",
    )
    parser.add_argument(
        "--submission-zip",
        help="The path to the output submission zip file. This file will be overwritten. Please submit this zip file to eval.ai for evaluation.",
    )
    parser.add_argument(
        "--cdn-file",
        help="input file listing the CDN urls, downloaded from ADT website.",
    )
    args = parser.parse_args()

    with open(args.cdn_file) as fp:
        metadata = json.load(fp)

    with zipfile.ZipFile(
        args.submission_zip, mode="w", compression=zipfile.ZIP_BZIP2
    ) as archive:
        for seq in metadata["challenge"][args.phase]:
            seq_file = seq + ".csv"
            archive.write(
                os.path.join(args.submission_folder, seq_file), arcname=seq_file
            )


if __name__ == "__main__":
    main()
