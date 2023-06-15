#!/usr/bin/env python3
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
from typing import List


class AriaDigitalTwinDatasetSearcher:
    def __init__(self, metadata_path: str):
        with open(metadata_path, "r") as file_obj:
            self.metadata = json.load(file_obj)

    def get_sequences_with_skeleton_number(self, skeleton_number: int = 1) -> List[str]:
        results = []
        for sequence in self.metadata.get("contents", []):
            for tour_name, metadata in sequence.items():
                if metadata.get("num_skeletons", 0) == skeleton_number:
                    results.append(tour_name)
        return results

    def get_sequences_with_multi_persons(self) -> List[str]:
        results = []
        for sequence in self.metadata.get("contents", []):
            for tour_name, metadata in sequence.items():
                if metadata.get("is_multi_person", False):
                    results.append(tour_name)
        return results

    def get_sequences_with_single_person(self) -> List[str]:
        results = []
        for sequence in self.metadata.get("contents", []):
            for tour_name, metadata in sequence.items():
                if metadata.get("is_multi_person", False):
                    results.append(tour_name)
        return results

    def get_sequences_of_scene(self, scene: str) -> List[str]:
        results = []
        for sequence in self.metadata.get("contents", []):
            for tour_name, metadata in sequence.items():
                if scene in metadata.get("scenes", []):
                    results.append(tour_name)
        return results


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset_metadata",
        dest="dataset_metadata",
        type=str,
        required=True,
        help="path to the metadata file of aria digital twin dataset",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    searcher = AriaDigitalTwinDatasetSearcher(args.dataset_metadata)
    print(
        "Sequences without skeleton: ", searcher.get_sequences_with_skeleton_number(0)
    )
    print("Sequences with 1 skeleton: ", searcher.get_sequences_with_skeleton_number(1))
    print(
        "Sequences with 2 skeleton2: ", searcher.get_sequences_with_skeleton_number(2)
    )
    print(
        "Sequences with multiple persons: ", searcher.get_sequences_with_multi_persons()
    )
    print("Sequences with single person: ", searcher.get_sequences_with_single_person())
    print("Sequences from Apartment: ", searcher.get_sequences_of_scene("Apartment"))
    print("Sequences from LiteOffice: ", searcher.get_sequences_of_scene("LiteOffice"))
