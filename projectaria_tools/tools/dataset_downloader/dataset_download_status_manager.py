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

import json
from typing import Final, List

DOWNLOAD_STATUS_FILE: Final[str] = ".download_status.json"


class DatasetDownloadStatusManager:
    def __init__(self, data_groups: List[str]):
        self.status = {data_group: False for data_group in data_groups}

    # Read status file from json and assign to status
    def from_json(self, json_path: str) -> None:
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
                for expected_data_group in self.status:
                    if expected_data_group not in data:
                        error = f"{DOWNLOAD_STATUS_FILE} is missing data group {expected_data_group}.\n"
                        error += f"Your data may be out of date, try clearing {DOWNLOAD_STATUS_FILE} in your sequence path"
                        raise Exception(error)
                self.status = data
        except Exception as e:
            print(f"[warning]: cannot read status json. {e}")

    def to_json(self, json_path: str):
        try:
            json_object = json.dumps(self.status, indent=4)
            with open(json_path, "w") as outfile:
                outfile.write(json_object)
        except Exception as e:
            print(f"[warning]: can not write status json. {e}")

    def set_download_status(self, data_type: str, value: bool):
        if data_type not in self.status:
            raise Exception(f"data type {data_type} is not supported")
        self.status[data_type] = value

    def get_download_status(self, data_type: str) -> bool:
        return self.status[data_type]
