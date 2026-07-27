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

import enum
from typing import Any

class InstanceType(enum.IntEnum):
    UNKNOWN = enum.auto()
    OBJECT = enum.auto()
    HUMAN = enum.auto()

class RigidityType(enum.IntEnum):
    UNKNOWN = enum.auto()
    RIGID = enum.auto()
    DEFORMABLE = enum.auto()

class MotionType(enum.IntEnum):
    UNKNOWN = enum.auto()
    STATIC = enum.auto()
    DYNAMIC = enum.auto()

class AriaDigitalTwinDataPaths:
    def __str__(self) -> str: ...

class AriaDigitalTwinDataPathsProvider:
    def __init__(self, sequence_path: str) -> None: ...
    def get_datapaths(self, skeleton_flag: bool = ...) -> AriaDigitalTwinDataPaths: ...
    def get_datapaths_by_device_num(
        self, device_num: int, skeleton_flag: bool = ...
    ) -> AriaDigitalTwinDataPaths: ...
    def get_datapaths_by_device_serial(
        self, device_serial: str, skeleton_flag: bool = ...
    ) -> AriaDigitalTwinDataPaths: ...
    def get_device_serial_number(self) -> str: ...
    def get_device_serial_numbers(self) -> list[str]: ...
    def get_scene_name(self) -> str: ...
    def get_num_skeletons(self) -> int: ...
    def is_multi_person(self) -> bool: ...
    def get_concurrent_sequence_name(self) -> str | None: ...

class AriaDigitalTwinDataProvider:
    def __init__(self, data_paths: AriaDigitalTwinDataPaths) -> None: ...
    def __getattr__(self, name: str) -> Any: ...
