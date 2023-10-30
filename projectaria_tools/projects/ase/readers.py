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

import os

import numpy as np
import pandas as pd
from projectaria_tools.core.sophus import SE3


# This function read the ASE language file
def read_language_file(filepath):
    assert os.path.exists(filepath), f"Could not find language file: {filepath}"
    with open(filepath, "r") as f:
        entities = []
        for line in f.readlines():
            line = line.rstrip()
            entries = line.split(", ")
            command = entries[0]

            entity_parameters = {}
            for parameter_def in entries[1:]:
                key, value = parameter_def.split("=")
                entity_parameters[key] = float(value)
            entities.append((command, entity_parameters))
    print(f"Loaded scene commands with a total of {len(entities)} entities.")
    return entities


# Reads a Ground truth trajectory line
def _read_trajectory_line(line):
    line = line.rstrip().split(",")
    pose = {}
    pose["timestamp"] = int(line[1])
    translation = np.array([float(p) for p in line[3:6]])
    quat_xyzw = np.array([float(o) for o in line[6:10]])
    transform = SE3.from_quat_and_translation(
        quat_xyzw[3],
        quat_xyzw[0:3],
        translation,
    )
    pose["transform"] = transform
    return pose


# Reads a Ground truth trajectory file
def read_trajectory_file(filepath):
    assert os.path.exists(filepath), f"Could not find trajectory file: {filepath}"
    with open(filepath, "r") as f:
        _ = f.readline()  # header
        transforms = []
        timestamps = []
        for line in f.readlines():
            pose = _read_trajectory_line(line)
            transforms.append(pose["transform"])
            timestamps.append(pose["timestamp"])
        timestamps = np.array(timestamps)
    print(f"Loaded trajectory with {len(timestamps)} device poses.")
    return {
        "Ts_world_from_device": transforms,
        "timestamps": timestamps,
    }


# Reads a Semidense point cloud
def read_points_file(filepath):
    assert os.path.exists(filepath), f"Could not find point cloud file: {filepath}"
    df = pd.read_csv(filepath, compression="gzip")
    point_cloud = df[["px_world", "py_world", "pz_world"]]
    print(f"Loaded point cloud with {len(point_cloud)} points.")
    return point_cloud.to_numpy()
