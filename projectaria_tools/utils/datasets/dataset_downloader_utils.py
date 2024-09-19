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
import os
from typing import Dict, Final, List


###########################################################################
# TODO: this was copied from dataverse_pipeline/constants.py
# We should consolidate this into one location so the code isn't duplicated
###########################################################################

# We require AT LEAST ONE of these files for eye_gaze
MPS_EYE_GAZE_FILES_REQUIRED: Final[List[str]] = [
    "general_eye_gaze.csv",
    "generalized_eye_gaze.csv",
    "personalized_eye_gaze.csv",
    "calibrated_eye_gaze.csv",
]

MPS_EYE_GAZE_FILES: Final[List[str]] = MPS_EYE_GAZE_FILES_REQUIRED + ["summary.json"]

# We requires ALL of these SLAM files
MPS_SLAM_FILES_REQUIRED: Final[List[str]] = [
    "closed_loop_trajectory.csv",
    "open_loop_trajectory.csv",
    "online_calibration.jsonl",
    "semidense_observations.csv.gz",
]

MPS_SLAM_FILES: Final[List[str]] = MPS_SLAM_FILES_REQUIRED + [
    "semidense_points.csv.gz",
    "global_points.csv.gz",
    "summary.json",
]

MPS_HAND_TRACKING_FILES: Final[List[str]] = ["wrist_and_palm_poses.csv", "summary.json"]

MPS_FOLDER: Final[str] = "mps"
MPS_SLAM_FOLDER: Final[str] = "slam"
MPS_EYE_GAZE_FOLDER: Final[str] = "eye_gaze"
MPS_HAND_TRACKING_FOLDER: Final[str] = "hand_tracking"

MPS_SLAM_GROUP_FILES: Final[Dict[str, List[str]]] = {
    "mps_slam_trajectories": [MPS_SLAM_FILES[0], MPS_SLAM_FILES[1]],
    "mps_slam_calibration": [MPS_SLAM_FILES[2]],
    "mps_slam_points": [
        MPS_SLAM_FILES[3],
        MPS_SLAM_FILES[4],
        MPS_SLAM_FILES[5],
    ],
    "mps_slam_summary": [MPS_SLAM_FILES[6]],
}
MPS_DATA_GROUP_FILES: Final[Dict[str, List[str]]] = {
    **MPS_SLAM_GROUP_FILES,
    **{
        "mps_eye_gaze": MPS_EYE_GAZE_FILES,
        "mps_hand_tracking": MPS_HAND_TRACKING_FILES,
    },
}
###########################################################################


# Load the list of sequences from the cdn file
def load_sequences_list_from_cdn(cdn_file: str) -> List[str]:
    if not os.path.exists(cdn_file):
        raise Exception("CDN file does not exist")

    # load sequences from cdn file
    sequences_list = []
    with open(cdn_file, "r") as f:
        cdn_file_data = json.load(f)
        if "sequences" not in cdn_file_data:
            raise Exception("invalid cdn_file, missing key: sequences")
        sequences_list = cdn_file_data["sequences"].keys()
    return sequences_list


# Load a list of data group name to all files included in that data group
def load_data_groups_from_cdn(cdn_file: str) -> Dict[str, List[str]]:
    if not os.path.exists(cdn_file):
        raise Exception("CDN file does not exist")

    # load sequence_config from cdn file
    sequence_config = {}
    with open(cdn_file, "r") as f:
        cdn_file_data = json.load(f)
        if "sequence_config" not in cdn_file_data:
            raise Exception("invalid cdn_file, missing key: sequence_config")
        sequence_config = cdn_file_data["sequence_config"]

    data_groups = {}

    # load main vrs data group
    if "main" not in sequence_config:
        raise Exception("invalid cdn_file, missing key: sequence_config/main")
    main_data = sequence_config["main"]
    if "recording" not in main_data:
        raise Exception("invalid cdn_file, missing key: sequence_config/main/recording")

    # Only add "recording" and "mps" data groups if defined as not "None"
    if main_data["recording"] != "None":
        data_groups["main_vrs"] = main_data["recording"]
    if main_data["mps"] != "None":
        data_groups.update(MPS_DATA_GROUP_FILES)

    # load custom data groups for this dataset
    if "data_groups" not in sequence_config:
        raise Exception("invalid cdn_file, missing key: sequence_config/data_groups")

    for data_group, files in sequence_config["data_groups"].items():
        data_groups[data_group] = files

    return data_groups
