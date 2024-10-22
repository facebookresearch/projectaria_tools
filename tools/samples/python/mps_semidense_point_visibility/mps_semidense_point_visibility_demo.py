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

import bisect
import os

import sys
from datetime import timedelta
from typing import Dict, List, Set

import numpy as np
import rerun as rr
from projectaria_tools.core import data_provider, mps
from projectaria_tools.core.mps import (
    MpsDataPathsProvider,
    MpsDataProvider,
    PointObservation,
)
from projectaria_tools.core.mps.utils import filter_points_from_confidence
from projectaria_tools.core.sensor_data import SensorDataType, TimeDomain
from projectaria_tools.core.stream_id import StreamId
from projectaria_tools.utils.rerun_helpers import AriaGlassesOutline, ToTransform3D
from tqdm import tqdm

# MPS Semi Dense Point Visibility Demo
# - Show how to use Semi Dense Point Observations in conjunction of the VRS image frames
#
#
# Key learnings:
# 1. MPS Point cloud data consists of:
# - A global point cloud (3D points)
# - Point cloud observations (visibility information for camera_serial and timestamps)
# Global points and their observations are linked together by their unique 3D point ids

# 2. How to connect MPS data to corresponding VRS image:
# MPS point cloud data observations is indexed by camera_serial and can be slam_left or slam_right
# MPS data is indexed by timestamp in microseconds
# VRS data is indexed by timestamp in nanoseconds

# 3. This code sample show how to keep a list of visible tracks at the current timestamp
# I.E point observations are accumulated and hashed by their global point unique ids
# - if a point is not visible on the current frame, the track is removed
#

#
# Deal with Python versioning potential restriction
# This code sample is using some function from BISECT that are available only in Python 3.10+
if sys.version_info < (3, 10):
    print(
        "This code sample use bisect and function from python 3.10+. Please upgrade your python version"
    )
    exit(1)

MAX_TRACK_LENGTH = 5

#
# Data Loading
# MPS output paths
mps_folder_data = "../../../../data/mps_sample"
vrs_file = os.path.join(mps_folder_data, "sample.vrs")
mps_data_paths = MpsDataPathsProvider(mps_folder_data)
mps_data_provider = MpsDataProvider(mps_data_paths.get_data_paths())

# Check we have the required data
if not (
    mps_data_provider.has_closed_loop_poses
    and mps_data_provider.has_semidense_point_cloud
    and mps_data_provider.has_semidense_observations
):
    print(
        "Missing required data for this demo (either closed loop trajectory, semi dense point cloud, semi dense observations are missing)"
    )
    exit(1)

left_slam_stream_id = StreamId("1201-1")
right_slam_stream_id = StreamId("1201-2")


# Helper function to create an index for semi-dense observations
def index_observations(points_observations_sorted: List[PointObservation]) -> Dict:
    """
    Hashes observations by timestamp and camera serial
    - leverage the fact that input observations are sorted for quick indexing
    """
    observations_index = {}
    start_index = 0
    while True:
        upper_bound = bisect.bisect_right(
            points_observations_sorted,
            points_observations_sorted[start_index].frame_capture_timestamp,
            lo=start_index,
            hi=len(points_observations_sorted),
            key=lambda x: x.frame_capture_timestamp,
        )
        upper_bound_serial = bisect.bisect_right(
            points_observations_sorted,
            points_observations_sorted[start_index].camera_serial,
            lo=start_index,
            hi=upper_bound - 1,
            key=lambda x: x.camera_serial,
        )
        observations_index[
            points_observations_sorted[start_index].frame_capture_timestamp,
            points_observations_sorted[start_index].camera_serial,
        ] = (start_index, upper_bound_serial - 1)
        observations_index[
            points_observations_sorted[start_index].frame_capture_timestamp,
            points_observations_sorted[upper_bound_serial].camera_serial,
        ] = (upper_bound_serial, upper_bound - 1)
        start_index = upper_bound
        if upper_bound >= len(points_observations_sorted):
            break
    return observations_index


# Helper function to query 2D semi-dense observations visible in a frame
def get_2dpoints_uvs_uids(
    capture_timestamp_ns: int,
    sensor_serial: str,
    observations: List[PointObservation],
    observations_index: Dict,
):
    """
    Leverage the observations index to retrieve the visible 2d uv observations and unique ids for this timestamp and camera serial
    Note: observations could be not existing, in this case we return empty arrays
    """
    try:
        start_index, end_index = observations_index[
            timedelta(microseconds=int(capture_timestamp_ns * 1e-3)), sensor_serial
        ]
        return (
            np.array([obs.uv for obs in observations[start_index:end_index]]),
            [obs.point_uid for obs in observations[start_index:end_index]],
        )
    except KeyError:
        return (np.array([]), np.array([]))


#
# Load MPS trajectory data
#
trajectory_data = mps.read_closed_loop_trajectory(
    mps_data_paths.get_data_paths().slam.closed_loop_trajectory
)
# Reduce the number of points to display more easily
device_trajectory = [
    it.transform_world_device.translation()[0] for it in trajectory_data
][0::80]

#
# Load MPS global point cloud and filter points
#
points_data = mps_data_provider.get_semidense_point_cloud()
points_data_length_before_filtering = len(points_data)
# Filter out low confidence points
points_data = filter_points_from_confidence(points_data)
print(
    f"Filtering make us keep: {int(100 * len(points_data)/points_data_length_before_filtering)} % of the total 3D global point data"
)
# Convert points to a dictionary for faster lookup by uid
points_dict = {pt.uid: pt for pt in points_data}
del points_data  # Free memory (no longer needed)

# Retrieve point observations and sort them by timestamp and camera serial
print("Loading semi-dense observations...")
points_observations_sorted = mps_data_provider.get_semidense_observations()
#
# If desired, do the following to keep only the FILTERED point observations
#
# Gather the global Point unique ids
kept_point_uids = set(points_dict.keys())
print("Total points UIDS after filtering: ", len(kept_point_uids))
# Keep only the observations related to those points
points_observations_sorted = [
    obs for obs in points_observations_sorted if obs.point_uid in kept_point_uids
]
# Sort the observations by timestamp and camera serial for easier indexing
print("Sorting semi-dense observations...")
points_observations_sorted = sorted(
    points_observations_sorted,
    key=lambda x: (x.frame_capture_timestamp, x.camera_serial),
)

# Index points using BISECT for easy retrieval by using key as (timestamp, camera_serial)
print("Indexing semi-dense observations...")
observations_index = index_observations(points_observations_sorted)

# Display
#
# Loop over the recording (SLAM Images, and display points that are visible at this timestamp)
#
vrs_data_provider = data_provider.create_vrs_data_provider(vrs_file)
device_calibration = vrs_data_provider.get_device_calibration()
deliver_option = vrs_data_provider.get_default_deliver_queued_options()
deliver_option.deactivate_stream_all()
deliver_option.activate_stream(left_slam_stream_id)
deliver_option.activate_stream(right_slam_stream_id)

left_slam_serial = (
    vrs_data_provider.get_configuration(left_slam_stream_id)
    .image_configuration()
    .sensor_serial
)
right_slam_serial = (
    vrs_data_provider.get_configuration(right_slam_stream_id)
    .image_configuration()
    .sensor_serial
)

rr.init("MPS SemiDensePoint Viewer", spawn=True)

# Display device trajectory
rr.log(
    "world/device_trajectory",
    rr.LineStrips3D(device_trajectory, radii=0.008),
    static=True,
)

# Display global point cloud
points = [pt.position_world for pt in points_dict.values()]
rr.log("world/points", rr.Points3D(points, radii=0.005), static=True)
del points  # Free memory (no longer needed)


#
# Log Aria Glasses outline
#
aria_glasses_point_outline = AriaGlassesOutline(device_calibration)
rr.log(
    "world/device/glasses_outline",
    rr.LineStrips3D([aria_glasses_point_outline]),
    static=True,
)


def add_point(
    tracks: List,
    track_id: int,
    x: float,
    y: float,
    max_track_length: int = MAX_TRACK_LENGTH,
):
    """
    Add a new point to the "tracks" dictionary
    - Tracks are defined as the following:
       track_id hash -> Key of the dictionary
       for each track we have a list of observations (x,y)
    If track is longer than expected, we remove the oldest observation
    """
    if track_id not in tracks:
        tracks[track_id] = {"points": [(x, y)]}
    else:
        tracks[track_id]["points"].append((x, y))
        if len(tracks[track_id]["points"]) > MAX_TRACK_LENGTH:
            tracks[track_id]["points"] = tracks[track_id]["points"][-MAX_TRACK_LENGTH:]


def remove_non_visible_observations(currently_visible_uids: Set[int], tracks: List):
    """
    Remove tracks/points that are no longer visible
    - Remove the tracks ids that are not currently visible in the currently_visible_uids set
    """
    track_ids_to_remove = [
        track_id for track_id in tracks if track_id not in currently_visible_uids
    ]
    for track_id in track_ids_to_remove:
        del tracks[track_id]
    return tracks


left_tracks = {}  # dictionary to store left camera tracks
right_tracks = {}  # dictionary to store right camera tracks


for data in tqdm(vrs_data_provider.deliver_queued_sensor_data(deliver_option)):
    device_time_ns = data.get_time_ns(TimeDomain.DEVICE_TIME)
    rr.set_time_nanos("device_time", device_time_ns)

    # Display device pose
    closed_loop_pose = mps_data_provider.get_closed_loop_pose(device_time_ns)
    if closed_loop_pose:
        T_world_device = closed_loop_pose.transform_world_device
        rr.log(
            "world/device",
            ToTransform3D(T_world_device, False),
        )

    # In order to factorize code, we use the following variables to store the label, camera serial and tracks of the current frame (left or right)
    stream_label = camera_serial = tracks = None
    if data.stream_id() == left_slam_stream_id:
        stream_label = "camera-slam-left"
        camera_serial = left_slam_serial
        tracks = left_tracks
    elif data.stream_id() == right_slam_stream_id:
        stream_label = "camera-slam-right"
        camera_serial = right_slam_serial
        tracks = right_tracks

    # Collect 2D coordinates (uvs) and unique ids (uuids) of all visible points in this frame & display them
    if data.sensor_data_type() == SensorDataType.IMAGE:
        slam_frame = data.image_data_and_record()[0].to_numpy_array()
        rr.log(stream_label, rr.Image(slam_frame))

        uvs, uids = get_2dpoints_uvs_uids(
            device_time_ns,
            camera_serial,
            points_observations_sorted,
            observations_index,
        )
        rr.log(
            stream_label + "/observations",
            rr.Points2D(uvs, colors=[255, 255, 0], radii=1, class_ids=uids),
        )

        #
        # Collect visible 3D points and display them
        #
        visible_points = [points_dict[uid].position_world for uid in uids]
        rr.log(
            f"world/tracked_points_{stream_label}",
            rr.Points3D(
                visible_points,
                radii=0.02,
                colors=(
                    [255, 255, 0]
                    if stream_label == "camera-slam-left"
                    else [0, 255, 255]
                ),
            ),
        )

        #
        # Display tracks history (track traces)
        #
        for uv, id in zip(uvs, uids):
            add_point(tracks, id, uv[0], uv[1])
        # remove track_ids that are not longer visible
        remove_non_visible_observations(set(uids), tracks)

        # Compile tracks as a list to display them as lines
        tracks_points = [tracks[track_id]["points"] for track_id in tracks]
        rr.log(
            stream_label + "/track",
            rr.LineStrips2D(tracks_points),
        )
