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
import sys
from datetime import timedelta
from typing import Dict, List, Tuple

from projectaria_tools.core.calibration import CameraCalibration
from projectaria_tools.core.mps import (
    GlobalPointPosition,
    MpsDataProvider,
    PointObservation,
)
from projectaria_tools.core.mps.utils import filter_points_from_confidence
from projectaria_tools.core.sophus import SE3

#
# Deal with Python versioning potential restriction
# This code sample is using some function from BISECT that are available only in Python 3.10+
if sys.version_info < (3, 10):
    print(
        "This code sample use bisect and function from python 3.10+. Please upgrade your python version"
    )
    exit(1)


PointsDict = Dict[int, GlobalPointPosition]
ObservationIndex = Dict[Tuple[timedelta, str], Tuple[int, int]]
PointsUVsList = List[Tuple[float, float]]
PointsIndexList = List[int]
PointsUVsAndIDs = Tuple[PointsUVsList, PointsIndexList]


class PointsAndObservationsManager:
    """
    A class to ease manipulation and retrieval of the Semi Dense Point in Aria camera stream
    """

    def __init__(self) -> None:
        """
        Initializer: create empty field - use PointsAndObservationsManager.from_mps_data_provider for initialization
        """
        # 3D points hashed by uids
        self._points: PointsDict = []
        # 2D observations sorted by timestamp and camera serial
        self._observations: List[PointObservation] = {}
        # Index to help retrieved the observations by (timestamp, camera_serial)
        self._observations_index: ObservationIndex = {}

    @property
    def observations(self):
        return self._observations

    @property
    def observations_index(self):
        return self._observations_index

    @property
    def points(self):
        return self._points

    @classmethod
    def from_mps_data_provider(cls, mps_data_provider: MpsDataProvider):
        """
        Initialize the Points And Observations manager from a mps_data_provider
        - Load data and clean it up using using default thresholds
        - Hash global point data by Unique point ids
        - Create an index for fast retrieval of the observations by (timestamp, camera_serial) key
        """
        my_object = cls()

        if (
            not mps_data_provider.has_semidense_point_cloud
            or not mps_data_provider.has_semidense_observations
        ):
            print("Semi dense point cloud and observations are not available.")
            return my_object

        #
        # Load MPS global point cloud and filter points
        #
        points_data = mps_data_provider.get_semidense_point_cloud()
        points_data_length_before_filtering = len(points_data)
        # Filter out low confidence points
        points_data = filter_points_from_confidence(points_data)
        print(
            f"Filtering make us keep: {int(100 * len(points_data) / points_data_length_before_filtering)} % of the total 3D global point data"
        )
        # Convert points to a dictionary for faster lookup by uid
        my_object._points = {pt.uid: pt for pt in points_data}
        del points_data  # Free memory (no longer needed)

        # Retrieve point observations and sort them by timestamp and camera serial
        print("Loading semi-dense observations...")
        points_observations_sorted = mps_data_provider.get_semidense_observations()
        #
        # If desired, do the following to keep only the FILTERED point observations
        #
        # Gather the global Point unique ids
        kept_point_uids = set(my_object._points.keys())
        print("Total points UIDS after filtering: ", len(kept_point_uids))
        # Keep only the observations related to those points
        points_observations_sorted = [
            obs
            for obs in points_observations_sorted
            if obs.point_uid in kept_point_uids
        ]
        # Sort the observations by timestamp and camera serial for easier indexing
        print("Sorting semi-dense observations...")
        my_object._observations = sorted(
            points_observations_sorted,
            key=lambda x: (x.frame_capture_timestamp, x.camera_serial),
        )
        del points_observations_sorted  # Free memory (no longer needed)

        # Index points using BISECT for easy retrieval by using key as (timestamp, camera_serial)
        print("Indexing semi-dense observations...")
        my_object._observations_index = my_object.index_observations(
            my_object._observations
        )

        return my_object

    # Helper function to query 2D semi-dense observations visible in a frame
    def get_slam_observations(
        self,
        capture_timestamp_ns: int,
        sensor_serial: str,
    ) -> PointsUVsAndIDs:
        """
        Enable to retrieve the visible points for this timestamp and camera serial as lists of [2d uv observations] & [unique ids]
        Note: if no observations are available we return empty lists
        """
        try:
            start_index, end_index = self._observations_index[
                timedelta(microseconds=int(capture_timestamp_ns * 1e-3)), sensor_serial
            ]
            return (
                [obs.uv for obs in self._observations[start_index:end_index]],
                [obs.point_uid for obs in self._observations[start_index:end_index]],
            )
        except KeyError:
            return [], []

    def get_rgb_observations(
        self,
        slam_visible_uids: set[int],
        camera_intrinsics: CameraCalibration,
        corrected_rgb_pose: SE3,
    ) -> PointsUVsAndIDs:
        """
        Given visible point unique ids (union of visibilities of slam left and right), we estimate the RGB visible points.
        A point is kept as visible iff the points are projected in the valid circle of the fish eye camera (i.e projection is not None)
        Will work best with Camera parameter provided by the function "OnlineRgbCameraHelper"
        """
        try:
            uvs = []
            uids = []
            # Collect possible visible 3D points
            possible_visible_points = [
                self._points[uid].position_world for uid in slam_visible_uids
            ]
            # Filter the list by using a visibility test
            for uid, point in zip(slam_visible_uids, possible_visible_points):
                rgb_projected_point = camera_intrinsics.project(
                    corrected_rgb_pose.inverse() @ point
                )
                if rgb_projected_point is not None:
                    uids.append(uid)
                    uvs.append(rgb_projected_point)
            return uvs, uids
        except KeyError:
            return [], []

    # Helper function to create an index for semi-dense observations
    @staticmethod
    def index_observations(
        points_observations_sorted: List[PointObservation],
    ) -> ObservationIndex:
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
