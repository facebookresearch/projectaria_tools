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

from typing import Dict, List, Set, Tuple

MAX_TRACK_LENGTH = 5

# Define "Tracks" as a list of (x,y) coordinates indexed by a Global Unique Id
CoordinateList = List[Tuple[float, float]]
Tracks = Dict[int, CoordinateList]
# As Aria can have multiple channel -> store the tracks for each Aria camera channel encoded as string
TracksPerChannel = Dict[str, Tracks]


class TracksManager:
    """
    Class to update and clean tracks related to MPS Point observations
    I.E maintaining a list of points associated to their track unique Id for various camera_label
    - To be used in conjunction of "PointsAndObservationsManager" to retrieve uvs and ids of visible points at given timestamps
    """

    def __init__(self, max_track_length: int = MAX_TRACK_LENGTH):
        self.tracks: TracksPerChannel = {}
        self.max_track_length = max_track_length

    def update_tracks_and_remove_old_observations(
        self, camera_label: str, uvs: List[Tuple[float, float]], uids: List[int]
    ) -> None:
        """
        Do note that we are here preventing a track to be longer than max_track_length
        """
        tracks = self.get_track_for_camera_label(camera_label)
        for uv, uid in zip(uvs, uids):
            self.add_point(tracks, uid, uv[0], uv[1], self.max_track_length)
        # remove track_ids that are not longer visible
        self.remove_non_visible_observations(set(uids), tracks)

    def get_track_for_camera_label(self, camera_label: str) -> Tracks:
        """
        Create or return the track list corresponding to the given camera_label
        """
        if camera_label not in self.tracks:
            self.tracks[camera_label] = {}

        return self.tracks[camera_label]

    @staticmethod
    def add_point(
        tracks: Tracks,
        track_id: int,
        x: float,
        y: float,
        max_track_length: int,
    ) -> None:
        """
        Add a new point to the "tracks" dictionary
        - Each track Id have a list (x,y) observations and stored as dictionary [int, List[Any]] to enable fast lookup by track Id
        - When adding new points, we trim the history by removing the oldest observations to fit the desired track length
        """
        if track_id not in tracks or len(tracks) == 0:
            tracks[track_id]: CoordinateList = []
        # Add the point to the list
        tracks[track_id].append((x, y))
        # Clean the list if necessary (too much history)
        if len(tracks[track_id]) > max_track_length:
            tracks[track_id] = tracks[track_id][-max_track_length:]

    @staticmethod
    def remove_non_visible_observations(
        currently_visible_point_uids: Set[int],
        tracks: Tracks,
    ):
        """
        Remove tracks/points that are no longer visible
        - Remove the tracks ids that are not currently visible in the currently_visible_point_uids set
        """
        track_ids_to_remove = [
            track_id
            for track_id in tracks
            if track_id not in currently_visible_point_uids
        ]
        for track_id in track_ids_to_remove:
            del tracks[track_id]
        return tracks
