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
from typing import Dict, List

import rerun as rr
import rerun.blueprint as rrb

from PointsAndObservationsManager import (
    PointsAndObservationsManager,
    PointsDict,
    PointsIndexList,
    PointsUVsList,
)

from projectaria_tools.core import data_provider, mps
from projectaria_tools.core.mps import MpsDataPathsProvider, MpsDataProvider
from projectaria_tools.core.sensor_data import SensorDataType, TimeDomain
from projectaria_tools.core.stream_id import StreamId
from projectaria_tools.utils.rerun_helpers import AriaGlassesOutline, ToTransform3D
from tqdm import tqdm
from TracksManager import MAX_TRACK_LENGTH, Tracks, TracksManager
from utils import OnlineRgbCameraHelper

# MPS Semi Dense Point Visibility Demo
# - Show how to use Semi Dense Point Observations in conjunction of the VRS image frames
#
#
# Key learnings:
# 1. MPS Point cloud data consists of:
# - A global point cloud (3D points)
# - Point cloud observations (visibility information for camera_serial and timestamps)
# -> Global points and their observations are linked together by their unique 3D point ids
# => We are introducing the PointsAndObservationsManager class to enable easy retrieval of visible point per timestamp for each camera

# 2. How to connect MPS data to corresponding VRS image:
# MPS point cloud data observations is indexed by camera_serial and can be slam_left or slam_right
# MPS data is indexed by timestamp in microseconds
# VRS data is indexed by timestamp in nanoseconds

# 3. This code sample show how to keep a list of visible tracks at the current timestamp
# I.E point observations are accumulated and hashed by their global point unique ids
# - if a point is not visible on the current frame, the track is removed
# => We are introducing the TracksManager manager class to update and store the visible tracks


RERUN_JPEG_QUALITY = 75
# Create alias for the stream ids
LEFT_SLAM_STREAM_ID = StreamId("1201-1")
RIGHT_SLAM_STREAM_ID = StreamId("1201-2")
RGB_STREAM_ID = StreamId("214-1")

#
# Configure Data Loading
# MPS output paths
mps_folder_data = "../../../../data/mps_sample"
vrs_file = os.path.join(mps_folder_data, "sample.vrs")


#
# Utility function
def display_tracks_and_points(
    uvs: PointsUVsList,
    uids: PointsIndexList,
    points_dict: PointsDict,  # Global Points indexed by uid
    tracks: Tracks,
    stream_id: StreamId,
    stream_label: str,
) -> None:
    """
    Display onto existing images:
     - 2D observations (uvs)
     - tracklets (the past trace of the tracked point)
     - 3D visible points (for SLAM left and right)
    """
    # Display the collected 2D point projections
    #
    rr.log(
        stream_label + "/observations",
        rr.Points2D(uvs, colors=[255, 255, 0], radii=1),
    )

    # Display tracklets
    #

    # Compile tracks as a list to display them as lines
    tracked_points = [tracks[track_id] for track_id in tracks]
    rr.log(
        stream_label + "/track",
        rr.LineStrips2D(tracked_points, radii=2),
    )

    # Collect visible 3D points and display them
    #
    if stream_id in [
        LEFT_SLAM_STREAM_ID,
        RIGHT_SLAM_STREAM_ID,
    ]:
        points = [points_dict[uid].position_world for uid in uids]
        rr.log(
            f"world/tracked_points_{stream_label}",
            rr.Points3D(
                points,
                radii=0.02,
                colors=(
                    [255, 255, 0] if stream_id == LEFT_SLAM_STREAM_ID else [0, 255, 255]
                ),
            ),
        )


###
###

#
# Initialize the interface to read MPS data
mps_data_paths = MpsDataPathsProvider(mps_folder_data)
mps_data_provider = MpsDataProvider(mps_data_paths.get_data_paths())
# Check we have the required MPS data available
if not (
    mps_data_provider.has_closed_loop_poses
    and mps_data_provider.has_online_calibrations  # Required to have accurate RGB camera poses
    and mps_data_provider.has_semidense_point_cloud
    and mps_data_provider.has_semidense_observations
):
    print(
        "Missing required data for this demo (either closed loop trajectory, semi dense point cloud, semi dense observations, online calibration are missing)"
    )
    exit(1)

#
# Initialize the Semi Dense Points and observations manager
# This interface will provide us an easy way to retrieve visible points per camera stream and timestamp
points_and_observations_manager = PointsAndObservationsManager.from_mps_data_provider(
    mps_data_provider
)

#
# Configure a VRS data provider to get all the SLAM and RGB images
vrs_data_provider = data_provider.create_vrs_data_provider(vrs_file)
device_calibration = vrs_data_provider.get_device_calibration()
deliver_option = vrs_data_provider.get_default_deliver_queued_options()
deliver_option.deactivate_stream_all()
deliver_option.activate_stream(LEFT_SLAM_STREAM_ID)
deliver_option.activate_stream(RIGHT_SLAM_STREAM_ID)
deliver_option.activate_stream(RGB_STREAM_ID)

#
# Load MPS trajectory data (and reduce the number of points for display)
trajectory_data = mps.read_closed_loop_trajectory(
    mps_data_paths.get_data_paths().slam.closed_loop_trajectory
)
device_trajectory = [
    it.transform_world_device.translation()[0] for it in trajectory_data
][0::80]


#
# Initialize Rerun and set the viewing layout (3D, RGB, VerticalStack(SlamLeft, SlamRight)):
rr.init("MPS SemiDensePoint Viewer", spawn=True)
my_blueprint = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Spatial3DView(origin="world"),
        rrb.Spatial2DView(origin="camera-rgb"),
        rrb.Vertical(
            rrb.Spatial2DView(origin="camera-slam-left"),
            rrb.Spatial2DView(origin="camera-slam-right"),
        ),
    ),
    collapse_panels=True,
)
rr.send_blueprint(my_blueprint)


# Display device trajectory
rr.log(
    "world/device_trajectory",
    rr.LineStrips3D(device_trajectory, radii=0.008),
    static=True,
)

# Display global point cloud
points = [pt.position_world for pt in points_and_observations_manager.points.values()]
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

#
# Save tracks for each stream_id (the last X visible 2d projection coordinates for each visible track ID in the current frame)
aria_track_manager = TracksManager(max_track_length=MAX_TRACK_LENGTH)

#
# SemiDense point visibility information is done only for the SLAM cameras
# RGB point cloud visibility is not pre-computed, we estimate point visibilities from the SLAM ones
# - We are here store the Global point cloud unique ids that are visible in the current frame set (left and right)
# - so we can know later estimate which point is visible in the RGB frame
frame_set_uids: Dict[str, List[int]] = {}

#
# Display loop
# - going over the images
# - retrieve visible points
# - update tracks and display them
#
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

    # Retrieve the stream label and current camera serial
    camera_serial = tracks = None
    stream_label = vrs_data_provider.get_label_from_stream_id(data.stream_id())
    if data.stream_id() == LEFT_SLAM_STREAM_ID:
        camera_serial = (
            vrs_data_provider.get_configuration(LEFT_SLAM_STREAM_ID)
            .image_configuration()
            .sensor_serial
        )
    elif data.stream_id() == RIGHT_SLAM_STREAM_ID:
        camera_serial = (
            vrs_data_provider.get_configuration(RIGHT_SLAM_STREAM_ID)
            .image_configuration()
            .sensor_serial
        )

    # For each view we will collect the visible uvs (2d projection) and the ids of the visible 3D points
    # Note: RGB view will be handled separately as projection needs to be computed and not directly available

    # If this is an image, display it
    if data.sensor_data_type() == SensorDataType.IMAGE:
        frame = data.image_data_and_record()[0].to_numpy_array()
        rr.log(stream_label, rr.Image(frame).compress(jpeg_quality=RERUN_JPEG_QUALITY))

        # Collect and display "slam images" visible semi dense point 2D coordinates (uvs) and unique ids (uuids)
        if data.stream_id() in [
            LEFT_SLAM_STREAM_ID,
            RIGHT_SLAM_STREAM_ID,
        ]:
            uvs, uids = points_and_observations_manager.get_slam_observations(
                device_time_ns,
                camera_serial,
            )
            # Store the current visible global points uids for this view to propagate to the RGB view
            frame_set_uids[str(data.stream_id())] = uids

            #
            # Update tracks and display them (for slam left or right)
            aria_track_manager.update_tracks_and_remove_old_observations(
                stream_label, uvs, uids
            )
            display_tracks_and_points(
                uvs,
                uids,
                points_and_observations_manager.points,
                aria_track_manager.get_track_for_camera_label(stream_label),
                data.stream_id(),
                stream_label,
            )

        # If we have accumulated SLAM image visibilities for both slam images, we can compute RGB visible points
        if len(frame_set_uids) == 2 and stream_label == "camera-rgb":
            # We will now estimate uvs and ids for the RGB view
            #
            # Collect visible 3D points for the SLAM images and see if they are "visible" in the RGB frame
            all_uids = set(frame_set_uids[str(LEFT_SLAM_STREAM_ID)]).union(
                frame_set_uids[str(RIGHT_SLAM_STREAM_ID)]
            )
            # Collect online camera calibration (used for point re-projection)
            camera_calibration, device_pose = OnlineRgbCameraHelper(
                vrs_data_provider, mps_data_provider, device_time_ns
            )
            # Retrieve visible points and uids
            uvs, uids = points_and_observations_manager.get_rgb_observations(
                all_uids, camera_calibration, device_pose
            )

            #
            # Clean up the left/right slam camera uids cache for the next frame set iteration
            frame_set_uids = {}

            #
            # Update tracks and display (for the RGB view)
            rgb_stream_name = vrs_data_provider.get_label_from_stream_id(RGB_STREAM_ID)
            aria_track_manager.update_tracks_and_remove_old_observations(
                rgb_stream_name, uvs, uids
            )
            display_tracks_and_points(
                uvs,
                uids,
                points_and_observations_manager.points,
                aria_track_manager.get_track_for_camera_label(rgb_stream_name),
                RGB_STREAM_ID,
                rgb_stream_name,
            )
