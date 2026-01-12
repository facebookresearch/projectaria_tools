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

import numpy as np
import rerun as rr
from PIL import Image
from projectaria_tools.core import mps
from projectaria_tools.core.mps.utils import (
    filter_points_from_confidence,
    filter_points_from_count,
    get_gaze_vector_reprojection,
)
from projectaria_tools.core.sensor_data import TimeDomain, TimeQueryOptions
from projectaria_tools.core.sophus import SE3
from projectaria_tools.core.stream_id import StreamId
from projectaria_tools.projects.aea import AriaEverydayActivitiesDataProvider
from projectaria_tools.utils.calibration_utils import (
    rotate_upright_image_and_calibration,
    undistort_image_and_calibration,
)
from projectaria_tools.utils.rerun_helpers import AriaGlassesOutline, ToTransform3D
from tqdm import tqdm


# Define global variables
RGB_STREAM_ID = StreamId("214-1")
POINT_COLOR = [200, 200, 200]
TRAJECTORY_COLORS = [[191, 255, 191], [191, 191, 255]]
GAZE_COLOR = [200, 0, 0]
MAX_POINT_CLOUD_POINTS = 500_000


# log static data that does not change over time such as trajectory and point cloud
def logStaticData(
    aea_data_provider: AriaEverydayActivitiesDataProvider,
    index: int,
    down_sampling_factor: float,
):
    rgb_stream_label = aea_data_provider.vrs.get_label_from_stream_id(RGB_STREAM_ID)
    device_calibration = aea_data_provider.vrs.get_device_calibration()
    rgb_camera_calibration = device_calibration.get_camera_calib(rgb_stream_label)

    # Log Point Cloud (reduce point count for display)
    if (
        aea_data_provider.has_mps_data()
        and aea_data_provider.mps.has_semidense_point_cloud()
    ):
        # Filter out low confidence points
        points_data = filter_points_from_confidence(
            aea_data_provider.mps.get_semidense_point_cloud()
        )
        # Down sample points
        points_data_down_sampled = filter_points_from_count(
            points_data, MAX_POINT_CLOUD_POINTS
        )
        # Retrieve point position
        point_positions = [it.position_world for it in points_data_down_sampled]
        rr.log(
            f"world/points_{index}",
            rr.Points3D(point_positions, colors=POINT_COLOR, radii=0.002),
            static=True,
        )

    # Log device trajectory (reduce sample count for display)
    if (
        aea_data_provider.has_mps_data()
        and aea_data_provider.mps.has_closed_loop_poses()
    ):
        timestamps_vec = aea_data_provider.vrs.get_timestamps_ns(
            RGB_STREAM_ID, TimeDomain.DEVICE_TIME
        )

        device_trajectory = []
        for time_ns in timestamps_vec:
            maybePose = aea_data_provider.mps.get_closed_loop_pose(
                time_ns, TimeQueryOptions.BEFORE
            )
            if maybePose:
                device_trajectory.append(
                    maybePose.transform_world_device.translation()[0]
                )

        device_trajectory = device_trajectory[0::10]

        rr.log(
            f"world/device_trajectory_{index}",
            rr.LineStrips3D(
                device_trajectory, colors=TRAJECTORY_COLORS[index], radii=0.01
            ),
            static=True,
        )
    if (
        aea_data_provider.has_mps_data()
        and aea_data_provider.mps.has_closed_loop_poses()
    ):
        rr.log(
            f"world/device_{index}/{rgb_stream_label}",
            rr.Pinhole(
                resolution=[
                    int(
                        rgb_camera_calibration.get_image_size()[0]
                        / down_sampling_factor
                    ),
                    int(
                        rgb_camera_calibration.get_image_size()[1]
                        / down_sampling_factor
                    ),
                ],
                focal_length=float(
                    rgb_camera_calibration.get_focal_lengths()[0] / down_sampling_factor
                ),
            ),
            static=True,
        )
    # Log Aria Glasses Outline
    aria_glasses_point_outline = AriaGlassesOutline(device_calibration)
    rr.log(
        f"world/device_{index}/glasses_outline",
        rr.LineStrips3D([aria_glasses_point_outline]),
        static=True,
    )


# Log instance data at different timestamp such as images from each sensor, eyegaze, and speech data
def logInstanceData(
    aea_data_provider: AriaEverydayActivitiesDataProvider,
    index: int,
    time_domain: TimeDomain,
    timestamp_ns: int,
    undistort: bool,
    rotate_image: bool,
    down_sampling_factor: float,
    jpeg_quality: float,
):
    device_time_ns = timestamp_ns
    if time_domain is TimeDomain.TIME_CODE:
        rr.set_time_nanos("timecode_ns", timestamp_ns)

        device_time_ns = aea_data_provider.vrs.convert_from_timecode_to_device_time_ns(
            timestamp_ns
        )
    rr.set_time_nanos("device_time_ns", device_time_ns)
    rr.set_time_sequence("timestamp", device_time_ns)

    rgb_stream_label = aea_data_provider.vrs.get_label_from_stream_id(RGB_STREAM_ID)
    device_calibration = aea_data_provider.vrs.get_device_calibration()
    rgb_camera_calibration = device_calibration.get_camera_calib(rgb_stream_label)

    image = aea_data_provider.vrs.get_image_data_by_time_ns(
        RGB_STREAM_ID, device_time_ns, TimeDomain.DEVICE_TIME, TimeQueryOptions.BEFORE
    )
    if image is None or image[0].is_valid() is False:
        return

    image_display = image[0].to_numpy_array()
    updated_camera_calibration = rgb_camera_calibration
    new_resolution = updated_camera_calibration.get_image_size() / down_sampling_factor
    new_resolution = new_resolution.astype(int)
    updated_camera_calibration = updated_camera_calibration.rescale(
        new_resolution, 1.0 / down_sampling_factor
    )
    image_display = Image.fromarray(image_display)
    image_display = image_display.resize((new_resolution[0], new_resolution[1]))
    image_display = np.array(image_display)

    if undistort:
        image_display, updated_camera_calibration = undistort_image_and_calibration(
            image_display,
            updated_camera_calibration,
        )

    if rotate_image:
        image_display, updated_camera_calibration = (
            rotate_upright_image_and_calibration(
                image_display, updated_camera_calibration
            )
        )

    rr.log(
        f"world/device_{index}/{rgb_stream_label}/image",
        rr.Image(image_display).compress(jpeg_quality=jpeg_quality),
    )

    T_world_device = SE3()

    # draw current camera position
    if (
        aea_data_provider.has_mps_data()
        and aea_data_provider.mps.has_closed_loop_poses()
    ):
        pose_info = aea_data_provider.mps.get_closed_loop_pose(
            device_time_ns, TimeQueryOptions.CLOSEST
        )

        if pose_info:
            T_world_device = pose_info.transform_world_device
            rr.log(
                f"world/device_{index}/",
                ToTransform3D(
                    T_world_device,
                    False,
                ),
            )

            rr.log(
                f"world/device_{index}/{rgb_stream_label}",
                ToTransform3D(
                    updated_camera_calibration.get_transform_device_camera(),
                    False,
                ),
            )

    # draw current eye gaze ray and projection
    if aea_data_provider.has_mps_data() and aea_data_provider.mps.has_general_eyegaze():
        depth_m = 1.0  # Select a fixed depth of 1m
        eye_gaze = aea_data_provider.mps.get_general_eyegaze(
            device_time_ns, TimeQueryOptions.CLOSEST
        )
        if eye_gaze:
            # Compute eye_gaze vector at depth_m reprojection in the image
            depth_m = eye_gaze.depth or depth_m
            gaze_vector_in_cpf = mps.get_eyegaze_point_at_depth(
                eye_gaze.yaw, eye_gaze.pitch, depth_m
            )

            T_device_CPF = device_calibration.get_transform_device_cpf()

            # use recommended function for computing gaze projection
            gaze_projection = get_gaze_vector_reprojection(
                eye_gaze,
                rgb_stream_label,
                device_calibration,
                updated_camera_calibration,
                depth_m,
                rotate_image,
            )
            rr.log(
                f"world/device_{index}/{rgb_stream_label}/image/eye-gaze-projection",
                rr.Points2D(
                    gaze_projection, colors=[TRAJECTORY_COLORS[index]], radii=8
                ),
            )

            # Draw EyeGaze vector
            rr.log(
                f"world/device_{index}/eye-gaze",
                rr.Arrows3D(
                    origins=[T_device_CPF @ [0, 0, 0]],
                    vectors=[T_device_CPF @ gaze_vector_in_cpf],
                    colors=[GAZE_COLOR],
                ),
            )

    # log text
    if aea_data_provider.has_speech_data():
        sentence = aea_data_provider.speech.get_sentence_data_by_timestamp_ns(
            device_time_ns, TimeQueryOptions.BEFORE
        )
        image_shape = np.shape(image_display)
        if sentence:
            if (
                device_time_ns >= sentence.start_timestamp_ns
                and device_time_ns < sentence.end_timestamp_ns
            ):
                rr.log(
                    f"world/device_{index}/{rgb_stream_label}/image/text",
                    rr.Points2D(
                        positions=[image_shape[0] / 2.0, image_shape[1] - 40],
                        radii=0,
                        labels=sentence,
                        colors=[TRAJECTORY_COLORS[index]],
                    ),
                )
            else:
                rr.log(
                    f"world/device_{index}/{rgb_stream_label}/image/text",
                    rr.Clear(recursive=False),
                )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--path",
        type=str,
        required=True,
        nargs="+",
        help="Path to multiple AEA sequence root direction (e.g. '--path /path/loc1_script2_seq4_rec1 /path/loc1_script2_seq4_rec2').",
    )
    parser.add_argument(
        "--undistort",
        action="store_true",
        default=False,
        help="Undistort image to pinhole camera model.",
    )
    parser.add_argument(
        "--rotate-image",
        action="store_true",
        default=False,
        help="Undistort and rotate image to upright viewing position.",
    )
    parser.add_argument("--down_sampling_factor", type=int, default=4)

    # Add options that does not show by default, but still accessible for debugging purpose
    parser.add_argument("--jpeg_quality", type=int, default=75, help=argparse.SUPPRESS)
    parser.add_argument(
        "--memory-limit", type=str, default="100%", help=argparse.SUPPRESS
    )

    # If this path is set, we will save the rerun (.rrd) file to the given path
    parser.add_argument(
        "--rrd_output_path", type=str, default="", help=argparse.SUPPRESS
    )

    return parser.parse_args()


def main():
    args = parse_args()

    paths = args.path
    aea_data_provider = []

    if len(paths) > 2:
        print(
            "AEA dataset only provide pairs of simultaneous recording. Please do submit only two path"
        )
        exit(1)

    for path in paths:
        aea_data_provider.append(AriaEverydayActivitiesDataProvider(path))

    # Initializing ReRun viewer
    rr.init("AEA Viewer", spawn=(not args.rrd_output_path))
    if args.rrd_output_path:
        print(f"Saving .rrd file to {args.rrd_output_path}")
        rr.save(args.rrd_output_path)

    time_domain = TimeDomain.DEVICE_TIME
    total_time_ns = []
    for index in range(0, np.size(paths)):
        logStaticData(aea_data_provider[index], index, args.down_sampling_factor)
        if aea_data_provider[index].vrs.supports_time_domain(
            RGB_STREAM_ID, TimeDomain.TIME_CODE
        ):
            time_domain = TimeDomain.TIME_CODE
        time_vec = aea_data_provider[index].vrs.get_timestamps_ns(
            RGB_STREAM_ID, time_domain
        )
        total_time_ns = time_vec

    # sort the vec
    total_time_ns = np.array(list(set(total_time_ns)))
    total_time_ns = np.sort(total_time_ns)

    for time_ns in tqdm(total_time_ns):
        for index in range(0, np.size(paths)):
            logInstanceData(
                aea_data_provider[index],
                index,
                time_domain,
                time_ns,
                args.undistort,
                args.rotate_image,
                args.down_sampling_factor,
                args.jpeg_quality,
            )


if __name__ == "__main__":
    main()
