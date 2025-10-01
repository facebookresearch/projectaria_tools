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
import os

from pathlib import Path
from typing import List

import rerun as rr

from projectaria_tools.core.mps import MpsDataPathsProvider
from projectaria_tools.tools.viewer_mps.rerun_viewer_mps import log_mps_to_rerun


WRIST_PALM_TIME_DIFFERENCE_THRESHOLD_NS: int = 2e8
WRIST_PALM_COLOR: List[int] = [255, 64, 0]
NORMAL_VIS_LEN = 0.03  # in meters
NORMAL_VIS_LEN_2D = 120.0  # in pixels


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vrs",
        type=str,
        help="path to VRS file",
    )
    # Add options for the MPS Artifacts
    # - They can be specified individually,
    # - Or globally as a 'mps_folder' path
    parser.add_argument(
        "--trajectory",
        nargs="+",
        type=str,
        help="path(s) to MPS closed-loop trajectory files",
    )
    parser.add_argument(
        "--points",
        nargs="+",
        type=str,
        help="path(s) to the MPS global points file",
    )
    parser.add_argument(
        "--eyegaze",
        type=str,
        help="path to the MPS eye gaze file",
    )
    parser.add_argument(
        "--hands",
        type=str,
        help="path to the MPS wrist and palm file",
    )
    parser.add_argument(
        "--hands_all",
        type=str,
        help="path to the MPS hand tracking file",
    )
    parser.add_argument(
        "--mps_folder",
        type=str,
        help="path to the MPS folder (will overwrite default value <vrs_file>/mps)",
    )

    # Add options that does not show by default, but still accessible for debugging purpose
    parser.add_argument(
        "--down_sampling_factor", type=int, default=4, help=argparse.SUPPRESS
    )
    parser.add_argument("--jpeg_quality", type=int, default=75, help=argparse.SUPPRESS)

    # If this path is set, we will save the rerun (.rrd) file to the given path
    parser.add_argument(
        "--rrd_output_path", type=str, default="", help=argparse.SUPPRESS
    )

    parser.add_argument(
        "--no_rectify_image",
        action="store_true",
        help="If set, the raw fisheye RGB images are shown without being undistorted.",
    )
    # User can choose to run the viewer in the web browser
    parser.add_argument("--web", action="store_true", help="Run viewer in web browser")

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    #
    # Gather data input
    #
    if args.vrs:
        vrs_folder_path = os.path.dirname(args.vrs)
        # - If MPS data has not been provided we try to find them automatically using default folder hierarchy
        if args.points is None and args.eyegaze is None and args.trajectory is None:
            if args.mps_folder:
                mps_data_paths_provider = MpsDataPathsProvider(args.mps_folder)
            else:
                # Try loading from default mps path (<vrs_file>/mps)
                mps_data_paths_provider = MpsDataPathsProvider(
                    str(Path(vrs_folder_path + "/mps"))
                )
            mps_data_paths = mps_data_paths_provider.get_data_paths()

            if not args.trajectory:
                # First try to load the closed loop trajectory
                if os.path.exists(mps_data_paths.slam.closed_loop_trajectory):
                    args.trajectory = [str(mps_data_paths.slam.closed_loop_trajectory)]
                # Then try to load the open loop trajectory
                elif os.path.exists(mps_data_paths.slam.open_loop_trajectory):
                    args.trajectory = [str(mps_data_paths.slam.open_loop_trajectory)]

            if not args.points and os.path.exists(mps_data_paths.slam.semidense_points):
                args.points = [str(mps_data_paths.slam.semidense_points)]

            if not args.eyegaze and os.path.exists(
                mps_data_paths.eyegaze.personalized_eyegaze
            ):
                args.eyegaze = mps_data_paths.eyegaze.personalized_eyegaze
            if not args.eyegaze and os.path.exists(
                mps_data_paths.eyegaze.general_eyegaze
            ):
                args.eyegaze = mps_data_paths.eyegaze.general_eyegaze

            if not args.hands and os.path.exists(
                mps_data_paths.hand_tracking.wrist_and_palm_poses
            ):
                args.hands = mps_data_paths.hand_tracking.wrist_and_palm_poses

            if not args.hands_all and os.path.exists(
                mps_data_paths.hand_tracking.hand_tracking_results
            ):
                args.hands_all = mps_data_paths.hand_tracking.hand_tracking_results

    mps_data_available = args.trajectory or args.points or args.eyegaze or args.hands

    print(
        f"""
    Trying to load the following list of files:
    - vrs: {args.vrs}
    - trajectory/open_loop_trajectory or closed_loop_trajectory: {args.trajectory}
    - trajectory/point_cloud: {args.points}
    - eye_gaze/general_eye_gaze: {args.eyegaze}
    - hand_tracking/wrist_and_palm_poses: {args.hands}
    - hand_tracking/hand_tracking_results: {args.hands_all}
    """
    )

    if mps_data_available is None and args.vrs is None:
        print("Nothing to display.")
        exit(1)

    # Initializing Rerun viewer
    rr.init("MPS Data Viewer", spawn=(not args.rrd_output_path and not args.web))

    # Run the viewer in the web browser or desktop app
    if args.web:
        rr.serve_web()
    else:
        rr.spawn()

    log_mps_to_rerun(
        vrs_path=args.vrs,
        trajectory_files=args.trajectory,
        points_files=args.points,
        eye_gaze_file=args.eyegaze,
        wrist_and_palm_poses_file=args.hands,
        hand_tracking_results_file=args.hands_all,
        should_rectify_image=not args.no_rectify_image,
        down_sampling_factor=args.down_sampling_factor,
        jpeg_quality=args.jpeg_quality,
        rrd_output_path=args.rrd_output_path,
    )
    if args.web:
        # Keep the server running
        try:
            while True:
                pass
        except KeyboardInterrupt:
            print("Shutting down server...")


if __name__ == "__main__":
    main()
