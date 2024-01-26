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
import time

import numpy as np
import rerun as rr

import torch

from efficient_sam.build_efficient_sam import (
    build_efficient_sam_vits,
    build_efficient_sam_vitt,
)
from PIL import Image
from projectaria_tools.core import data_provider
from projectaria_tools.core.mps import MpsDataPathsProvider, read_eyegaze
from projectaria_tools.core.mps.utils import (
    get_gaze_vector_reprojection,
    get_nearest_eye_gaze,
)
from projectaria_tools.core.sensor_data import SensorDataType, TimeDomain
from projectaria_tools.core.stream_id import StreamId
from torchvision import transforms
from tqdm import tqdm

# Sample:
# Demonstrate how to run EfficientSAM Tiny and Small on RGB image with EyeGaze image reprojection as prompt
#
# 1. Loop over the RGB frames
# 2. run Sam with EyeGaze re-projection prompt
# 3. Display RGB image, EyeGaze image reprojection, and EfficientSam mask output
# 4. Report some statistics on EfficientSAM time inference and mask area size
# Note:
# Since SAM is not rotation invariant, we rotate the image and the eye_gaze projection before feeding it into SAM


def rotatePoints(x: list, y: list, image_width: int):
    """
    Rotate 2d points Clock wise by 90 degrees
    """
    return image_width - 1 - y, x


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vrs",
        type=str,
        required=True,
        help="path to VRS file",
    )
    parser.add_argument(
        "--eyegaze",
        type=str,
        required=False,
        help="path to the MPS eye gaze file",
    )
    parser.add_argument(
        "--model_list",
        default="small tiny",
        nargs="+",
        required=False,
        type=str,
        help="List of models to run (either one of both), 'small', 'tiny' and 'small tiny' are supported",
    )

    # Add options that does not show by default, but still accessible for debugging purpose
    parser.add_argument(
        "--down_sampling_factor", type=int, default=4, help=argparse.SUPPRESS
    )
    parser.add_argument("--jpeg_quality", type=int, default=75, help=argparse.SUPPRESS)

    return parser.parse_args()


def main():
    args = parse_args()

    #
    # Gather data input
    # - If MPS data has not been provided we try to find them automatically using default folder hierarchy
    #
    vrs_folder_path = os.path.dirname(args.vrs)

    # Retrieve which eye gaze file to use
    if args.eyegaze:
        # If specified from command line, we use it
        eyegaze_data = read_eyegaze(args.eyegaze)
    else:
        # Else we don't have any eye gaze path: we try to auto load it from its default possible MPS location
        mps_path_provider = MpsDataPathsProvider(vrs_folder_path)
        eyegaze_data = read_eyegaze(
            mps_path_provider.get_data_paths().eyegaze.general_eyegaze_filepath
        )
    # If no eye gaze data was found, we exit
    if len(eyegaze_data) == 0:
        print(
            f"Unable to load the provided '{args.eyegaze}', ensure you have MPS eye gaze along your VRS file or specify one."
        )
        exit(1)

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    print(
        f"""
    Info: Running with the following configuration:
    - vrs: {args.vrs}
    - eye_gaze: {args.eyegaze}
    Inference:
    - Inference will run on device: {device}
    - Inference will run SAM models: {args.model_list}
    """
    )

    #
    # ML model loading and config
    torch.set_grad_enabled(False)
    models = {}
    if "tiny" in args.model_list:
        models["efficientsam_t"] = build_efficient_sam_vitt().to(device)
    if "small" in args.model_list:
        models["efficientsam_s"] = build_efficient_sam_vits().to(device)
    if len(models) == 0:
        print(
            f"No valid model name provided in --model_list={args.model_list}, please provide one of 'tiny', 'small' or 'small tiny'"
        )
        exit(1)

    running_mask_area_average = {}
    for model_name, _model in models.items():
        running_mask_area_average[model_name] = []

    # Configure the VRS data provider (provide only RGB frames)
    provider = data_provider.create_vrs_data_provider(args.vrs)

    rgb_stream_id = StreamId("214-1")
    rgb_stream_label = provider.get_label_from_stream_id(rgb_stream_id)

    deliver_option = provider.get_default_deliver_queued_options()
    deliver_option.deactivate_stream_all()
    deliver_option.activate_stream(rgb_stream_id)  # RGB Stream Id
    rgb_frame_count = provider.get_num_data(rgb_stream_id)

    # Retrieve calibration data
    device_calibration = provider.get_device_calibration()
    rgb_camera_calibration = device_calibration.get_camera_calib(rgb_stream_label)

    # Initializing Rerun viewer
    rr.init("SAM Eye Gaze Prompting Viewer", spawn=True)
    # Configure class_id == 255 to be green for EfficientSAM masks visualization
    rr.log(
        "/",
        rr.AnnotationContext([(255, "green", (0, 255, 0))]),
        timeless=True,
    )

    progress_bar = tqdm(total=rgb_frame_count)
    img = None
    iteration = 0
    # Iterate over the data and LOG data as we see fit
    for data in provider.deliver_queued_sensor_data(deliver_option):
        device_time_ns = data.get_time_ns(TimeDomain.DEVICE_TIME)
        rr.set_time_nanos("device_time", device_time_ns)
        rr.set_time_sequence("timestamp", device_time_ns)
        progress_bar.update(1)
        iteration += 1

        #
        # Retrieve the image and display it
        #
        if data.sensor_data_type() == SensorDataType.IMAGE:
            img = Image.fromarray(data.image_data_and_record()[0].to_numpy_array())
            img = img.resize(
                (
                    (int)(img.width / args.down_sampling_factor),
                    (int)(img.height / args.down_sampling_factor),
                ),
                Image.NEAREST,
            )
            img = img.rotate(-90)
            img = np.array(img)

        #
        # Eye Gaze (vector and image re-projection)
        #
        gaze_projection = np.array([])
        if eyegaze_data:
            depth_m = 1.0  # Select a fixed depth of 1m
            eye_gaze = get_nearest_eye_gaze(eyegaze_data, device_time_ns)
            if eye_gaze:
                # Compute eye_gaze vector at depth_m re-projection in the image
                gaze_projection = (
                    get_gaze_vector_reprojection(
                        eye_gaze,
                        rgb_stream_label,
                        device_calibration,
                        rgb_camera_calibration,
                        depth_m,
                    )
                    / args.down_sampling_factor
                )

                x, y = rotatePoints(
                    gaze_projection[0], gaze_projection[1], img.shape[0]
                )
                gaze_projection = np.array([x, y]).T

        #
        # Process SAM
        # If an image is available, process SAM with the eye gaze data as point prompt
        #
        for model_name, model in models.items():

            # Log image
            rr.log(
                f"{model_name}/{rgb_stream_label}",
                rr.Image(img).compress(jpeg_quality=args.jpeg_quality),
            )

            # Log gaze 2d projection
            rr.log(
                f"{model_name}/{rgb_stream_label}/eye-gaze_projection",
                rr.Points2D(gaze_projection, radii=8),
            )

            if gaze_projection.any():
                img_tensor = transforms.ToTensor()(img.copy())

                input_points = gaze_projection.astype(int)
                input_labels = np.array([1])

                pts_sampled = torch.reshape(torch.tensor(input_points), [1, 1, -1, 2])
                pts_labels = torch.reshape(torch.tensor(input_labels), [1, 1, -1])

                # Running inference and monitor timing
                start_time = time.time()
                predicted_logits, predicted_iou = model(
                    img_tensor[None, ...].to(device),
                    pts_sampled.to(device),
                    pts_labels.to(device),
                )
                end_time = time.time()
                elapsed_time = (end_time - start_time) * 1000  # seconds to ms
                mask = (
                    torch.ge(predicted_logits[0, 0, 0, :, :], 0).cpu().detach().numpy()
                )
                mask_area = sum(sum(mask))

                #
                # Log scalar data
                # - mask_area
                # - inference_time
                rr.log(f"mask_area/{model_name}", rr.TimeSeriesScalar(mask_area))
                rr.log(
                    f"inference_time/{model_name}", rr.TimeSeriesScalar(elapsed_time)
                )

                # Log std_dev area (Update mask statistics and compute std_dev)
                running_mask_area_average[model_name].append(mask_area)
                rr.log(
                    f"mask_area/{model_name}/std_dev",
                    rr.TimeSeriesScalar(np.std(running_mask_area_average[model_name])),
                )

                # Log SAM mask
                masked_image_black_white = mask[:, :, None].astype(np.uint8) * 255
                rr.log(
                    f"{model_name}/{rgb_stream_label}/mask",
                    rr.SegmentationImage(masked_image_black_white),
                )


if __name__ == "__main__":
    main()
