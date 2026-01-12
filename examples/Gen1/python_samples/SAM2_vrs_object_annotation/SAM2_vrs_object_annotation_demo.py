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
import json
import os
from typing import Any, Final, List, Optional

import cv2
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image
from projectaria_tools.core import data_provider
from projectaria_tools.core.stream_id import StreamId
from sam2.build_sam import build_sam2_video_predictor
from tqdm import tqdm

GENERATE_VIZ: Final[bool] = True
VIZ_FRAME_STRIDE: Final[float] = 10
STREAM_ID = StreamId("214-1")


def parse_args():
    parser = argparse.ArgumentParser(
        "Semi-manually annotate 2D bboxes and masks using SAM2"
    )
    parser.add_argument("--vrs", required=True, help="The path to the vrs file.")
    parser.add_argument("--output", required=True, help="The path to the output folder")
    parser.add_argument(
        "--checkpoint",
        required=True,
        help="Path to SAM2 checkpoint (example: sam2_path/checkpoints/sam2.1_hiera_large.pt)",
    )
    parser.add_argument(
        "--config",
        required=True,
        help="Path to SAM2 config (example: sam2_path/configs/sam2.1/sam2.1_hiera_l.yaml)",
    )
    parser.add_argument(
        "--max_images_per_chunk",
        required=False,
        default=300,
        type=int,
        help="We break up the processing into chunks to not exceed the RAM of the GPU. Set this to set how many images to process per chunk",
    )
    args = parser.parse_args()
    return args


def load_vrs(vrs_path: str):
    print(f"Creating vrs data provider from {vrs_path}")
    provider = data_provider.create_vrs_data_provider(vrs_path)
    if not provider:
        print("Invalid vrs data provider")
    else:
        print("Successfully created vrs data provider")
    return provider


def get_clicked_points(img_path: str):
    # Load the image
    img = cv2.imread(img_path)

    # Define a function to handle mouse click event
    def onclick(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            nonlocal current_clicked_point  # Use nonlocal to modify the variable
            current_clicked_point = (x, y)

    # Initialize clicked_point variable
    current_clicked_point = None
    clicked_points = []
    labels = []

    # Create a window and display the image
    cv2.namedWindow("Select a point on the image")
    cv2.imshow("Select a point on the image", img)

    # Connect the function to the mouse click event
    cv2.setMouseCallback("Select a point on the image", onclick)

    # Wait for the user to click
    print(
        "select a point on the image and then press:\n"
        "'p' to mark as positive, "
        "'n' to mark as negative, "
        "'d' to delete last, "
        "and 'enter' to complete adding points"
    )

    while True:
        k = cv2.waitKey(0)
        if k == 112:
            print("adding positive point prompt")
            center = (
                int(current_clicked_point[0]),
                int(current_clicked_point[1]),
            )
            cv2.circle(img, center, 5, (0, 255, 0), 2)
            clicked_points.append(current_clicked_point)
            labels.append(1)
            cv2.imshow("Select a point on the image", img)
        elif k == 110:
            print("adding negative point prompt")
            center = (
                int(current_clicked_point[0]),
                int(current_clicked_point[1]),
            )
            cv2.circle(img, center, 5, (0, 0, 255), 2)
            clicked_points.append(current_clicked_point)
            labels.append(0)
            cv2.imshow("Select a point on the image", img)
        elif k == 13:
            print("done selecting points")
            break
        elif k == 100:
            print("removing last point if available")
            if clicked_points:
                center = (
                    int(clicked_points[-1][0]),
                    int(clicked_points[-1][1]),
                )
                cv2.circle(img, center, 5, (0, 0, 0), 2)
                cv2.imshow("Select a point on the image", img)
                clicked_points = clicked_points[:-1]
                labels = labels[:-1]
        else:
            print("invalid key, options: p, n, d, enter")

    cv2.destroyAllWindows()
    return clicked_points, labels


def show_mask(mask: np.ndarray, bbox: List[int], ax: Any, random_color: bool = False):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30 / 255, 144 / 255, 255 / 255, 0.6])
    h_mask, w_mask = mask.shape[-2:]
    mask_image = mask.reshape(h_mask, w_mask, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)
    x_min, y_min, w, h = bbox
    rect = plt.Rectangle((x_min, y_min), w, h, fill=False, edgecolor="red", linewidth=2)
    ax.add_patch(rect)


def use_result() -> bool:
    while True:
        user_input = input("use this labels? (y/n):").lower()
        if user_input == "y":
            return True
        elif user_input == "n":
            return False
        else:
            print("Invalid input. Please enter 'y' or 'n'.")


def get_bbox_from_mask(mask: np.ndarray) -> Optional[List[int]]:
    # Find the indices of the non-zero elements in the mask
    nonzero_indices = np.nonzero(mask)
    if len(nonzero_indices[0]) == 0:
        return None

    # Get the minimum and maximum indices
    x_min = np.min(nonzero_indices[2])
    x_max = np.max(nonzero_indices[2])
    y_min = np.min(nonzero_indices[1])
    y_max = np.max(nonzero_indices[1])
    # Calculate the width and height of the bounding box
    w = x_max - x_min + 1
    h = y_max - y_min + 1
    return [int(x_min), int(y_min), int(w), int(h)]


def extract_images(
    vrs_path: str, output_folder: str, max_images_per_chunk: int
) -> List[str]:
    provider = load_vrs(vrs_path)
    num_data = provider.get_num_data(STREAM_ID)
    print("extracting images from vrs")
    counter_chunk = 0
    chunk_number = -1
    counter_total = 0
    output_folders = []
    current_folder = None
    for index in tqdm(range(0, num_data)):
        if counter_chunk >= max_images_per_chunk or chunk_number == -1:
            counter_chunk = 0
            chunk_number += 1
            current_folder = os.path.join(output_folder, f"chunk_{chunk_number}")
            os.makedirs(current_folder, exist_ok=True)
            output_folders.append(current_folder)

        image_data_and_record = provider.get_image_data_by_index(STREAM_ID, index)
        image_record = image_data_and_record[1]
        timestamp_ns = image_record.capture_timestamp_ns

        save_path = os.path.join(current_folder, f"{timestamp_ns}.jpg")
        counter_chunk += 1
        counter_total += 1
        image_data = image_data_and_record[0]
        image_np = image_data.to_numpy_array()
        im = Image.fromarray(image_np)
        im.save(save_path)

    print(f"extracted {counter_total} images, into {chunk_number + 1} chunks")
    return output_folders


def process_video(video_dir: str, predictor: Any):
    frame_names = [
        p
        for p in os.listdir(video_dir)
        if os.path.splitext(p)[-1] in [".jpg", ".jpeg", ".JPG", ".JPEG"]
    ]
    frame_names.sort(key=lambda p: int(os.path.splitext(p)[0]))

    inference_state = predictor.init_state(video_path=video_dir)

    # Annotate first image in a loop so user can correct until results are good
    while True:
        # get annotation of first frame
        first_image_path = os.path.join(video_dir, frame_names[0])
        points, labels = get_clicked_points(first_image_path)
        predictor.reset_state(inference_state)

        # run annotation on first frame
        _, out_obj_ids, out_mask_logits = predictor.add_new_points_or_box(
            inference_state=inference_state,
            frame_idx=0,
            obj_id=0,
            points=points,
            labels=labels,
        )

        # show the results on the current (interacted) frame
        print(
            "showing the mask on frame 0, close window and then select y or no to keep or discard"
        )
        plt.figure(figsize=(15, 15))
        plt.title("frame 0")
        plt.imshow(Image.open(first_image_path))
        mask = (out_mask_logits[0] > 0.0).cpu().numpy()
        bbox = get_bbox_from_mask(mask)
        if bbox is None:
            print("no mask found, please try again")
            continue
        show_mask(
            mask=mask,
            bbox=bbox,
            ax=plt.gca(),
        )
        plt.show()
        if use_result():
            break

    # run propagation throughout the video and collect the results in a dict
    output_bboxes = {}
    output_masks_list = {}
    for out_frame_idx, out_obj_ids, out_mask_logits in predictor.propagate_in_video(
        inference_state
    ):
        timestamp_ns = int(frame_names[out_frame_idx].split(".")[0])
        output_bboxes[timestamp_ns] = {}
        output_masks_list[timestamp_ns] = {}
        for i, out_obj_id in enumerate(out_obj_ids):
            mask = (out_mask_logits[i] > 0.0).cpu().numpy()
            nonzero_indices = np.nonzero(mask)
            mask_list = []
            xs = nonzero_indices[2]
            ys = nonzero_indices[1]
            for i in range(len(xs)):
                mask_list.append(int(xs[i]))
                mask_list.append(int(ys[i]))
            bbox = get_bbox_from_mask(mask)
            if bbox is None:
                print(f"no mask found for timestamp {timestamp_ns}")
                continue
            output_masks_list[timestamp_ns][out_obj_id] = mask_list
            output_bboxes[timestamp_ns][out_obj_id] = bbox

    bbox_path = os.path.join(video_dir, "bboxes_2d.json")
    print("saving bboxes to:", bbox_path)
    with open(bbox_path, "w") as f:
        json.dump(output_bboxes, f, indent=2)
    masks_path = os.path.join(video_dir, "masks.json")
    print("saving masks to:", masks_path)
    with open(masks_path, "w") as f:
        json.dump(output_masks_list, f, indent=2)


def visualize_results(
    video_dirs: List[str], bboxes_path: str, masks_path: str, output_folder: str
):
    if not GENERATE_VIZ:
        print("not generating visualizations since GENERATE_VIZ is False")
        return

    viz_output_folder = os.path.join(output_folder, "viz")
    os.makedirs(viz_output_folder, exist_ok=True)
    print("saving visualizations to:", viz_output_folder)

    with open(bboxes_path, "r") as f:
        bboxes = json.load(f)
    with open(masks_path, "r") as f:
        masks_list = json.load(f)

    # get image dimensions to set mask
    height = None
    width = None
    for video_dir in video_dirs:
        frame_names = [
            p
            for p in os.listdir(video_dir)
            if os.path.splitext(p)[-1] in [".jpg", ".jpeg", ".JPG", ".JPEG"]
        ]
        frame_names.sort(key=lambda p: int(os.path.splitext(p)[0]))

        if not height:
            first_image_path = os.path.join(video_dir, frame_names[0])
            img_tmp = Image.open(first_image_path)
            img_array_tmp = np.array(img_tmp)
            height = img_array_tmp.shape[0]
            width = img_array_tmp.shape[1]

        # render the segmentation results every few frames
        plt.close("all")
        for out_frame_idx in range(0, len(frame_names), VIZ_FRAME_STRIDE):
            plt.figure(figsize=(15, 15))
            plt.title(f"frame {out_frame_idx}")
            frame_name = frame_names[out_frame_idx]
            timestamp_ns = frame_name.split(".")[0]
            frame_path = os.path.join(video_dir, frame_name)
            plt.imshow(Image.open(frame_path))
            assert timestamp_ns in bboxes, f"timestamp {timestamp_ns} not in bboxes"
            assert timestamp_ns in masks_list, f"timestamp {timestamp_ns} not in masks"

            mask_list = masks_list[timestamp_ns]
            for obj_id in mask_list:
                mask = np.zeros((width, height))
                mask_list_obj = masks_list[timestamp_ns][obj_id]
                for i in range(len(mask_list_obj)):
                    if i % 2 == 0:
                        x = mask_list_obj[i]
                        y = mask_list_obj[i + 1]
                        mask[y, x] = 1
                show_mask(mask=mask, bbox=bboxes[timestamp_ns][obj_id], ax=plt.gca())
            plt.savefig(
                os.path.join(viz_output_folder, f"annotation_{timestamp_ns}.jpg")
            )


def combine_results(
    video_dirs: List[str], bboxes_path_combined: str, masks_path_combined: str
):
    print("Combining results into a single json bbox and mask file")
    bboxes_combined = {}
    masks_combined = {}
    for video_dir in video_dirs:
        bbox_path = os.path.join(video_dir, "bboxes_2d.json")
        masks_path = os.path.join(video_dir, "masks.json")
        with open(bbox_path, "r") as f:
            bboxes = json.load(f)
        with open(masks_path, "r") as f:
            masks = json.load(f)
        bboxes_combined.update(bboxes)
        masks_combined.update(masks)
    with open(bboxes_path_combined, "w") as f:
        json.dump(bboxes_combined, f, indent=2)
    with open(masks_path_combined, "w") as f:
        json.dump(masks_combined, f, indent=2)
    print("Done combining results")


def main():
    args = parse_args()

    # extract images from vrs
    if not os.path.exists(args.output):
        print("creating output folder:", args.output)
        os.makedirs(args.output)
    video_dir = os.path.join(args.output, "images")
    if not os.path.exists(video_dir):
        print("creating output folder:", video_dir)
        os.makedirs(video_dir)
    else:
        print("WARNING: output folder already exists, please clear previous results")

    video_dirs = extract_images(args.vrs, video_dir, args.max_images_per_chunk)

    # setup SAM
    if torch.cuda.is_available():
        device = torch.device("cuda")
    elif torch.backends.mps.is_available():
        device = torch.device("mps")
    else:
        device = torch.device("cpu")
    print(f"using device: {device}")
    if device.type == "cuda":
        torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
        # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
        if torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
    elif device.type == "mps":
        print(
            "\nSupport for MPS devices is preliminary. SAM 2 is trained with CUDA and might "
            "give numerically different outputs and sometimes degraded performance on MPS. "
            "See e.g. https://github.com/pytorch/pytorch/issues/84936 for a discussion."
        )

    predictor = build_sam2_video_predictor(args.config, args.checkpoint, device=device)

    # process each video (chunk)
    for video_dir in video_dirs:
        process_video(video_dir, predictor)

    masks_path = os.path.join(args.output, "masks.json")
    bboxes_path = os.path.join(args.output, "bboxes_2d.json")
    combine_results(video_dirs, bboxes_path, masks_path)
    visualize_results(video_dirs, bboxes_path, masks_path, args.output)


if __name__ == "__main__":
    main()
