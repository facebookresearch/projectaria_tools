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
import zipfile

import numpy as np
from projectaria_tools.projects.adt.utils import (
    compute_mssd,
    get_3d_bounding_box,
    get_diameters,
    get_symmetries,
    get_timed_poses,
    get_vertices,
    voc_ap,
)
from tqdm import tqdm


THRESHOLDS = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]


def evaluate(
    test_annotation_file, user_annotation_file, sequences, prototypes_of_interest
):
    """
    Evaluate the given prediction against annotations.

    This function computes the MSSD-based mAPs between the uploaded predictions
    and the ground truth annotations. An object pose is considered correct with
    respect to the MMSD if MMSD is below a threshold. The fraction of annotated
    object instances, for which a correct pose is estimated, is referred to as
    recall. The fraction of predicted object instances, for which a correct pose
    is estimated, is referred to as precision. An Average Precision (AP) can
    then be computed from precisions and recalls. A mean Average Precision (mAP)
    is defined as the average of the AP calculated for all prototypes, multiple
    thresholds from 5% to 50% of the object diameter with a step of 5%. We use
    the mAP as the performance score to rank all participants’ algorithms.

    Algorithm:
    We introduce the concept of prototype, a set of objects sharing the same shape
    and textures. Our evaluation algorithm evaluates the performance per prototype
    and later average across them.
    For each sequence, each frame, each threshold, and each prototype,
    1. Compute the MSSD of every combination between predicted and ground truth 3D
       bounding boxes.
    2. Select the prediction with the minimum box-based MSSD as the associated
       prediction.
    3. Compute the mesh-based MSSD
    4. If the computed mesh-based MSSD is less than the threshold, it is counted as
       a true positive, otherwise a false positive.
    Then the precision, recall, and AP are computed across all frames, all sequences.
    Finally, compute the mAP averaging thresholds and prototypes.
    Per threshold mAP and averaged mAP will be reported in the leaderboard.

    Args:
        test_annotation_file: path to the ground truth annotation file.
        user_annotation_file: path to the user uploaded prediction file.
        sequences: The list of sequences to evaluate.
        prototypes_of_interest: The list of prototypes to evaluate.
    Returns: A dictionary following eval.ai's requirement, for showing in the
        leaderboard.
    """
    submission_archive = zipfile.ZipFile(
        user_annotation_file, mode="r", compression=zipfile.ZIP_BZIP2
    )
    annotation_archive = zipfile.ZipFile(
        test_annotation_file, mode="r", compression=zipfile.ZIP_BZIP2
    )

    all_vertices = get_vertices(annotation_archive)
    all_bboxes3d = get_3d_bounding_box(annotation_archive)
    all_symmetries = get_symmetries(annotation_archive)
    all_diameters = get_diameters(annotation_archive)

    false_positive = {
        threshold: {prototype: [] for prototype in prototypes_of_interest}
        for threshold in THRESHOLDS
    }
    true_positive = {
        threshold: {prototype: [] for prototype in prototypes_of_interest}
        for threshold in THRESHOLDS
    }
    num_gt = {prototype: 0 for prototype in prototypes_of_interest}

    # Go through every sequence to compute the MSSD predictions and recalls
    for seq_name in tqdm(sequences):
        annotated_poses = get_timed_poses(annotation_archive, seq_name)
        predicted_poses = get_timed_poses(submission_archive, seq_name)
        for timestamp, anno_poses in annotated_poses.items():
            pred_poses = predicted_poses.get(timestamp, [])
            for proto in anno_poses:
                if proto not in prototypes_of_interest:
                    continue
                symmetry = all_symmetries[proto]
                vertices = all_vertices[proto]
                diameter = all_diameters[proto]
                bbox3d = all_bboxes3d[proto]
                if proto not in pred_poses:
                    num_gt[proto] += len(anno_poses[proto])
                    continue
                tp = {
                    threshold: [0] * len(pred_poses[proto]) for threshold in THRESHOLDS
                }
                fp = {
                    threshold: [1] * len(pred_poses[proto]) for threshold in THRESHOLDS
                }
                # Only one or zero instance per prototype.
                for anno_pose in anno_poses[proto]:
                    matched_idx = None
                    matched_mssd_bbox3d = None
                    for pred_idx, pred_pose in enumerate(pred_poses[proto]):
                        mssd = compute_mssd(pred_pose, anno_pose, symmetry, bbox3d)
                        if matched_mssd_bbox3d is None or mssd < matched_mssd_bbox3d:
                            matched_mssd_bbox3d = mssd
                            matched_idx = pred_idx
                    if matched_idx is not None:
                        matched_mssd = compute_mssd(
                            pred_pose, anno_pose, symmetry, vertices
                        )
                        for threshold in tp:
                            if matched_mssd < threshold * diameter:
                                tp[threshold][matched_idx] = 1
                                fp[threshold][matched_idx] = 0
                for threshold in THRESHOLDS:
                    true_positive[threshold][proto] += tp[threshold]
                    false_positive[threshold][proto] += fp[threshold]
                num_gt[proto] += len(anno_poses[proto])

    average_precision = {
        threshold: {proto: 0 for proto, num in num_gt.items() if num != 0}
        for threshold in THRESHOLDS
    }
    for threshold in average_precision:
        for proto in average_precision[threshold]:
            fp = np.cumsum(false_positive[threshold][proto])
            tp = np.cumsum(true_positive[threshold][proto])
            recalls = tp / num_gt[proto]
            precisions = tp / np.maximum(tp + fp, np.finfo(np.float64).eps)
            average_precision[threshold][proto] = voc_ap(recalls, precisions)
    output = {}
    output["result"] = [{"test_split": {}}]
    for threshold in THRESHOLDS:
        output["result"][0]["test_split"][f"mAP@{threshold:.02f}"] = np.mean(
            list(average_precision[threshold].values())
        )
    output["result"][0]["test_split"]["mAP"] = np.mean(
        list(output["result"][0]["test_split"].values())
    )
    submission_archive.close()
    annotation_archive.close()
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Evaluate the submissions by the MSSD-based mAPs.")
    parser.add_argument(
        "--test-annotation-file", help="The path to the ground truth annotation file."
    )
    parser.add_argument(
        "--user-annotation-file", help="The path to the user uploaded submission file."
    )
    parser.add_argument(
        "--phase-codename",
        help="Name of the phase to evaluate. Use 'example' for the example sequence.",
        default="example",
    )
    parser.add_argument(
        "--sequences",
        nargs="+",
        help="The list of sequences to evaluate on.",
        default=[
            "Apartment_release_challenge_example",
        ],
    )
    parser.add_argument(
        "--prototypes-of-interest",
        nargs="+",
        help="The list of prototypes to evaluate on.",
        default=[
            "BlackCeramicMug",
            "ChoppingBoard",
            "CoffeeCanisterLarge",
            "CoffeeCanisterSmall",
            "FakeButcherKnife",
            "UtilityCart",
            "WhiteFlatwareTray",
            "WhiteLiddedTrashBin",
            "WhiteTrashBin",
            "WhiteUtensilTray",
            "WhiteVase",
            "WoodenBowl",
            "WoodenFork",
            "WoodenSpoon",
        ],
    )
    args = parser.parse_args()
    output = evaluate(
        args.test_annotation_file,
        args.user_annotation_file,
        args.sequences,
        args.prototypes_of_interest,
    )
    print(output)
