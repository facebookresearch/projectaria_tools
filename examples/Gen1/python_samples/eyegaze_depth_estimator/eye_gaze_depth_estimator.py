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

# Description:
# This is a code sample of using a custom patch-matching algorithm inspired by
# SFM / stereo reconstruction techniques to perform eye
# gaze depth estimation given the yaw and pitch angles of the eye gaze. The
# algorithm is experimental and not extensively evaluated but can be used
# (at your own risk) as an alternative to the vergence-model obtained using MPS.
# Suggestions and Pull requests are welcome for any improvement.

import argparse
import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
from PIL import Image, ImageOps
from projectaria_tools.core import data_provider, mps
from projectaria_tools.core.calibration import DeviceCalibration
from projectaria_tools.core.sensor_data import TimeDomain, TimeQueryOptions
from tqdm import tqdm

GAZE_CSV_COLUMNS = [
    "tracking_timestamp_us",
    "yaw_rads_cpf",
    "pitch_rads_cpf",
    "depth_m",
    "yaw_low_rads_cpf",
    "pitch_low_rads_cpf",
    "yaw_high_rads_cpf",
    "pitch_high_rads_cpf",
    "session_uid",
]


@dataclass
class GazeWithDepthEstimate:
    """
    This class stores the mps gaze data with depth estimate
    """

    gaze: (
        mps.EyeGaze
    )  # single instances of mps.EyeGaze, mps.read_eyegaze generates List[mps.EyeGaze]
    scores: np.ndarray  # scores (correlations) corresponding to each depth choice
    depth_choices: (
        np.ndarray
    )  # depth choices in meters from which depth estimate was chosen
    depth_estimate: float  # depth estimate in meters


def get_calibs(
    provider: data_provider.VrsDataProvider, label: str = "camera-rgb"
) -> Tuple[DeviceCalibration, np.ndarray]:
    """
    Get intrinsics and extrinsics for projecting points to image sensor
    """

    device_calib = provider.get_device_calibration()
    transform_cpf_sensor_cad = device_calib.get_transform_cpf_sensor(
        label, get_cad_value=True
    )
    transform_sensor_cpf_cad = transform_cpf_sensor_cad.inverse()
    transform_sensor_cpf_cad = transform_sensor_cpf_cad.to_matrix()
    # returns None if the calibration label does not exist
    cam_calib = device_calib.get_camera_calib(label)
    return cam_calib, transform_sensor_cpf_cad


def get_image_projections(
    cpf_3d: np.ndarray, cam_calib: DeviceCalibration, transform_sensor_cpf: np.ndarray
) -> np.ndarray:
    """
    Transform 3D points in CPF to sensor coordinates
    Then project to image sensor coordinates using corresponding DeviceCalibration
    Returns (len(cpf_3d), 2) array
    """
    cpf_3d = np.append(cpf_3d, np.ones((cpf_3d.shape[0], 1)), axis=1)
    cpf_sensor = np.dot(transform_sensor_cpf, cpf_3d.T).T

    projs = []
    for pt in cpf_sensor:
        proj = cam_calib.project_no_checks(pt[:3])
        projs.append(proj)

    projs = np.array(projs).astype(np.int64)

    return projs


def export_to_csv(gazedata_list: List[GazeWithDepthEstimate], filename: str) -> None:
    """
    Export gaze with depth estimate to csv in old format with the following colums:
    """
    header = GAZE_CSV_COLUMNS

    with open(filename, "w") as f:
        f.write(",".join(header) + "\n")
        for gaze_with_depth in tqdm(gazedata_list):
            f.write(
                ",".join(
                    [
                        str(
                            int(
                                gaze_with_depth.gaze.tracking_timestamp.total_seconds()
                                * 1e6
                            )
                        ),
                        str(gaze_with_depth.gaze.yaw),
                        str(gaze_with_depth.gaze.pitch),
                        str(gaze_with_depth.depth_estimate),
                        str(gaze_with_depth.gaze.yaw_low),
                        str(gaze_with_depth.gaze.pitch_low),
                        str(gaze_with_depth.gaze.yaw_high),
                        str(gaze_with_depth.gaze.pitch_high),
                        str(gaze_with_depth.gaze.session_uid),
                    ]
                )
            )
            f.write("\n")


@dataclass
class GazeDepthEstimator:
    """
    This class computes the depth of the gaze point using RGB and SLAM streams
    using a hand crafted algorithm we call patch matching.

    The patch matching algorithm works by projecting 3D points along the gaze ray (by sampling depths)
    to the RGB and SLAM cameras. The patches are then compared using cross correlation
    to find the best match. The worst case correlation across 3 pairs (rgb-leftslam, rgb-rightslam,
    leftslam-rightslam) is taken. The depth estimate is the one with highest correlation.

    """

    provider: data_provider.VrsDataProvider
    depth_min_meters: float = 0.2  # minimum depth estimated
    depth_max_meters: float = 4.0  # maximum depth estimated
    num_depth_choices: int = 60  # number of uniform samples between min and max depth
    patch_circle_radius_pixels: int = 150  # for computing correlations
    num_initial_samples: int = 3000
    # tolerance for finding nearest images to gaze timestamp. outside of this tolerance
    # images are dropped and no depth is generated (set to nan)
    image_timestamp_tolerance_milliseconds = 100

    def __post_init__(self):
        alpha = math.pow(
            self.depth_max_meters / self.depth_min_meters,
            1.0 / (self.num_depth_choices - 1),
        )
        K = np.arange(self.num_depth_choices, dtype=float)
        self.depths = self.depth_min_meters * np.power(alpha, K)

        self.camera_matrix = {}
        self.camera_center = {}
        self.r_mat = {}
        self.t_vec = {}
        self.focal_length = {}
        self.distortion = {}
        self.camera_resolution = {}
        self.cam_calib = {}
        self.transform_sensor_cpf = {}
        try:
            for label in ["camera-slam-left", "camera-slam-right", "camera-rgb"]:
                calib, transform_sensor_cpf = get_calibs(self.provider, label)
                self.cam_calib[label] = calib
                self.transform_sensor_cpf[label] = transform_sensor_cpf
                self.r_mat[label] = transform_sensor_cpf[:3, :3]
                self.t_vec[label] = transform_sensor_cpf[:3, 3]
                self.camera_resolution[label] = calib.get_image_size()
                (
                    self.camera_matrix[label],
                    self.focal_length[label],
                    self.camera_center[label],
                    self.distortion[label],
                ) = self._get_camera_matrix_and_distortion(calib.projection_params())

        except Exception as e:
            raise Exception(f"Need all image streams to use this estimator: {e}")

    def _get_camera_matrix_and_distortion(
        self,
        camera_params: np.ndarray,
    ) -> Tuple[np.ndarray, float, np.ndarray, np.ndarray]:
        """
        parse projection parameters
        """
        focal_length = camera_params[0]
        camera_center = np.array(
            (
                camera_params[1],
                camera_params[2],
            )
        ).reshape(1, 2)
        camera_matrix = np.zeros((3, 3))
        camera_matrix[0, 0] = focal_length
        camera_matrix[1, 1] = focal_length
        camera_matrix[:2, 2] = camera_center.reshape(2)
        camera_matrix[2, 2] = 1
        distortion_coefficients = np.array(camera_params[3:9])
        return camera_matrix, focal_length, camera_center, distortion_coefficients

    def _get_valid(self, image_pts: np.ndarray, image: np.ndarray) -> np.ndarray:
        """
        check which projections are within bounds
        """
        h, w = image.shape
        pts_within_height = np.logical_and(
            np.greater_equal(image_pts[..., 0], 0), np.less(image_pts[..., 0], w)
        )
        pts_within_width = np.logical_and(
            np.greater_equal(image_pts[..., 1], 0), np.less(image_pts[..., 1], h)
        )
        return np.logical_and(pts_within_height, pts_within_width)

    def _get_gaze_point3d_choices(self, yaw: float, pitch: float) -> np.ndarray:
        """
        Get all 3d point choices for depth range
        """
        gaze_point_3d_choices = np.array(
            [mps.get_eyegaze_point_at_depth(yaw, pitch, depth) for depth in self.depths]
        )
        return gaze_point_3d_choices

    def _get_images_at_timestamp(
        self, timestamp_us: int
    ) -> List[Tuple[str, np.ndarray]]:
        """
        Get nearest rgb, slam-left and slam-right images at provided timestamp
        """
        query_timestamp_ns = int(timestamp_us * 1e3)
        cam_id_and_images = []
        for label in ["camera-slam-left", "camera-slam-right", "camera-rgb"]:
            stream_id = self.provider.get_stream_id_from_label(label)
            image_data = self.provider.get_image_data_by_time_ns(
                stream_id,
                query_timestamp_ns,
                TimeDomain.DEVICE_TIME,
                TimeQueryOptions.CLOSEST,
            )

            image_array = image_data[0].to_numpy_array()
            retrieved_timestamp = image_data[1].capture_timestamp_ns
            delta = abs(query_timestamp_ns - retrieved_timestamp)
            if delta > self.image_timestamp_tolerance_milliseconds * 1e6:
                print(f"Warning: No image found at timestamp {query_timestamp_ns}")
                return []
            if len(image_array.shape) == 3:
                pil_image = Image.fromarray(image_array)
                grayscale_img = pil_image.convert("L")
                image_array = np.array(grayscale_img)
            cam_id_and_images.append((label, image_array))
        return cam_id_and_images

    def get_depth_scores_for_gaze(
        self, gaze: mps.EyeGaze
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        yaw = gaze.yaw
        pitch = gaze.pitch
        gaze_point_3d_choices = self._get_gaze_point3d_choices(yaw, pitch)
        gaze_point_3d_choices = np.expand_dims(gaze_point_3d_choices, axis=0)

        timestamp_us = int(gaze.tracking_timestamp.total_seconds() * 1e6)
        cam_id_and_images = self._get_images_at_timestamp(timestamp_us)
        if not cam_id_and_images:
            return np.array([np.nan] * len(self.depths)), self.depths, np.nan
        scores = self._get_depth_scores_for_cpf_vectors(
            gaze_point_3d_choices, cam_id_and_images
        ).reshape(-1)
        return scores, self.depths, self.depths[np.argmax(scores)]

    def _normalize(self, image: np.ndarray, valid: np.ndarray) -> np.ndarray:
        """
        Normalize image patches Z = (X - mean) / stddev
        """
        _, num_pts = image.shape
        v = valid.astype(float)
        num_pts = np.maximum(1.0, np.sum(v, axis=-1))
        mean = np.sum(image * v, axis=-1) / num_pts
        stddev = np.sqrt(
            1e-6
            + np.sum(
                (image - mean[:, np.newaxis]) * (image - mean[:, np.newaxis]) * v,
                axis=-1,
            )
            / num_pts
        )
        return (image - mean[:, np.newaxis]) / stddev[:, np.newaxis]

    def _cross_correlation(
        self,
        image_1: np.ndarray,
        image_2: np.ndarray,
        valid: np.ndarray,
        weights: np.ndarray,
    ) -> np.ndarray:
        """
        Compute cross correlation coefficient between two image patches
        """
        normalized_image_1 = self._normalize(image_1, valid)
        normalized_image_2 = self._normalize(image_2, valid)
        return np.sum(
            normalized_image_1 * normalized_image_2 * weights * valid, axis=-1
        ) / np.sqrt(
            np.maximum(
                1.0,
                np.sum(
                    weights * valid * normalized_image_1 * normalized_image_1, axis=-1
                )
                * np.sum(
                    weights * valid * normalized_image_2 * normalized_image_2, axis=-1
                ),
            )
        )

    def _get_image_similarity_per_set(
        self,
        patch_points_in_3d: np.ndarray,
        weights: np.ndarray,
        cam_id_and_images: List[Tuple[str, np.ndarray]],
    ) -> np.ndarray:
        """
        Project patches in 3D to images
        Compute image similarity
        """
        num_images = len(cam_id_and_images)
        image_list_values_list = []

        num_sets, num_points, _ = patch_points_in_3d.shape
        valid_list = []
        for cam_id, image in cam_id_and_images:
            pil_image = Image.fromarray(image)
            equalized_img = ImageOps.equalize(pil_image)
            image = np.array(equalized_img).astype(float)
            image = (image - np.mean(image)) / np.maximum(1e-4, np.std(image))
            patch_points_in_3d = patch_points_in_3d.reshape(num_sets * num_points, 3)
            # compute image projections
            patch_points = get_image_projections(
                patch_points_in_3d,
                self.cam_calib[cam_id],
                self.transform_sensor_cpf[cam_id],
            )
            # reshape
            patch_points = patch_points.reshape(num_sets, num_points, 2)

            # indicate which projections are valid i.e. within image bounds
            valid = self._get_valid(patch_points, image)

            pi = patch_points * np.expand_dims(valid, axis=-1)
            image_list_values_list.append(image[pi[:, :, 1], pi[:, :, 0]].astype(float))
            valid_list.append(valid)

        # compute image similarity (cross correlation) across all image pairs (slam_left, slam_right, rgb)
        all_sims = []
        for i in range(num_images):
            for j in range(i + 1, num_images):
                valid = np.logical_and(valid_list[i], valid_list[j])
                valid = np.logical_and(valid, np.greater(weights, 0))
                sim = self._cross_correlation(
                    image_list_values_list[i],
                    image_list_values_list[j],
                    valid,
                    weights,
                )
                num_valid = np.sum(valid.astype(float), axis=-1)
                # only consider points with at least 1/15th of initial samples as valid projections
                sim = sim * np.greater_equal(
                    num_valid, int(self.num_initial_samples / 15)
                )
                all_sims.append(sim)

        return np.min(np.stack(all_sims, axis=0), axis=0)

    def _get_depth_scores_for_cpf_vectors(
        self, cpf_vecs: np.ndarray, cam_id_and_images: List[Tuple[str, np.ndarray]]
    ) -> np.ndarray:
        """
        Compute image similarity from patch points in 3D, with weights computed based on distance from patch center
        """
        num_cpf_vectors = cpf_vecs.shape[0]
        num_depths = cpf_vecs.shape[1]
        if len(cam_id_and_images) == 0 or num_cpf_vectors == 0:
            return np.zeros((num_cpf_vectors, num_depths))

        # get focal length of the first image in pixels
        focal_length_pixels = self.focal_length[cam_id_and_images[0][0]]
        centroids = cpf_vecs[:, :, np.newaxis, :]
        distance_from_center = self.patch_circle_radius_pixels * np.sqrt(
            np.random.rand(self.num_initial_samples)
        )
        theta = np.random.rand(self.num_initial_samples) * 2 * math.pi
        patch_xy_coords = np.stack(
            [
                distance_from_center * np.cos(theta),
                distance_from_center * np.sin(theta),
                np.zeros_like(theta),
            ],
            axis=-1,
        )
        depths = cpf_vecs[..., 2]

        # scale the patch coordinates by 1 / focal length
        patch_points_in_3d = (
            centroids
            + patch_xy_coords.reshape(1, 1, self.num_initial_samples, 3)
            * depths.reshape(num_cpf_vectors, num_depths, 1, 1)
            / focal_length_pixels
        )

        # compute weights based on distance from patch center. correlations will be weighted based on these.
        weights = np.exp(
            -2 * np.square(distance_from_center / self.patch_circle_radius_pixels)
        )
        weights = np.reshape(weights, (1, self.num_initial_samples))

        weights = weights[:, np.newaxis, :] * np.ones(
            (num_cpf_vectors, num_depths, self.num_initial_samples)
        )
        patch_points_in_3d = patch_points_in_3d.reshape(
            num_cpf_vectors * num_depths, self.num_initial_samples, 3
        )
        weights = weights.reshape(
            num_cpf_vectors * num_depths, self.num_initial_samples
        )
        return self._get_image_similarity_per_set(
            patch_points_in_3d.reshape(
                num_cpf_vectors * num_depths, self.num_initial_samples, 3
            ),
            weights.reshape(num_cpf_vectors * num_depths, self.num_initial_samples),
            cam_id_and_images,
        ).reshape(num_cpf_vectors, num_depths)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compute depth scores for eye gaze data"
    )
    parser.add_argument(
        "--vrs_file",
        type=str,
        required=True,
        help="path to local vrs file",
    )
    parser.add_argument(
        "--eyegaze_data_file",
        type=str,
        required=True,
        help="Path to file containing eye gaze data",
    )
    parser.add_argument(
        "--depth_min_meters",
        type=float,
        default=0.2,
        help="minimum depth estimated",
    )
    parser.add_argument(
        "--depth_max_meters",
        type=float,
        default=4.0,
        help="maximum depth estimated",
    )
    parser.add_argument(
        "--num_depth_choices",
        type=int,
        default=60,
        help="number of depth choices",
    )
    parser.add_argument(
        "--patch_circle_radius_pixels",
        type=int,
        default=150,
        help="for computing correlations",
    )
    parser.add_argument(
        "--num_initial_samples",
        type=int,
        default=3000,
        help="number of initial samples",
    )
    parser.add_argument(
        "--image_timestamp_tolerance_milliseconds",
        type=int,
        default=100,
        help="tolerance for time delta when retrieving closest images to gaze timestamp",
    )
    parser.add_argument(
        "--output_csv_filepath",
        type=str,
        required=False,
        help="Path to output csv file",
    )

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    # get data provider
    provider = data_provider.create_vrs_data_provider(args.vrs_file)

    # read eye gaze data
    gaze_data = mps.read_eyegaze(args.eyegaze_data_file)

    # initialize depth estimator
    depth_estimator = GazeDepthEstimator(
        provider=provider, num_depth_choices=args.num_depth_choices
    )

    # get depth scores for each gaze
    gaze_data_list: List[GazeWithDepthEstimate] = []
    for gaze in tqdm(gaze_data):
        (scores, depth_choices, depth_est) = depth_estimator.get_depth_scores_for_gaze(
            gaze
        )
        gaze_with_depth_estimate = GazeWithDepthEstimate(
            gaze, scores, depth_choices, depth_est
        )
        gaze_data_list.append(gaze_with_depth_estimate)

    # print estimates

    for gaze_with_depth in gaze_data_list:
        print(
            f"Timestamp: {gaze_with_depth.gaze.tracking_timestamp} Yaw: {gaze_with_depth.gaze.yaw} Pitch: {gaze_with_depth.gaze.pitch} Depth Estimate: {gaze_with_depth.depth_estimate}"
        )

    if args.output_csv_filepath is not None:
        export_to_csv(gaze_data_list, args.output_csv_filepath)
        gaze_output_with_depth = mps.read_eyegaze(args.output_csv_filepath)
        print(f"Read {len(gaze_output_with_depth)} gaze estimates from output csv file")


if __name__ == "__main__":
    main()
