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

import json
import logging
import traceback
from asyncio import Semaphore
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Union

from projectaria_tools.core import vrs_health_check as vhc

from .common import Config, CustomAdapter, to_proc
from .constants import ConfigKey, ConfigSection
from .types import AriaRecording, MpsFeature

logger = logging.getLogger(__name__)

config = Config.get()


@dataclass
class Summary:
    """
    Summary of SLAM specific VRS health checks.
    """

    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    infos: List[str] = field(default_factory=list)

    def error(self, msg: str) -> None:
        self.errors.append(msg)

    def warn(self, msg: str) -> None:
        self.warnings.append(msg)

    def info(self, msg: str) -> None:
        self.infos.append(msg)

    def to_file(self, output: Path) -> None:
        with open(output, "w") as f:
            json.dump(asdict(self), f, indent=2)


def run_vrs_health_check_slam(
    healthcheck_output: Optional[Path], summary_output: Optional[Path]
) -> Summary:
    summary = Summary()
    logger_ = CustomAdapter(logger, {"vhc": healthcheck_output})

    try:
        # ========== run actual checks ==========
        with open(healthcheck_output, "r") as f:
            try:
                metrics_json = json.load(f)
            except BaseException:
                message = "Failed to load and parse .json file {}".format(
                    healthcheck_output
                )
                summary.error(message)
                logger_.error(message)
                return

            camera_names = ["Camera Data (SLAM) #1", "Camera Data (SLAM) #2"]
            imu_names = ["IMU Data (SLAM) #1", "IMU Data (SLAM) #2"]
            align_pairs = ["Camera Data (SLAM) #1 - Camera Data (SLAM) #2"]

            #  check cameras
            for cam in camera_names:
                if cam not in metrics_json or type(metrics_json[cam]) is not dict:
                    message = "Required camera stream {} not found or invalid".format(
                        cam
                    )
                    summary.error(message)
                    logger_.error(message)
                    continue
                logger_.info(f"Checking Camera '{cam}'")
                props = metrics_json[cam]

                # ========== Hard Checks that should never fail ==========

                # check at least 3 frames were processed for the other metrics to make sense.
                _check_geq(summary, cam, props, "processed", 3, 3, logger_)

                # gain can never be out of range (1 to 22)
                _check_leq(summary, cam, props, "gain_out_of_range", 0, 0, logger_)

                # exposure can never be out of range (1e-3ms to 22ms)
                _check_leq(summary, cam, props, "exposure_out_of_range", 0, 0, logger_)

                # hard fail on non-monotonically increasing timestamps
                _check_leq(summary, cam, props, "non_monotonic", 0, 0, logger_)

                # hard fail when reported exposure makes frames overlap
                _check_leq(
                    summary,
                    cam,
                    props,
                    "num_frames_with_unphysical_exposure_time",
                    0,
                    0,
                    logger_,
                )

                # check for invalid pixel values
                _check_leq(summary, cam, props, "roi_bad_frames", 0, 0, logger_)

                # temperature can never be out of range (-20C to 70C)
                # _check_leq(summary, cam, props, "temp_out_of_range", 0, 0)

                # misc. timing errors that should never happen
                _check_leq(summary, cam, props, "time_error", 0, 0, logger_)

                # we should never have "bad" frames, at least for Aria.
                _check_leq(summary, cam, props, "bad", 0, 0, logger_)

                # check that the sensor serial from the data stream exists in the calibration.
                _check_true(
                    summary, cam, props, "calibration_sensor_serials_match", logger_
                )

                # check that the overall calib in the VRS matches the one in the specific stream
                _check_true(
                    summary, cam, props, "factory_calibration_consistent", logger_
                )

                # check that the calibration was deemed "valid" during factory calibration per Calibration team's checks
                _check_true(summary, cam, props, "factory_calibration_valid", logger_)

                # ========== Soft Checks that do occasionally fail ==========

                # we don't accept frame drops over more than 210ms. Warn for more than 160ms.
                # Warn happens on ~0.7% of recordings
                # Error happens on ~0.3% of recordings
                _check_leq(
                    summary,
                    cam,
                    props,
                    "longest_frame_drop_us",
                    160000,
                    210000,
                    logger_,
                )

                # we don't accept deviation from set period by more than 500us. Warn on 250us.
                _check_leq(
                    summary,
                    cam,
                    props,
                    "largest_deviation_from_period_us",
                    250,
                    500,
                    logger_,
                )

                # we don't accept more than 0.2% frame drops overall. Warn at 0.1%
                # Warn happens on ~0.4% of recordings
                # Error happens almost never.
                _check_leq_ratio(
                    summary, cam, props, "dropped", "expected", 0.001, 0.002, logger_
                )

            # check IMUs
            for imu in imu_names:
                if imu not in metrics_json or type(metrics_json[imu]) is not dict:
                    message = "Required IMU stream {} not found or invalid".format(imu)
                    summary.error(message)
                    logger_.error(message)
                    continue
                logger_.info(f"Checking IMU '{imu}'")

                props = metrics_json[imu]

                # ========== Hard Checks that should never fail ==========

                # check at least 100 measurements were processed for the other metrics to make sense.
                _check_geq(summary, imu, props, "processed", 100, 100, logger_)

                # hard fail on non-monotonically increasing timestamps
                _check_leq(summary, imu, props, "non_monotonic", 0, 0, logger_)

                # misc. timing errors that should never happen
                _check_leq(summary, imu, props, "time_error", 0, 0, logger_)

                # ========== Soft Checks that do occasionally fail ==========

                # we don't accept IMU drops over more than 15ms. Warn for more than 10ms.
                _check_leq(
                    summary, imu, props, "longest_imu_skip_us", 10000, 15000, logger_
                )

                # we don't accept IMU drops of more than 0.2%. Warn on 0.1%
                _check_leq_ratio(
                    summary, imu, props, "dropped", "expected", 0.001, 0.002, logger_
                )

                # we don't accept deviation from set period by more than 100us.
                _check_leq(
                    summary,
                    imu,
                    props,
                    "largest_deviation_from_period_us",
                    100,
                    200,
                    logger_,
                )

                # ========== Data Integrity Heuristics ==========
                # The following are Heuristics to find data corruption.
                # They flag when the expected distribution is out of what's expected.
                # treat them with care!

                # max acceptable rotation acceleration is 25,000 rad/s2. Warn on 15,000.
                # Note that for Aria with Default settings, this cannot possibly occur for BMI263 as
                # it exceeds the saturation limits for 500dps. It can occur for BMI085.
                _check_leq(
                    summary,
                    imu,
                    props,
                    "max_observed_rotAccel_rad_per_s2",
                    15000,
                    25000,
                    logger_,
                )

                # count number of occurrences of rot accel exceeding 10,000 rad/s. This could happen
                # occasionally due to shock - but it should really not happen very often.
                _check_leq(
                    summary, imu, props, "non_physical_rotAccel", 250, 500, logger_
                )

                # The value here has been set to very loose, since repeated IMU samples has been found
                # as result of IMU output quantization, causing setting tight thresholds here resulting
                # false alarms
                _check_leq_ratio(
                    summary,
                    imu,
                    props,
                    "repeat_acceleration",
                    "total",
                    0.1,
                    0.5,
                    logger_,
                )
                _check_leq_ratio(
                    summary, imu, props, "repeat_gyroscope", "total", 0.1, 0.5, logger_
                )

                # check the longest continuously repeated IMU samples, which indicates the IMU output
                # is stuck
                _check_leq(
                    summary,
                    imu,
                    props,
                    "longest_continuous_repeat_acceleration",
                    10,
                    50,
                    logger_,
                )
                _check_leq(
                    summary,
                    imu,
                    props,
                    "longest_continuous_repeat_gyroscope",
                    10,
                    50,
                    logger_,
                )

            # check Pairs
            for pair in align_pairs:
                if pair not in metrics_json or type(metrics_json[pair]) is not dict:
                    message = "Required Camera-Pair {} not found or invalid".format(
                        pair
                    )
                    summary.error(message)
                    logger_.error(message)
                    continue
                logger_.info(f"Checking Camera Pair '{pair}'")
                props = metrics_json[pair]

                # we don't accept SLAM-SLAM misalignment of mid_exposure times by more than 400us. Warn on 200us.
                _check_leq(
                    summary, pair, props, "largest_misalignment_us", 200, 400, logger_
                )

                # we don't accept more than 0.5% framesets with misalignment of more than 200us. Warn on more than 0.1%.
                _check_leq_ratio(
                    summary,
                    pair,
                    props,
                    "num_frames_misaligned",
                    "num_frames_checked",
                    0.001,
                    0.005,
                    logger_,
                )

    except BaseException as e:
        traceback.print_exc()
        summary.error("Vrs Health check slam failed.")
        logger_.exception(e)
    finally:
        summary.to_file(summary_output)
    return summary


# ======================== Check Functions ========================
# probably there is some way of making this code more compact...
def _has_required_prop(
    summary: Summary,
    sensor_name: str,
    metrics_json: Dict[str, Union[float, int, bool]],
    prop_name: str,
    logger_: logging.Logger,
) -> bool:
    if prop_name not in metrics_json:
        message = "Could not find required property {}.{}".format(
            sensor_name, prop_name
        )
        summary.error(message)
        logger_.error(f"   {message}")
        return False
    return True


def _check_leq(
    summary: Summary,
    sensor_name: str,
    metrics_json: Dict[str, Union[float, int, bool]],
    prop_name: str,
    warn_threshold: float,
    error_threshold: float,
    logger_: logging.Logger,
):
    if not _has_required_prop(summary, sensor_name, metrics_json, prop_name, logger_):
        return

    message = "Check on {}.{}: Is {:,.2f} (Warn for >{:,}; Error for >{:,})".format(
        sensor_name, prop_name, metrics_json[prop_name], warn_threshold, error_threshold
    )

    if not metrics_json[prop_name] <= error_threshold:
        summary.error(message)
        logger_.error(f"   {message}")
    elif not metrics_json[prop_name] <= warn_threshold:
        summary.warn(message)
        logger_.warn(f"   {message}")
    else:
        logger_.info(f"   Pass: {message}")


def _check_geq(
    summary: Summary,
    sensor_name: str,
    metrics_json: Dict[str, Union[float, int, bool]],
    prop_name: str,
    warn_threshold: float,
    error_threshold: float,
    logger_: logging.Logger,
):
    if not _has_required_prop(summary, sensor_name, metrics_json, prop_name, logger_):
        return

    message = "Check on {}.{}: Is {:,.2f} (Warn for <{:,}; Error for <{:,})".format(
        sensor_name, prop_name, metrics_json[prop_name], warn_threshold, error_threshold
    )

    if not metrics_json[prop_name] >= error_threshold:
        summary.error(message)
        logger_.error(f"   {message}")
    elif not metrics_json[prop_name] >= warn_threshold:
        summary.warn(message)
        logger_.warn(f"   {message}")
    else:
        logger_.info(f"   Pass: {message}")


def _check_true(
    summary: Summary,
    sensor_name: str,
    metrics_json: Dict[str, Union[float, int, bool]],
    prop_name: str,
    logger_: logging.Logger,
):
    if not _has_required_prop(summary, sensor_name, metrics_json, prop_name, logger_):
        return

    message = "Check on {}.{}: Is {} (Error for False)".format(
        sensor_name, prop_name, metrics_json[prop_name]
    )

    if not metrics_json[prop_name]:
        summary.error(message)
        logger_.error(f"   {message}")
    else:
        logger_.info(f"   Pass: {message}")


def _check_leq_ratio(
    summary: Summary,
    sensor_name: str,
    metrics_json: Dict[str, Union[float, int, bool]],
    prop_name_num: str,
    prop_name_denom: str,
    warn_threshold: float,
    error_threshold: float,
    logger_: logging.Logger,
):
    if not _has_required_prop(
        summary, sensor_name, metrics_json, prop_name_num, logger_
    ):
        return
    if not _has_required_prop(
        summary, sensor_name, metrics_json, prop_name_denom, logger_
    ):
        return

    val = metrics_json[prop_name_num] / (metrics_json[prop_name_denom] or 1)
    message = "Check on {}.[{}/{}]: Is {}/{}={:,.2f}% (Warn for >{:,.2f}%; Error for >{:,.2f}%)".format(
        sensor_name,
        prop_name_num,
        prop_name_denom,
        metrics_json[prop_name_num],
        metrics_json[prop_name_denom],
        100 * val,
        100 * warn_threshold,
        100 * error_threshold,
    )

    if not val <= error_threshold:
        summary.error(message)
        logger_.error(f"   {message}")
    elif not val <= warn_threshold:
        summary.warn(message)
        logger_.warn(f"   {message}")
    else:
        logger_.info(f"   Pass: {message}")


def _vhc_run(path: Path, jsonOutFilename: Path):
    """
    Helper function to run the health check on a given vrs file. This needs to be in a
    separate function to keep ProcessPoolExecutor happy
    """
    logger.debug(f"Running health check on {path}")
    vhc.run(
        path=str(path),
        json_out_filename=str(jsonOutFilename),
        disable_logging=True,
    )
    logger.debug(f"Health check output written to {jsonOutFilename}")


async def run_health_check(vrs_file: Path, json_out: Path) -> None:
    """
    Run health check on a given vrs file.
    Args:
        vrs_file (Path): Path to the vrs file
    """
    if not hasattr(run_health_check, "semaphore_"):
        run_health_check.semaphore_: Semaphore = Semaphore(
            value=config.getint(
                ConfigSection.HEALTH_CHECK, ConfigKey.CONCURRENT_HEALTH_CHECKS
            )
        )

    async with run_health_check.semaphore_:
        await to_proc(_vhc_run, path=str(vrs_file), jsonOutFilename=str(json_out))


def is_eligible(feature: MpsFeature, rec: AriaRecording) -> bool:
    """
    Check if a feature is eligible for processing based on the results of the health check
    """
    if not rec.health_check_path.exists():
        raise FileNotFoundError(f"No health check found for {feature.file_path}")
    if feature in [MpsFeature.SLAM, MpsFeature.MULTI_SLAM]:
        summary: Summary = run_vrs_health_check_slam(
            rec.health_check_path, rec.health_check_slam_path
        )
        logger.debug(f"Summary: {summary}")
        return not bool(summary.errors)
    elif feature is MpsFeature.EYE_GAZE:
        ## We only check that the recording contains ET stream
        with open(rec.health_check_path) as vhc:
            return "Eye Camera Class #1" in json.load(vhc)
    elif feature is MpsFeature.HAND_TRACKING:
        ## We only check that the recording contains SLAM streams
        with open(rec.health_check_path) as vhc:
            vhc_json = json.load(vhc)
            return all(f"Camera Data (SLAM) #{i}" in vhc_json for i in (1, 2))
    raise NotImplementedError(f"Unknown feature type {feature}")
