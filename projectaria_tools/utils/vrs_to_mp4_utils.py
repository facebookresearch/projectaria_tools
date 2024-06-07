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

import csv
import json
import os
import shutil
import subprocess
import tempfile

import ffmpeg

import numpy as np
from moviepy.audio.AudioClip import AudioClip
from moviepy.editor import AudioFileClip
from moviepy.video.io.VideoFileClip import VideoClip
from PIL import Image

from projectaria_tools.core import data_provider
from projectaria_tools.core.sensor_data import TimeDomain, TimeQueryOptions
from projectaria_tools.core.stream_id import StreamId


def max_signed_value_for_bytes(n):
    return (1 << (8 * n - 1)) - 1


# Get the vrs_device_time_ns array from mp4 'description' tag using ffprobe command
def get_timestamp_from_mp4(file_path) -> np.ndarray:
    ffprobe_binary = "ffprobe"
    try:
        result = subprocess.run(
            [
                ffprobe_binary,
                "-v",
                "quiet",
                "-print_format",
                "json",
                "-show_format",
                "-show_entries",
                "format_tags",
                file_path,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        metadata = json.loads(result.stdout)
        description = metadata.get("format", {}).get("tags", {})["description"]
        description = description.replace("[", "")
        description = description.replace("]", "")

        vrs_device_timestamps_ns = list(map(int, description.split(",")))
        return np.array(vrs_device_timestamps_ns)
    except subprocess.CalledProcessError:
        raise ValueError(
            f"Binary {ffprobe_binary} does not exist. Please install {ffprobe_binary} before running the code"
        )


# Save the timestamp_array into 'description' tag and generate new output_video_file
def save_timestamp_to_mp4(input_video_file, output_video_file, timestamp_array):
    mp4_metadata = {"description": timestamp_array}
    ffmpeg_input = ffmpeg.input(input_video_file)
    ffmpeg_output = ffmpeg.output(
        ffmpeg_input,
        output_video_file,
        **{"metadata": " ".join([f"{k}={v}" for k, v in mp4_metadata.items()])},
        codec="copy",
    )
    ffmpeg_output.run(overwrite_output=True)


def convert_vrs_to_mp4(
    vrs_file: str, output_video: str, log_folder=None, down_sample_factor=1
):
    """Convert a VRS file to MP4 video."""
    use_temp_folder = False
    if log_folder is None:
        # If no log folder is provided, we will create a temporary one
        use_temp_folder = True
        log_folder = tempfile.mkdtemp()
    elif os.path.exists(log_folder) is False:
        os.mkdir(log_folder)

    # Create a vrs to mp4 converter
    converter = Vrs2Mp4Converter(vrs_file, down_sample_factor)
    duration_ns = converter.duration_ns()
    duration_in_second = duration_ns * 1e-9
    video_writer_clip = VideoClip(converter.make_frame, duration=duration_in_second)

    temp_video_file = os.path.join(log_folder, "video.mp4")
    temp_audio_file = os.path.join(log_folder, "audio.mp3")
    # extract audio from vrs file
    if converter.contain_audio():

        audio_writer_clip = AudioClip(
            converter.make_audio_data,
            duration=duration_in_second,
            fps=converter.audio_config.sample_rate,
        )
        audio_writer_clip.nchannels = converter.audio_config.num_channels
        audio_writer_clip.write_audiofile(
            temp_audio_file,
            fps=converter.audio_config.sample_rate,
            buffersize=converter.audio_buffersize(),
        )
        audio_clip = AudioFileClip(
            temp_audio_file,
        )
        video_writer_clip = video_writer_clip.set_audio(audio_clip)
        audio_writer_clip.close()

    video_writer_clip.write_videofile(temp_video_file, fps=converter.video_fps())
    video_writer_clip.close()

    vrs_device_time_ns_array = converter.write_mp4_to_vrs_time_ns(log_folder)
    converter.write_log(log_folder)

    # add vrs_device_time_ns_array to file tag
    os.makedirs(os.path.dirname(output_video), exist_ok=True)
    save_timestamp_to_mp4(temp_video_file, output_video, vrs_device_time_ns_array)

    # check if saved timestamp is the same
    metadata_timestamps = get_timestamp_from_mp4(output_video)
    correct_metadata = True
    for _, (metadata_time, vrs_time) in enumerate(
        zip(metadata_timestamps, vrs_device_time_ns_array)
    ):
        if metadata_time != vrs_time:
            correct_metadata = False

    if correct_metadata is False:
        print("Error: Timestamp saved to mp4 is not the same as VRS timestamp")

    # remove log folder if it is a temporary folder
    if use_temp_folder:
        shutil.rmtree(log_folder)
    else:
        # remove the temp audio and video file if log_folder is provided
        if os.path.exists(temp_video_file):
            os.remove(temp_video_file)
        if os.path.exists(temp_audio_file):
            os.remove(temp_audio_file)


class Vrs2Mp4Converter:
    """
    A class that is used to convert VRS RGB frames to a moviepy Video Clip and save to MP4.
    make_frame(t)->np.ndarray is called to insert each RGB frame into MP4 for timestamp t in MP4 domain
    make_audio_data(t)->np.ndarray is called to insert audio stream into MP4
    """

    def __init__(self, vrs_path: str, down_sampling_factor: int = 1):
        self.down_sampling_factor_ = down_sampling_factor

        self.provider_ = data_provider.create_vrs_data_provider(vrs_path)
        if not self.provider_:
            raise ValueError(f"vrs file: '{vrs_path}' cannot be read")

        # Extract RGB frame with streamId "214-1" and audio stream "231-1"
        self.image_streamid_ = StreamId("214-1")
        self.audio_streamid_ = StreamId("231-1")

        self.contain_rgb_ = self.provider_.check_stream_is_active(self.image_streamid_)
        self.contain_audio_ = self.provider_.check_stream_is_active(
            self.audio_streamid_
        )

        if self.contain_rgb_ is False:
            raise SystemExit("The vrs does not contain RGB frames to convert to mp4")

        self.start_timestamp_ns_ = self.provider_.get_first_time_ns(
            self.image_streamid_, TimeDomain.DEVICE_TIME
        )
        self.end_timestamp_ns_ = self.provider_.get_last_time_ns(
            self.image_streamid_, TimeDomain.DEVICE_TIME
        )
        if self.contain_audio_ is True:
            self.audio_config = self.provider_.get_audio_configuration(
                self.audio_streamid_
            )
            self.audio_max_value_ = max_signed_value_for_bytes(4)

            # RECORD_TIME for audio is the START of a Record
            # DEVICE_TIME for audio is the END of a Record
            # start_audio_timestamp use RECORD_TIME while end_audio_timestamp use DEVICE_TIME
            # The time period between both times is the duration of the MP4
            self.start_timestamp_ns_ = self.provider_.get_first_time_ns(
                self.audio_streamid_, TimeDomain.RECORD_TIME
            )
            self.end_timestamp_ns_ = self.provider_.get_last_time_ns(
                self.audio_streamid_, TimeDomain.DEVICE_TIME
            )

        # initialize the first image data
        image_data_and_record = self.provider_.get_image_data_by_time_ns(
            self.image_streamid_,
            self.start_timestamp_ns_,
            TimeDomain.DEVICE_TIME,
            TimeQueryOptions.CLOSEST,
        )
        self.image_prev_index_ = self.provider_.get_index_by_time_ns(
            self.image_streamid_,
            self.start_timestamp_ns_,
            TimeDomain.DEVICE_TIME,
            TimeQueryOptions.CLOSEST,
        )
        img_array = self.convert_image(image_data_and_record)
        self.last_valid_image_ = img_array
        self.last_time_ns_ = image_data_and_record[1].capture_timestamp_ns

        # Initialize logging data
        self.mp4_to_vrs_timestamp_ns_ = {}
        self.invalid_frames_ = np.array([])
        self.skipped_frames_ = np.array([])
        self.duplicated_frames_ = np.array([])

    # write mp4 timestamp to vrs device timestamp in csv file and return array of vrs_device_time_ns
    def write_mp4_to_vrs_time_ns(self, log_path: str) -> np.ndarray:
        output_file = os.path.join(log_path, "mp4_to_vrs_time_ns.csv")
        vrs_device_time_ns_array = []
        with open(output_file, "w") as csv_file:
            writer = csv.writer(csv_file)
            title_string = [
                "mp4_time_ns",
                "relative_vrs_device_time_ns",
                "vrs_device_time_ns",
            ]
            writer.writerow(title_string)
            for key, value in self.mp4_to_vrs_timestamp_ns_.items():
                writer.writerow([key, value - self.start_timestamp_ns_, value])
                # add vrs_device_time_ns to array
                vrs_device_time_ns_array.append(value)
        return vrs_device_time_ns_array

    def contain_audio(self) -> bool:
        return self.contain_audio_

    # write log file
    def write_log(self, log_path: str):
        output_file = os.path.join(log_path, "vrs_to_mp4_log.json")
        log_data = {}
        log_data["num_mp4_frames"] = len(self.mp4_to_vrs_timestamp_ns_)
        log_data["down_sampling_factor_"] = self.down_sampling_factor_
        log_data["num_skipped_frames"] = len(self.skipped_frames_)
        log_data["num_duplicated_frames_"] = len(self.duplicated_frames_)
        log_data["num_invalid_frames_"] = len(self.invalid_frames_)
        log_data["first_video_timestamp_ns"] = self.start_timestamp_ns_
        log_data["end_video_timestamp_ns"] = self.end_timestamp_ns_
        log_data["video_duration_ns"] = self.duration_ns()

        for key, value in log_data.items():
            print(f"{key} : {value}")

        with open(output_file, "w") as outfile:
            json.dump(log_data, outfile)

    def convert_image(self, image_data_and_record):
        img_array = image_data_and_record[0].to_numpy_array().copy()
        if self.down_sampling_factor_ > 1:
            img_array = Image.fromarray(img_array)
            img_array = img_array.resize(
                [
                    int(img_array.width / self.down_sampling_factor_),
                    int(img_array.height / self.down_sampling_factor_),
                ]
            )
            img_array = np.array(img_array)
        img_array = np.rot90(img_array, -1)
        return img_array

    def duration_ns(self):
        return self.end_timestamp_ns_ - self.start_timestamp_ns_

    def video_fps(self):
        return self.provider_.get_nominal_rate_hz(self.image_streamid_)

    # add each frame to the video clip based on the closest image frame to the vrs_timestamp_ns
    def make_frame(self, t) -> np.ndarray:
        video_timestamp_ns = t * 1e9
        vrs_timestamp_ns = int(self.start_timestamp_ns_ + video_timestamp_ns)
        image_data_and_record = self.provider_.get_image_data_by_time_ns(
            self.image_streamid_,
            vrs_timestamp_ns,
            TimeDomain.DEVICE_TIME,
            TimeQueryOptions.CLOSEST,
        )

        image_index = self.provider_.get_index_by_time_ns(
            self.image_streamid_,
            vrs_timestamp_ns,
            TimeDomain.DEVICE_TIME,
            TimeQueryOptions.CLOSEST,
        )
        if image_index > self.image_prev_index_ + 1:
            self.skipped_frames_ = np.append(
                self.skipped_frames_, self.image_prev_index_ + 1
            )
        elif image_index == self.image_prev_index_:
            self.duplicated_frames_ = np.append(
                self.duplicated_frames_, self.image_prev_index_
            )

        if image_data_and_record[0].is_valid() is False:
            self.invalid_frames_ = np.append(self.invalid_frames_, image_index)
            self.duplicated_frames_ = np.append(
                self.duplicated_frames_, self.image_prev_index_
            )
            self.mp4_to_vrs_timestamp_ns_[video_timestamp_ns] = self.last_time_ns_
            return self.last_valid_image_

        image_device_time_ns = image_data_and_record[1].capture_timestamp_ns
        self.mp4_to_vrs_timestamp_ns_[video_timestamp_ns] = image_device_time_ns

        self.last_valid_image_ = self.convert_image(image_data_and_record)
        self.image_prev_index_ = image_index
        self.last_time_ns_ = image_device_time_ns

        return self.last_valid_image_

    # Number of samples of each records (e.g. 2048 or 4096)
    def audio_buffersize(self):
        audio_data = self.provider_.get_audio_data_by_index(self.audio_streamid_, 1)
        return len(audio_data[1].capture_timestamps_ns)

    # add each audio record to the MP4
    def make_audio_data(self, t) -> np.ndarray:
        if self.contain_audio_ is False:
            raise SystemExit(
                "The vrs does not contain audio and cannot be used to extract audio data."
            )

        # length of input variable len(t) == audio_buffersize - 1
        if np.size(t) == 1:
            return 0
        query_timestamp_ns = t * 1e9
        vrs_timestamp_ns = int(self.start_timestamp_ns_ + query_timestamp_ns[0])

        # obtaining the closest audio record to the vrs_timestamp_ns
        audio_data_and_config = self.provider_.get_audio_data_by_time_ns(
            self.audio_streamid_,
            vrs_timestamp_ns,
            TimeDomain.RECORD_TIME,
            TimeQueryOptions.CLOSEST,
        )
        audio_data = np.array(audio_data_and_config[0].data)
        audio_data = audio_data.astype(np.float64) / self.audio_max_value_

        # return all channels
        # a subset of channels create crackle sounds
        return audio_data
