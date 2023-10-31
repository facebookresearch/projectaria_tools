#!/usr/bin/env python3
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
import shutil
import tempfile

import numpy as np
from moviepy.editor import AudioFileClip
from moviepy.video.io.VideoFileClip import VideoClip

from projectaria_tools.core import data_provider
from projectaria_tools.core.sensor_data import TimeDomain
from projectaria_tools.core.stream_id import StreamId
from projectaria_tools.core.vrs import extract_audio_track


def extract_audio(vrs_file_path: str) -> str:
    """Extract audio from a VRS file as a wav file in a temporary folder."""
    temp_folder = tempfile.mkdtemp()
    if not temp_folder:
        return None
    # else continue process vrs audio extraction
    json_output_string = extract_audio_track(
        vrs_file_path, os.path.join(temp_folder, "audio.wav")
    )
    json_output = json.loads(json_output_string)  # Convert string to Dict
    if json_output and json_output["status"] == "success":
        return json_output["output"]
    # Else we were not able to export a Wav file from the VRS file
    return None


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--vrs",
        type=str,
        required=True,
        help="path to the VRS file to be converted to a video",
    )
    parser.add_argument(
        "--output_video",
        type=str,
        required=True,
        help="path to the VIDEO file you want to create",
    )
    parser.add_argument(
        "--downsample",
        type=int,
        required=False,
        default=2,
        help="Downsampling factor on VRS images (Must be >=1)",
    )
    return parser.parse_args()


class Vrs2MoviePyFrameConverter:
    """
    Class used to convert the VRS RGB frame to a moviepy Video Clip.
    A Vrs2MoviePyFrameConverter object is defined as callable to be used as a make_frame(t) functor by moviepy.
    """

    def sample_frame_and_timestamp(self, image_data_and_record) -> [np.ndarray, int]:
        """
        Return the image frame and corresponding timestamp.
            Image is down sampled and rotated if required.
        """
        img = image_data_and_record.image_data_and_record()[0].to_numpy_array().copy()

        if self.down_sampling_factor_ > 1:
            img = img[:: self.down_sampling_factor_, :: self.down_sampling_factor_]
        # Rotate image
        img = np.rot90(img, -1)

        capture_timestamp = image_data_and_record.image_data_and_record()[
            1
        ].capture_timestamp_ns
        return [img, capture_timestamp]

    def __init__(self, vrs_path: str, down_sampling_factor: int = 1):
        self.down_sampling_factor_ = down_sampling_factor

        ##
        # Initialize the VRS data provider
        self.provider_ = data_provider.create_vrs_data_provider(vrs_path)
        if not self.provider_:
            raise ValueError(f"vrs file: '{vrs_path}' cannot be read")

        self.rgb_stream_id_ = StreamId("214-1")

        ##
        # Configure a deliver queue to provide only RGB image data stream

        deliver_option = self.provider_.get_default_deliver_queued_options()
        deliver_option.deactivate_stream_all()
        deliver_option.activate_stream(self.rgb_stream_id_)

        self.seq_ = self.provider_.deliver_queued_sensor_data(deliver_option)
        self.iter_data_ = iter(self.seq_)
        image_data_and_record = next(self.iter_data_)
        self.last_valid_frame_, self.last_timestamp_ = self.sample_frame_and_timestamp(
            image_data_and_record
        )
        self.first_timestamp_ = self.last_timestamp_
        self.dropped_frames_count_ = 0

    def stream_fps(self) -> int:
        """Collect stream characteristic, Get the time period between two image sample."""
        return int(self.provider_.get_nominalRateHz(self.rgb_stream_id_))

    def stream_duration(self) -> int:
        """Return the RGB stream duration in seconds."""
        t_first = self.provider_.get_first_time_ns(
            self.rgb_stream_id_, TimeDomain.DEVICE_TIME
        )
        t_last = self.provider_.get_last_time_ns(
            self.rgb_stream_id_, TimeDomain.DEVICE_TIME
        )
        import math

        # Keeping only integer seconds (no decimals)
        duration_in_seconds = math.floor((t_last - t_first) / 1e9)
        return duration_in_seconds

    def dropped_frames_count(self) -> int:
        """
        Return the number of counted frame drop (frame that did not match the expect timestamp)
        """
        return self.dropped_frames_count_

    def __call__(self, t) -> np.ndarray:
        """
        Create a functor compatible with the make_frame(t) functor concept of moviePy VideoClip.
        This function return VRS frame in time alignment with the time {t} request of moviePy.
        - If a frame is not present as a given expected time, we count the frame as missing/dropped and return the last valid frame.
        """
        try:
            moviepy_timestamp_us = t * 1e9
            vrs_timestamp = self.last_timestamp_ - self.first_timestamp_

            # If time match we return the last frame
            if vrs_timestamp > moviepy_timestamp_us:
                # VRS is in advance, so we return the last frame we collected
                self.dropped_frames_count_ += 1
                return self.last_valid_frame_
            else:
                obj = next(self.iter_data_)
                # We get a new image from the queue
                (
                    self.last_valid_frame_,
                    self.last_timestamp_,
                ) = self.sample_frame_and_timestamp(obj)
            return self.last_valid_frame_

        except StopIteration:
            print("We have exhausted the VRS stream, keep sending last VRS frame")
            return self.last_valid_frame_


def main():
    args = parse_args()

    if args.downsample < 1:
        raise ValueError(
            f"Invalid downsample value: {args.downsample}. Must be greater than or equal to 1"
        )

    # Test to see if we can extract the VRS audio stream as a wav file
    audio_path = None
    audio_path = extract_audio(args.vrs)

    ##
    # Prepare the Vrs2MoviePyFrameConverter and configure a moviePy video clip
    frame_converter = Vrs2MoviePyFrameConverter(args.vrs, args.downsample)
    duration_in_seconds = frame_converter.stream_duration()
    # Create a VideoClip of the desired duration, and using the Vrs Source
    video_writer_clip = VideoClip(frame_converter, duration=duration_in_seconds)
    # Add the audio as audio clip if any
    if audio_path:
        audio_clip = AudioFileClip(audio_path)
        video_writer_clip = video_writer_clip.set_audio(audio_clip)

    # Configure the VideoWriter and run the process
    output_video_fps = frame_converter.stream_fps()
    video_writer_clip.write_videofile(args.output_video, fps=output_video_fps)
    video_writer_clip.close()

    # If audio file has been extracted, remove the containing temporary folder that we created
    if audio_path:
        shutil.rmtree(os.path.dirname(audio_path))

    print("VRS to MP4 summary:")
    print(f"Created a {duration_in_seconds} seconds video.")
    print(f"- FPS: {output_video_fps}")
    print(f"- Drop frame(s) count: {frame_converter.dropped_frames_count()}")


if __name__ == "__main__":
    main()
