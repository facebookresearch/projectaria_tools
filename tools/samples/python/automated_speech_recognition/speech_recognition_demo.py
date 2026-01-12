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
import csv
import json
import os
import shutil
import tempfile
from typing import Optional

from faster_whisper import WhisperModel
from projectaria_tools.core import data_provider
from projectaria_tools.core.sensor_data import TimeDomain
from projectaria_tools.core.vrs import extract_audio_track

# Sample:
# Demonstrate how to run faster-whisper Aria audio stream
#
# 1. Extract audio stream
# 2. Run faster-whisper as
#    - segments
#    - words
# 3. Show how translate timestamp to Aria time domain
#
# Note:
# - If you using GPU inference you will have to install cudnn
# python -m pip install cudnn
# export LD_LIBRARY_PATH=`python -c 'import os; import nvidia.cublas.lib; import nvidia.cudnn.lib; print(os.path.dirname(nvidia.cublas.lib.__file__) + ":" + os.path.dirname(nvidia.cudnn.lib.__file__))'`


def extract_audio(vrs_file_path: str) -> Optional[str]:
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
        help="path to the VRS file to be considered for Automated Speech Recognition",
    )
    parser.add_argument(
        "--voice_activity_detector",
        action="store_true",
        help="Filter out parts of the audio without speech by using a Voice Activity Detector",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Extract the VRS audio stream as a wav file
    audio_path = None
    audio_path = extract_audio(args.vrs)
    if audio_path:
        print(f"audio extraction done: {audio_path}")
    else:
        print("Unable to extract audio from VRS file. Exiting...")
        exit(1)

    # Run audio transcription using faster-whisper
    #
    # Configure Speech recognition
    # - Choose the model you want to run and its inference settings (CPU/GPU)

    # model_size = "large-v3"
    model_size = "tiny.en"  # i.e. Force english language
    # Model can be picked along this list: {tiny, base, small, medium, large}
    # Here a list you can choose from:
    # - Run on GPU with FP16
    # model = WhisperModel(model_size, device="cuda", compute_type="float16")
    # - or run on GPU with INT8
    # model = WhisperModel(model_size, device="cuda", compute_type="int8_float16")
    # - or run on CPU with INT8
    model = WhisperModel(model_size, device="cpu", compute_type="int8")

    # The API can be used to extract both segments (1) and words (2),
    # we are showing here how to use both,
    # and (3) showing how to convert time information into Aria time domain.

    #
    # (1) Using API to extract segments
    #
    segments, info = model.transcribe(
        audio_path, beam_size=5, vad_filter=args.voice_activity_detector
    )

    print(
        "Detected language '%s' with probability %f"
        % (info.language, info.language_probability)
    )
    print(f"Voice Activity Detection: {args.voice_activity_detector}")

    print("Detected text segments:")
    for segment in segments:
        print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))

    #
    # (2) Using API to extract timestamps per word
    #
    print("\n\n\n")
    segments, _ = model.transcribe(
        audio_path, word_timestamps=True, vad_filter=args.voice_activity_detector
    )
    print("Detected text WORD segments:")
    for segment in segments:
        for word in segment.words:
            print(f"[{round(word.start, 2)}s, -> {round(word.end, 2)}s] {word.word}")

    #
    # (3) Move to Aria time domain by adding audio starting timestamp to each word timestamp
    #
    try:
        provider = data_provider.create_vrs_data_provider(args.vrs)
    except Exception as e:
        print(
            f"Can't create a vrs data provider, we will not time align the speech timestamps to the Aria VRS timestamps. Exception: {e}"
        )
    else:
        audio_stream_id = provider.get_stream_id_from_label("mic")
        audio_starting_timestamp = provider.get_first_time_ns(
            audio_stream_id, TimeDomain.DEVICE_TIME
        )
        segments, _ = model.transcribe(
            audio_path, word_timestamps=True, vad_filter=args.voice_activity_detector
        )
        # save data to an array to log them to a CSV file
        data = [["startTime_ns", "endTime_ns", "written", "confidence"]]
        print("Detected text segments (time aligned to Aria time domain):")
        print(f"VRS audio stream starting timestamp(ns): {audio_starting_timestamp}")
        for segment in segments:
            for word in segment.words:
                # move to aria TimeDomain
                s_to_ns = int(1e9)
                begin = int(word.start * s_to_ns + audio_starting_timestamp)
                end = int(word.end * s_to_ns + audio_starting_timestamp)
                print(f"[{begin}ns, -> {end}ns] {word.word}")
                data.append([begin, end, word.word, word.probability])

        # Show how to export this data to CSV
        filename = "speech.csv"
        with open(filename, mode="w") as file:
            writer = csv.writer(file)
            writer.writerows(data)

    #
    # Cleanup
    # If audio file has been extracted, remove the containing temporary folder that we created
    if audio_path:
        shutil.rmtree(os.path.dirname(audio_path))


if __name__ == "__main__":
    main()
