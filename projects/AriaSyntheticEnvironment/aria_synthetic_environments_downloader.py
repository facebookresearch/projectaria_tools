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
import hashlib
import json
import os
import ssl
import urllib.request
from zipfile import ZipFile

from tqdm import tqdm

ssl._create_default_https_context = ssl._create_unverified_context

SCENES_PER_CHUNK = 10

# Copied from tqdm website/documentation - probably worth moving into a utils.py
def urllib_tqdm_hook(t):
    last_b = [0]

    def inner(b=1, bsize=1, tsize=None):
        if tsize is not None:
            t.total = tsize
        t.update((b - last_b[0]) * bsize)
        last_b[0] = b

    return inner


# Handler for list of scene ids. Handles lists of integers and ranges.
def ASEIdsParser(string):
    try:
        ids = string.split(",")
        ids = [
            list(range(int(x.split("-")[0]), int(x.split("-")[1]) + 1))
            if "-" in x
            else [int(x)]
            for x in ids
        ]
        ids = [item for sublist in ids for item in sublist]
        return list(set(ids))
    except Exception as e:
        print("Error: ", e)
        raise argparse.ArgumentTypeError(
            "Scene ids must be comma separated integers or ranges. For example: 1,2,3-5,6"
        )


parser = argparse.ArgumentParser(
    prog="Aria Synthetic Environments Downloader",
    description="Downloads Aria Synthetic Environments as defined in CDN json file.",
)
parser.add_argument(
    "--set",
    help="The type of scenes to download. Options are train (test set will be added soon).",
    choices=["train"],
    required=True,
)
parser.add_argument(
    "--scene-ids",
    help="Scene ids to download. ",
    required=True,
    type=ASEIdsParser,
)
parser.add_argument(
    "--cdn-file",
    help="Input file listing the CDN urls, downloaded from ASE website.",
    required=True,
)

parser.add_argument("--output-dir", help="Output directory", required=True)

parser.add_argument("--unzip", help="Unzip the downloaded zip files. ", required=True)


def load_meta_data(cdn_file: str):
    # Load the metadata file downloaded from the ASE website.
    with open(cdn_file) as fp:
        metadata = json.load(fp)
    return metadata


def main(
    cdn_file: str, output_dir: str, scene_ids: list, set_type: str, unzip_flag: bool
):
    # Load the metadata file downloaded from the ASE website.
    metadata = load_meta_data(cdn_file)

    # Create the output directory.
    if not os.path.exists(output_dir):
        print(f"Creating local output folder {output_dir}")
        os.makedirs(output_dir)

    chunk_ids_to_download = list(set([x // SCENES_PER_CHUNK for x in scene_ids]))
    chunk_ids_to_download.sort()
    for i, chunk_id in enumerate(chunk_ids_to_download):
        chunk_filename = "{}_chunk_{}.zip".format(set_type, f"{chunk_id:07}")
        print(
            "Downloading chunk {}/{}: {}".format(
                i + 1, len(chunk_ids_to_download), chunk_filename
            )
        )
        chunk_details = next(
            (item for item in metadata if item["filename"] == chunk_filename), None
        )
        download_url = chunk_details["cdn"]
        print(download_url)
        download_sha = chunk_details["sha"]
        download_local_filename = os.path.join(output_dir, chunk_filename)
        with tqdm(
            unit="B", unit_scale=True, leave=True, miniters=1, desc="Progress"
        ) as t:  # all optional kwargs
            urllib.request.urlretrieve(
                download_url,
                download_local_filename,
                reporthook=urllib_tqdm_hook(t),
                data=None,
            )
            with open(str(download_local_filename), "rb") as f:
                local_sha = hashlib.sha1(f.read()).hexdigest()
            assert (
                local_sha == download_sha
            ), "Downloaded file does not match sha1 hash in metadata file."

            if unzip_flag:
                with ZipFile(download_local_filename, "r") as zip_ref:
                    zip_ref.extractall(output_dir)
                os.remove(download_local_filename)


if __name__ == "__main__":

    args = parser.parse_args()
    main(
        cdn_file=args.cdn_file,
        output_dir=args.output_dir,
        scene_ids=args.scene_ids,
        set_type=args.set,
        unzip_flag=args.unzip,
    )
