#!/usr/bin/env bash
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

# Parts of this software are licensed under the <a href=http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>LGPLv2.1</a>
# and its source can be downloaded <a href=https://github.com/FFmpeg/FFmpeg/archive/refs/tags/n7.1.tar.gz>here</a>

set -e

if [[ $(uname -s) == "Darwin" ]]; then
  VRS_PLATFORM="macos"
elif [[ $(uname -s) == "Linux" ]]; then
  VRS_PLATFORM="linux"
elif [[ $(uname -s) == *"MINGW"* ]]; then
  VRS_PLATFORM="windows"
else
  echo "ERROR: Unsupported operating system: $(uname -s)" >&2
  exit 1
fi

# Exit if Windows
echo "Detected platform: $VRS_PLATFORM"
if [[ "$VRS_PLATFORM" == "windows" ]]; then
    echo "This script is intended for Unix-like systems (Linux/macOS)."
    exit 1
fi


################################################################################
# Build FFmpeg
################################################################################

# Define the FFmpeg version to download
FFMPEG_VERSION="n7.1"
# Define the directory where FFmpeg will be downloaded and built
BUILD_DIR="$HOME/ffmpeg_build"
# Define the custom installation path
INSTALL_PATH="$HOME/vrs_third_party_libs/ffmpeg"
# Check if FFmpeg is already installed
if [ -x "$INSTALL_PATH/bin/ffmpeg" ]; then
    echo "FFmpeg is already installed at $INSTALL_PATH"
    "$INSTALL_PATH/bin/ffmpeg" -version
    exit 0
fi


# Create the build directory if it doesn't exist
mkdir -p "$BUILD_DIR"
mkdir -p "$INSTALL_PATH"
# Change to the build directory
cd "$BUILD_DIR"
# Download the FFmpeg source code from GitHub
echo "Downloading FFmpeg version $FFMPEG_VERSION..."
wget "https://github.com/FFmpeg/FFmpeg/archive/refs/tags/$FFMPEG_VERSION.tar.gz"

# Extract the downloaded tarball
echo "Extracting FFmpeg..."
tar -xzf "$FFMPEG_VERSION.tar.gz"

# Change to the FFmpeg source directory, configure, compile, and install
cd "FFmpeg-$FFMPEG_VERSION"
echo "Configuring FFmpeg..."
./configure --prefix="$INSTALL_PATH" \
  --disable-swresample --disable-swscale --disable-everything \
  --enable-decoder=hevc \
  --disable-asm --disable-x86asm \
  $( [ "$VRS_PLATFORM" = "macos" ] && printf '%s ' \
       --enable-decoder=hevc_videotoolbox \
       --enable-hwaccel=hevc_videotoolbox \
       --enable-videotoolbox ) \
  --enable-shared --disable-static
echo "Compiling FFmpeg..."
if ! make -j"$(nproc)"
then
    echo "FFmpeg compilation failed."
    exit 1
fi

echo "Installing FFmpeg..."
if ! make install
then
    echo "FFmpeg installation failed."
    exit 1
fi

# Clean up
echo "Cleaning up..."
cd "$HOME"
rm -rf "$BUILD_DIR"
echo "FFmpeg installed successfully to $INSTALL_PATH"
