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

set -euo pipefail
echo "=== STARTING manylinux deps install ==="

# Update the mirror list for Centos mirror deprecation
sed -i 's/mirrorlist/#mirrorlist/g' /etc/yum.repos.d/CentOS-*
sed -i 's|#baseurl=http://mirror.centos.org|baseurl=http://vault.centos.org|g' /etc/yum.repos.d/CentOS-*

# Install system deps
# Toolchains
yum install -y cmake git gtest-devel \
    lz4-devel libzstd-devel xxhash-devel libpng-devel libjpeg-turbo-devel wget

# Install and compile libraries
thread=$(nproc)

# Build Fmt
cd /tmp; git clone https://github.com/fmtlib/fmt.git -b 10.2.1 \
    && cd fmt \
    && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE -DFMT_TEST=OFF .; make -j$thread install; rm -rf /tmp/fmt;

# Build lib jpeg turbo
cd /tmp && git clone https://github.com/libjpeg-turbo/libjpeg-turbo.git -b 3.0.1 \
    && cd libjpeg-turbo \
    && cmake -DCMAKE_BUILD_TYPE=Release -DWITH_JPEG8=1 -DCMAKE_INSTALL_DEFAULT_PREFIX=/usr .;make -j$thread install; rm -rf /tmp/libjpeg-turbo;

# Build Boost
cd /tmp && git clone --recursive https://github.com/boostorg/boost.git -b boost-1.84.0 \
  && cd boost \
  && ./bootstrap.sh \
  && ./b2 --with-system --with-filesystem --with-thread --with-chrono --with-date_time --with-serialization --with-iostreams install \
  && rm -rf /tmp/boost;

# Build Opus
cd /tmp && curl -kOL http://downloads.xiph.org/releases/opus/opus-1.5.2.tar.gz \
  && tar -zxf opus-1.5.2.tar.gz \
  && cd opus-1.5.2 \
  && ./configure \
  && make -j4 \
  && make install \
  && rm -rf /tmp/opus-1.5.2.tar.gz /tmp/opus-1.5.2 \

# Build FFmpeg
cd /project \
  && ./build_third_party_libs/build_ffmpeg_linuxunix.sh
