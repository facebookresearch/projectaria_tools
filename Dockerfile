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

# Start from an ubuntu container
FROM ubuntu:jammy as base

# Ensure a SUDO command is available in the container
RUN if type sudo 2>/dev/null; then \
     echo "The sudo command already exists... Skipping."; \
    else \
     echo "#!/bin/sh\n\${@}" > /usr/sbin/sudo; \
     chmod +x /usr/sbin/sudo; \
    fi

# Update & upgrade distribution repositories
RUN apt-get update --fix-missing && DEBIAN_FRONTEND="noninteractive" TZ="America/New_York" apt-get install -y tzdata --fix-missing; sudo apt upgrade -y --fix-missing

# Install build essentials
RUN sudo apt-get install -y cmake git build-essential;

# Install python pip essentials
RUN sudo apt-get install -y libpython3-dev python3-pip;

# Install VRS dependencies and compile/install VRS

RUN sudo apt-get install -y \
    libboost-date-time-dev \
    libboost-filesystem-dev \
    libboost-iostreams-dev \
    libboost-system-dev \
    libfmt-dev \
    libgmock-dev libgtest-dev \
    liblz4-dev \
    libpng-dev libturbojpeg-dev \
    libxxhash-dev libzstd-dev;

# Install pangolin
RUN sudo apt-get install -y libeigen3-dev libglew-dev libgl1-mesa-dev -y; \
    cd /tmp; git clone https://github.com/stevenlovegrove/Pangolin.git \
    && mkdir Pangolin_Build && cd Pangolin_Build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_EXAMPLES=OFF ../Pangolin \
    && sudo make -j2 install;

# Code
ADD ./ /opt/projectaria_tools

# Configure
RUN mkdir /opt/projectaria_tools_Build; cd /opt/projectaria_tools_Build; \
    cmake -DBUILD_UNIT_TEST=ON \
      -DPROJECTARIA_TOOLS_BUILD_TOOLS=ON \
      -DPROJECTARIA_TOOLS_BUILD_PROJECTS=ON \
      -DPROJECTARIA_TOOLS_BUILD_PROJECTS_ADT=ON \
      -DPROJECTARIA_TOOLS_BUILD_PROJECTS_ASE=ON \
      /opt/projectaria_tools;

# Build
RUN cd /opt/projectaria_tools_Build; make -j2;

# Build python bindings
RUN cd /opt/projectaria_tools;\
    pip install --upgrade pip --user; \
    pip install patchelf; \
    sudo apt-get install -y libopenblas-dev; \
    CMAKE_BUILD_PARALLEL_LEVEL=4 pip3 install --global-option=build_ext .;

# See multistage docker file
# https://docs.docker.com/language/java/run-tests/#multi-stage-dockerfile-for-testing

FROM base as test

# Run C++ unit test
RUN cd /opt/projectaria_tools_Build; ctest -j;

# Run Python unit test
RUN cd /opt/projectaria_tools; \
    export TEST_FOLDER="/opt/projectaria_tools/data/"; \
    python3 -m unittest core/python/test/corePyBindTest.py; \
    python3 -m unittest core/python/test/mpsPyBindTest.py; \
    python3 -m unittest core/python/sophus/test/sophusPybindTest.py;
