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

import os
import re
import subprocess
import sys

from platform import system

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))


def _get_version():
    path = os.path.join(ROOT_DIR, "version.txt")
    version = open(path, "r").read().strip()
    return version


# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # required for auto-detection & inclusion of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",  # not used on MSVC, but no harm
            "-DBUILD_PYTHON_BINDINGS=ON",
            "-DBUILD_UNIT_TEST=OFF",
            "-DPROJECTARIA_TOOLS_BUILD_TOOLS=OFF",
            "-DPROJECTARIA_TOOLS_BUILD_PROJECTS=ON",
            "-DPROJECTARIA_TOOLS_BUILD_PROJECTS_ADT=ON",
            "-DPROJECTARIA_TOOLS_BUILD_PROJECTS_ASE=ON",
            "-DPROJECTARIA_TOOLS_BUILD_PROJECTS_AEA=ON",
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
        ]
        build_args = []
        # Adding CMake arguments set as environment variable
        # (needed e.g. to build for ARM OSx on conda-forge)
        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        if "PYTHONPATH" in os.environ and "pip-build-env" in os.environ["PYTHONPATH"]:
            # When the users use `pip install .` to install projectaria_tools and
            # the cmake executable is a python entry script, there will be
            # `Fix ModuleNotFoundError: No module named 'cmake'` from the cmake script.
            # This is caused by the additional PYTHONPATH environment variable added by pip,
            # which makes cmake python entry script not able to find correct python cmake packages.
            # Actually, sys.path is well enough for `pip install -e .`.
            # Therefore, we delete the PYTHONPATH variable.
            del os.environ["PYTHONPATH"]

        #
        # Setup platform specifics (compiler settings)
        #
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")
        if self.compiler.compiler_type != "msvc":
            if "Ninja" in cmake_generator:
                cmake_args += ["-GNinja"]
        else:
            build_args += ["--config=Release", "--parallel 4"]
            if "Visual Studio" in cmake_generator:
                cmake_args += [f"-G {cmake_generator}"]
            if os.path.exists("vcpkg"):
                # Note: VCPKG can be used to build static libraries & build a portable pypi compatible wheel
                # We are using vcpkg to compile dependencies
                # i.e configure the build to use CMAKE_TOOLCHAIN_FILE and VCPKG_TARGET_TRIPLET
                cmake_args += ["-DVCPKG_TARGET_TRIPLET=x64-windows-static"]
                vcpkg_toolchain_relative_filepath = (
                    "vcpkg\\scripts\\buildsystems\\vcpkg.cmake"
                )
                cmake_args += [
                    f"-DCMAKE_TOOLCHAIN_FILE={os.path.abspath(vcpkg_toolchain_relative_filepath)}"
                ]

        if sys.platform.startswith("darwin"):
            # Cross-compile support for macOS - respect ARCHFLAGS if set
            archs = re.findall(r"-arch (\S+)", os.environ.get("ARCHFLAGS", ""))
            if archs:
                cmake_args += ["-DCMAKE_OSX_ARCHITECTURES={}".format(";".join(archs))]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(
            ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp
        )
        subprocess.check_call(
            ["cmake", "--build", "."] + build_args, cwd=self.build_temp
        )

        # MSVC is putting pyd files in a subfolder
        # We are here setting them aside of the projectaria_tools build folder
        if system() == "Windows":
            from glob import glob

            file_list = glob(f"{self.build_lib}/**/*.pyd", recursive=True)
            for file in file_list:
                source = file
                destination = os.path.join(
                    os.path.dirname(os.path.dirname(file)), os.path.basename(file)
                )
                if os.path.exists(destination):
                    os.remove(destination)
                os.rename(source, destination)

        # If python-stubs exists, move it to the right folder for being packaged
        stub_folder = "projectaria_tools-stubs/projectaria_tools/"
        if os.path.exists(stub_folder):
            import shutil

            shutil.copytree(stub_folder, os.path.join(self.build_lib, stub_folder))


def main():
    # The information here can also be placed in setup.cfg - better separation of
    # logic and declaration, and simpler if you include description/version in a file.
    setup(
        name="projectaria-tools",
        version=_get_version(),
        description="Project Aria Tools",
        long_description="Python API for consuming Aria VRS files, calibration and MPS output and for downloading and consuming Aria open datasets",
        url="https://github.com/facebookresearch/projectaria_tools",
        ext_modules=[CMakeExtension("projectaria_tools", sourcedir=ROOT_DIR)],
        author="Meta Reality Labs Research",
        cmdclass={"build_ext": CMakeBuild},
        zip_safe=False,
        python_requires=">=3.8",
        install_requires=[
            "numpy",
            "requests",  # Required for datasets downloader
            "rerun-sdk>=0.20.0",
            "tqdm",
        ],
        extras_require={
            "all": [
                ## Required for tutorial/quickstart notebooks
                "jupyter",
                "matplotlib",
                "pandas",
                "pillow",
                "plotly",
                "scipy",
                ## Required for vrs_to_mp4
                "moviepy==1.0.3",
            ]
        },
        entry_points={
            "console_scripts": [
                "aria_dataset_downloader = projectaria_tools.tools.dataset_downloader.dataset_downloader_main:main",
                "dtc_object_downloader = projectaria_tools.projects.dtc_objects.downloader_main:main",
                "aria_mps = projectaria_tools.aria_mps_cli.cli:main",
                "aria_rerun_viewer = projectaria_tools.tools.aria_rerun_viewer.aria_rerun_viewer:main",
                "viewer_mps = projectaria_tools.tools.viewer_mps.viewer_mps:main",
                "viewer_projects_adt = projectaria_tools.utils.viewer_projects_adt:main",
                "viewer_projects_ase = projectaria_tools.utils.viewer_projects_ase:main",
                "viewer_projects_aea = projectaria_tools.utils.viewer_projects_aea:main",
                "vrs_to_mp4 = projectaria_tools.tools.vrs_to_mp4.vrs_to_mp4:main",
                "gen2_mp_csv_exporter = projectaria_tools.tools.gen2_mp_csv_exporter.run_gen2_mp_csv_exporter:main",
                # vrs_health_check will be added later for Gen2
                # "run_vrs_health_check = projectaria_tools.utils.run_vrs_health_check:main",
            ]
        },
        packages=find_packages(),
        license="Apache-2.0",
    )


if __name__ == "__main__":
    main()
