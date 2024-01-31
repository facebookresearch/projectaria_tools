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

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

# Convert distutils Windows platform specifiers to CMake -A arguments
PLAT_TO_CMAKE = {
    "win32": "Win32",
    "win-amd64": "x64",
    "win-arm32": "ARM",
    "win-arm64": "ARM64",
}


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

        # CMake lets you override the generator - we need to check this.
        # Can be set with Conda-Build, for example.
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

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

        if self.compiler.compiler_type != "msvc":
            # Using Ninja-build since it a) is available as a wheel and b)
            # multithreaded automatically. MSVC would require all variables be
            # exported for Ninja to pick it up, which is a little tricky to do.
            # Users can override the generator with CMAKE_GENERATOR in CMake
            # 3.15+.
            if not cmake_generator:
                try:
                    import ninja  # noqa: F401

                    cmake_args += ["-GNinja"]
                except ImportError:
                    pass

        else:

            # Single config generators are handled "normally"
            single_config = any(x in cmake_generator for x in {"NMake", "Ninja"})

            # CMake allows an arch-in-generator style for backward compatibility
            contains_arch = any(x in cmake_generator for x in {"ARM", "Win64"})

            # Specify the arch if using MSVC generator, but only if it doesn't
            # contain a backward-compatibility arch spec already in the
            # generator name.
            if not single_config and not contains_arch:
                cmake_args += ["-A", PLAT_TO_CMAKE[self.plat_name]]

            # Multi-config generators have a different way to specify configs
            if not single_config:
                cmake_args += [
                    f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}"
                ]
                build_args += ["--config", cfg]

        if sys.platform.startswith("darwin"):
            # Cross-compile support for macOS - respect ARCHFLAGS if set
            archs = re.findall(r"-arch (\S+)", os.environ.get("ARCHFLAGS", ""))
            if archs:
                cmake_args += ["-DCMAKE_OSX_ARCHITECTURES={}".format(";".join(archs))]

        # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
        # across all generators.
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            # self.parallel is a Python 3 only way to set parallel jobs by hand
            # using -j in the build_ext call, not supported by pip or PyPA-build.
            if hasattr(self, "parallel") and self.parallel:
                # CMake 3.12+ only.
                build_args += [f"-j{self.parallel}"]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(
            ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp
        )
        subprocess.check_call(
            ["cmake", "--build", "."] + build_args, cwd=self.build_temp
        )

        subprocess.run(
            "mkdir -p projectaria_tools-stubs/projectaria_tools/",
            shell=True,
            check=True,
        )

        subprocess.run(
            f"cp -r projectaria_tools-stubs/projectaria_tools {self.build_lib}/",
            shell=True,
            check=True,
        )


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
        install_requires=["numpy", "rerun-sdk>=0.12.0", "tqdm"],
        extras_require={
            "all": [
                ## Required for datasets
                "requests",
                ## Required for tutorial/quickstart notebooks
                "jupyter",
                "matplotlib",
                "pandas",
                "pillow",
                "plotly",
                "scipy",
                ## Required for vrs_to_mp4
                "moviepy",
            ]
        },
        entry_points={
            "console_scripts": [
                "adt_benchmark_dataset_downloader = projectaria_tools.projects.adt.adt_benchmark_dataset_downloader:main",
                "adt_challenge_dataset_downloader = projectaria_tools.projects.adt.adt_challenge_dataset_downloader:main",
                "adt_challenge_prepare_submission = projectaria_tools.projects.adt.adt_challenge_prepare_submission:main",
                "aea_dataset_downloader = projectaria_tools.projects.aea.aria_everyday_activities_dataset_downloader:main",
                "viewer_aria_sensors = projectaria_tools.utils.viewer_aria_sensors:main",
                "viewer_mps = projectaria_tools.utils.viewer_mps:main",
                "viewer_projects_adt = projectaria_tools.utils.viewer_projects_adt:main",
                "viewer_projects_ase = projectaria_tools.utils.viewer_projects_ase:main",
                "vrs_to_mp4 = projectaria_tools.utils.vrs_to_mp4:main",
                "run_vrs_health_check = projectaria_tools.utils.run_vrs_health_check:main",
            ]
        },
        packages=find_packages(),
        license="Apache-2.0",
    )


if __name__ == "__main__":
    main()
