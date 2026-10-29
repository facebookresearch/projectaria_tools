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

from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


MOCK_COMMAND: str = """#!/bin/bash
set -eu
command=${0##*/}
{
    printf '%s' "$command"
    printf ' %s' "$@"
    printf '\\n'
} >> "$TRACE"
case "$command" in
    nproc) exit 77 ;;
    sleep) exit 0 ;;
    sed)
        shift 2
        for repo_file in "$@"; do
            [[ "$repo_file" == "$YUM_REPOS_DIR/"* && -f "$repo_file" ]] || exit 91
        done
        exit 0 ;;
esac
if [ "$1" = clean ] && [ "$FAIL_CLEAN" = 1 ]; then
    exit 19
fi
if [ "$1" = config-manager ] && [ "$FAIL_CONFIG" = 1 ]; then
    exit 18
fi
if [ "$1" = install ] && [ "$command:$3" = "$FAIL_INSTALL" ]; then
    count=0
    if [ -f "$COUNTER" ]; then
        read -r count < "$COUNTER"
    fi
    count=$((count + 1))
    printf '%s\\n' "$count" > "$COUNTER"
    if [ "$count" -le "$FAILURES" ]; then
        echo "Failed to download metadata: repomd.xml is empty" >&2
        exit 17
    fi
fi
"""


class InstallManylinuxDepsTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temporary_dir.cleanup)
        self.root = Path(temporary_dir.name)
        self.bin_dir = self.root / "bin"
        self.bin_dir.mkdir()
        self.repo_dir = self.root / "yum repos"
        self.repo_dir.mkdir()
        self.trace = self.root / "trace"
        self.script = Path(__file__).parent.parent / "install_manylinux_deps.sh"
        # nproc stops the real script after package setup, before native builds.
        for name in ["dnf", "yum", "sleep", "sed", "nproc"]:
            command = self.bin_dir / name
            command.write_text(MOCK_COMMAND)
            command.chmod(0o755)

    def run_installer(self, **settings: str) -> tuple[int, list[str]]:
        result = subprocess.run(
            ["/bin/bash", str(self.script)],
            env={
                **os.environ,
                "PATH": str(self.bin_dir),
                "YUM_REPOS_DIR": str(self.repo_dir),
                "TRACE": str(self.trace),
                "COUNTER": str(self.root / "counter"),
                "FAIL_INSTALL": "",
                "FAILURES": "0",
                "FAIL_CLEAN": "0",
                "FAIL_CONFIG": "0",
                **settings,
            },
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.stderr = result.stderr
        self.assertTrue(self.trace.exists(), result.stderr)
        return result.returncode, self.trace.read_text().splitlines()

    def test_success_does_not_retry_or_clean_metadata(self) -> None:
        code, calls = self.run_installer()
        self.assertEqual(code, 77)
        self.assertEqual(
            calls,
            [
                "dnf install -y dnf-plugins-core",
                "dnf config-manager --set-enabled powertools",
                "yum install -y epel-release",
                "yum install -y cmake git gtest-devel lz4-devel libzstd-devel xxhash-devel libpng-devel libjpeg-turbo-devel wget",
                "nproc ",
            ],
        )

    def test_centos_rewrites_only_use_isolated_repo_directory(self) -> None:
        repo_file = self.repo_dir / "CentOS-Base.repo"
        repo_file.write_text("# isolated test repository\n")
        code, calls = self.run_installer()
        self.assertEqual(code, 77)
        self.assertEqual(
            [call for call in calls if call.startswith("sed ")],
            [
                f"sed -i s/mirrorlist/#mirrorlist/g {repo_file}",
                f"sed -i s|#baseurl=http://mirror.centos.org|baseurl=http://vault.centos.org|g {repo_file}",
            ],
        )
        self.assertIn("dnf install -y dnf-plugins-core", calls)

    def test_dnf_metadata_failure_recovers_on_third_attempt(self) -> None:
        code, calls = self.run_installer(
            FAIL_INSTALL="dnf:dnf-plugins-core", FAILURES="2"
        )
        self.assertEqual(code, 77)
        self.assertEqual(
            calls[:7],
            [
                "dnf install -y dnf-plugins-core",
                "dnf clean metadata",
                "sleep 5",
                "dnf install -y dnf-plugins-core",
                "dnf clean metadata",
                "sleep 10",
                "dnf install -y dnf-plugins-core",
            ],
        )
        self.assertIn("dnf config-manager --set-enabled powertools", calls)
        self.assertIn("yum install -y epel-release", calls)

    def test_yum_metadata_failure_recovers_for_each_transaction(self) -> None:
        for package in ["epel-release", "cmake"]:
            with self.subTest(package=package):
                self.trace.unlink(missing_ok=True)
                (self.root / "counter").unlink(missing_ok=True)
                code, calls = self.run_installer(
                    FAIL_INSTALL=f"yum:{package}", FAILURES="1"
                )
                self.assertEqual(code, 77)
                installs = [
                    c for c in calls if c.startswith(f"yum install -y {package}")
                ]
                self.assertEqual(len(installs), 2)
                self.assertEqual(installs[0], installs[1])
                self.assertEqual(calls.count("yum clean metadata"), 1)
                self.assertEqual(calls.count("sleep 5"), 1)

    def test_yum_only_image_does_not_require_dnf(self) -> None:
        (self.bin_dir / "dnf").unlink()
        code, calls = self.run_installer(FAIL_INSTALL="yum:epel-release", FAILURES="1")
        self.assertEqual(code, 77)
        self.assertFalse(any(c.startswith("dnf ") for c in calls))
        self.assertEqual(calls.count("yum install -y epel-release"), 2)
        self.assertEqual(calls.count("yum clean metadata"), 1)

    def test_persistent_failure_is_bounded_and_preserves_exit_code(self) -> None:
        code, calls = self.run_installer(
            FAIL_INSTALL="dnf:dnf-plugins-core", FAILURES="99"
        )
        self.assertEqual(code, 17)
        self.assertEqual(calls.count("dnf install -y dnf-plugins-core"), 3)
        self.assertEqual(calls.count("dnf clean metadata"), 2)
        self.assertEqual(
            [c for c in calls if c.startswith("sleep ")], ["sleep 5", "sleep 10"]
        )
        self.assertFalse(any(c.startswith("yum ") for c in calls))
        self.assertNotIn("nproc ", calls)

    def test_failed_metadata_cleanup_does_not_prevent_recovery(self) -> None:
        code, calls = self.run_installer(
            FAIL_INSTALL="dnf:dnf-plugins-core", FAILURES="1", FAIL_CLEAN="1"
        )
        self.assertEqual(code, 77)
        self.assertEqual(
            calls[:4],
            [
                "dnf install -y dnf-plugins-core",
                "dnf clean metadata",
                "sleep 5",
                "dnf install -y dnf-plugins-core",
            ],
        )
        self.assertIn("Warning: metadata cleanup failed", self.stderr)
        self.assertIn("yum install -y epel-release", calls)

    def test_failed_cleanup_does_not_mask_exhausted_install_failure(self) -> None:
        code, calls = self.run_installer(
            FAIL_INSTALL="dnf:dnf-plugins-core", FAILURES="99", FAIL_CLEAN="1"
        )
        self.assertEqual(code, 17)
        self.assertEqual(calls.count("dnf install -y dnf-plugins-core"), 3)
        self.assertEqual(calls.count("dnf clean metadata"), 2)
        self.assertEqual(self.stderr.count("Warning: metadata cleanup failed"), 2)
        self.assertNotIn("nproc ", calls)

    def test_config_manager_failure_is_not_retried(self) -> None:
        code, calls = self.run_installer(FAIL_CONFIG="1")
        self.assertEqual(code, 18)
        self.assertEqual(
            calls,
            [
                "dnf install -y dnf-plugins-core",
                "dnf config-manager --set-enabled powertools",
            ],
        )
