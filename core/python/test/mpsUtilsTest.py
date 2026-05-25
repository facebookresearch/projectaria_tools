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

import unittest
from datetime import timedelta
from types import SimpleNamespace

from projectaria_tools.core.mps.utils import bisection_timestamp_search


def _make(ns_values):
    # timedelta has microsecond resolution; use multiples of 1000 ns for exact round-trip.
    return [SimpleNamespace(tracking_timestamp=timedelta(microseconds=v / 1000)) for v in ns_values]


class BisectionTimestampSearchTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ts = _make([1000, 3000, 5000, 7000, 9000, 11000, 13000, 15000, 17000, 19000])

    def test_returns_truly_closest_neighbor(self) -> None:
        # Regression for #100: query 14900 is closer to 15000 (index 7) than to 13000 (index 6).
        self.assertEqual(bisection_timestamp_search(self.ts, 14900), 7)

    def test_equidistant_prefers_lower_index(self) -> None:
        # Tie-break is deterministic toward the lower index.
        self.assertEqual(bisection_timestamp_search(self.ts, 6000), 2)


if __name__ == "__main__":
    unittest.main()
