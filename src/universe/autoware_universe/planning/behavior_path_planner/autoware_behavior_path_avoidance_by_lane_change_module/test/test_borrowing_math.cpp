// Copyright 2026 BYD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// SPDX-License-Identifier: Apache-2.0
// Deliberately independent of ROS so prediction arithmetic can also be verified on host machines.
#include "borrowing_math.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

using autoware::behavior_path_planner::borrowing::arrivalTimes;
using autoware::behavior_path_planner::borrowing::brakingVelocity;
using autoware::behavior_path_planner::borrowing::coversReturn;

int main()
{
  const std::vector<double> distance{0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<double> limits(distance.size(), 1.0);
  const auto cruise = arrivalTimes(distance, limits, 1.0, 0.5, 1.0, 1.0);
  assert(cruise.size() == distance.size());
  assert(std::abs(cruise.back() - 4.0) < 1e-9);
  const auto start = arrivalTimes(distance, limits, 0.0, 0.5, 1.0, 1.0);
  const auto slower_start = arrivalTimes(distance, limits, 0.0, 0.25, 1.0, 1.0);
  assert(start.back() > cruise.back());
  assert(slower_start.back() > start.back());
  assert(coversReturn(cruise, 3.0, 7.0));
  assert(!coversReturn(cruise, 3.0, 6.99));  // Never silently truncate the return.
  assert(!coversReturn({}, 3.0, 30.0));
  assert(arrivalTimes(distance, {1.0, 1.0, 0.0, 1.0, 1.0}, 0.0, 0.5, 1.0, 1.0).empty());
  assert(arrivalTimes({0.0, 0.0}, {1.0, 1.0}, 0.0, 0.5, 1.0, 1.0).empty());
  assert(arrivalTimes(distance, {}, 0.0, 0.5, 1.0, 1.0).empty());
  assert(arrivalTimes(distance, limits, 0.0, 0.0, 1.0, 1.0).empty());
  assert(arrivalTimes(distance, limits, 4.0, 0.5, 0.1, 1.0).empty());
  const auto crawl = arrivalTimes(distance, std::vector<double>(5, 0.1), 0.1, 0.5, 1.0, 1.0);
  assert(std::abs(crawl.back() - 40.0) < 1e-9);  // No artificial minimum ego speed.
  assert(!coversReturn(crawl, 3.0, 30.0));
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  assert(arrivalTimes(distance, limits, nan, 0.5, 1.0, 1.0).empty());
  assert(arrivalTimes(distance, limits, 0.0, nan, 1.0, 1.0).empty());
  assert(arrivalTimes({0.0, nan}, {1.0, 1.0}, 0.0, 0.5, 1.0, 1.0).empty());
  assert(!coversReturn(cruise, nan, 30.0));
  assert(!coversReturn(cruise, 3.0, std::numeric_limits<double>::infinity()));
  assert(brakingVelocity(-1.0, 1.0) == 0.0);
  assert(brakingVelocity(0.0, 1.0) == 0.0);
  assert(std::abs(brakingVelocity(2.0, 1.0) - 2.0) < 1e-9);
  // Acceleration/deceleration constraints and monotone time for a range of speeds.
  for (double initial = 0.0; initial < 2.0; initial += 0.1) {
    const auto times = arrivalTimes(distance, std::vector<double>(5, 2.0), initial, 0.5, 1.0, 1.0);
    assert(times.size() == distance.size());
    for (size_t i = 1; i < times.size(); ++i) assert(times[i] > times[i - 1]);
  }
  return 0;
}
