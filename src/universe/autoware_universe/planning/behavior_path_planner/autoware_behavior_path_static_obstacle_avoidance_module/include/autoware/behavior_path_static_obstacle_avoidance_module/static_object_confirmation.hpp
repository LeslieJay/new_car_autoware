// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__STATIC_OBJECT_CONFIRMATION_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__STATIC_OBJECT_CONFIRMATION_HPP_

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{

// Confirmation uses perception timestamps, not planner cycles: reprocessing one message must
// never turn a single zero-speed detection into a confirmed stationary obstacle.
class StaticObjectConfirmation
{
public:
  struct Parameters
  {
    bool enabled{true};
    double observation_time{0.2};
    std::size_t minimum_observations{3};
    double position_tolerance{0.1};
    double maximum_gap{0.3};
  };

  struct Observation
  {
    std::string uuid;
    double stamp;
    double x;
    double y;
    double speed;
    double speed_threshold;
  };

  explicit StaticObjectConfirmation(const Parameters & parameters) : parameters_(parameters)
  {
    if (
      !std::isfinite(parameters.observation_time) || parameters.observation_time <= 0.0 ||
      parameters.minimum_observations < 2 || !std::isfinite(parameters.position_tolerance) ||
      parameters.position_tolerance < 0.0 || !std::isfinite(parameters.maximum_gap) ||
      parameters.maximum_gap <= 0.0) {
      throw std::invalid_argument("Invalid static obstacle confirmation parameters");
    }
  }

  void clear() { tracks_.clear(); }

  std::unordered_map<std::string, bool> update(
    const std::vector<Observation> & observations, const double now)
  {
    std::unordered_map<std::string, bool> result;
    if (!parameters_.enabled) {
      clear();
      return result;
    }
    for (const auto & observation : observations) {
      result[observation.uuid] = false;
      if (
        !std::isfinite(now) || !std::isfinite(observation.stamp) || !std::isfinite(observation.x) ||
        !std::isfinite(observation.y) || !std::isfinite(observation.speed) ||
        !std::isfinite(observation.speed_threshold) || observation.speed < 0.0 ||
        observation.speed_threshold <= 0.0 || observation.speed >= observation.speed_threshold ||
        observation.stamp > now || now - observation.stamp > parameters_.maximum_gap) {
        tracks_.erase(observation.uuid);
        continue;
      }
      auto it = tracks_.find(observation.uuid);
      if (it != tracks_.end()) {
        const auto & track = it->second;
        if (
          observation.stamp < track.last_stamp ||
          observation.stamp - track.last_stamp > parameters_.maximum_gap ||
          std::hypot(observation.x - track.anchor_x, observation.y - track.anchor_y) >
            parameters_.position_tolerance ||
          (observation.stamp == track.last_stamp &&
           (observation.x != track.last_x || observation.y != track.last_y))) {
          tracks_.erase(it);
          it = tracks_.end();
        }
      }
      if (it == tracks_.end()) {
        tracks_.emplace(
          observation.uuid, Track{
                              observation.stamp, observation.stamp, observation.x, observation.y,
                              observation.x, observation.y, 1});
        continue;
      }
      auto & track = it->second;
      if (observation.stamp > track.last_stamp) {
        ++track.observations;
        track.last_stamp = observation.stamp;
        track.last_x = observation.x;
        track.last_y = observation.y;
      }
      result[observation.uuid] =
        track.observations >= parameters_.minimum_observations &&
        observation.stamp - track.first_stamp >= parameters_.observation_time;
    }
    // A missing detection breaks continuity, even when its UUID reappears shortly afterwards.
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (result.count(it->first) == 0) {
        it = tracks_.erase(it);
      } else {
        ++it;
      }
    }
    return result;
  }

private:
  struct Track
  {
    double first_stamp;
    double last_stamp;
    double anchor_x;
    double anchor_y;
    double last_x;
    double last_y;
    std::size_t observations;
  };

  Parameters parameters_;
  std::unordered_map<std::string, Track> tracks_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__STATIC_OBJECT_CONFIRMATION_HPP_
