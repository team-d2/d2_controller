// Copyright 2025 miyajimad0620
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

#ifndef D2__CONTROLLER__REMOVE_PATH_SELF_INTERSECTION_HPP_
#define D2__CONTROLLER__REMOVE_PATH_SELF_INTERSECTION_HPP_

#include <limits>

#include "Eigen/Dense"

#include "d2/controller/calculate_obstacle_avoidance_path.hpp"

namespace d2::controller
{

namespace
{

inline std::deque<Eigen::Vector2d> remove_path_self_intersection(const std::deque<Eigen::Vector2d> & path_points)
{
  if (path_points.size() < 4) {
    return path_points;
  }

  std::vector<Line> path;
  path.reserve(path_points.size());
  for (auto itr = std::next(path_points.begin()); itr != path_points.end(); ++itr) {
    const auto prev_itr = std::prev(itr);
    path.push_back(
      Line{*prev_itr, *itr - *prev_itr});
  }

  Line line = path.front();
  std::deque<Eigen::Vector2d> self_intersection_removed_path_points;
  for (auto itr = std::next(path.begin()); itr != path.end(); ++itr) {
    self_intersection_removed_path_points.push_back(line.point);
    bool is_intersected = false;
    for (auto jtr = std::prev(path.end()); jtr >= itr; --jtr) {
      const auto [a_rate, b_rate] = calculate_line_cross_point_rates(line, *jtr);
      if (a_rate < 0.0 || 1.0 <= a_rate || b_rate < 0.0 || 1.0 <= b_rate) {
        continue;
      }
      is_intersected = true;
      line.point = line.point + line.direction * a_rate;
      line.direction = jtr->direction * (1.0 - b_rate);
      itr = jtr;
      break;
    }

    if (!is_intersected) {
      line = *itr;
    }
  }

  self_intersection_removed_path_points.push_back(line.point + line.direction);

  return self_intersection_removed_path_points;
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__REMOVE_PATH_SELF_INTERSECTION_HPP_
