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

#ifndef D2__CONTROLLER__SMOOTH_PATH_HPP_
#define D2__CONTROLLER__SMOOTH_PATH_HPP_

#include <deque>
#include <limits>

#include "Eigen/Dense"

namespace d2::controller
{

namespace
{

inline std::tuple<Eigen::Vector2d, double> calculate_prev_point_with_distance(
  const std::deque<Eigen::Vector2d> & path,
  std::deque<Eigen::Vector2d>::const_iterator itr,
  double distance)
{
  while (itr != path.begin()) {
    const auto prev_itr = std::prev(itr);
    const auto diff = *itr - *prev_itr;
    const auto diff_norm = diff.norm();
    if (diff_norm > distance) {
      return {*itr - diff * (distance / diff_norm), 0.0};
    }
    distance -= diff_norm;
    itr = prev_itr;
  }
  return {path.front(), distance};
}

inline std::tuple<Eigen::Vector2d, double> calculate_next_point_with_distance(
  const std::deque<Eigen::Vector2d> & path,
  std::deque<Eigen::Vector2d>::const_iterator itr,
  double distance)
{
  while (std::next(itr) != path.end()) {
    const auto next_itr = std::next(itr);
    const auto diff = *next_itr - *itr;
    const auto diff_norm = diff.norm();
    if (diff_norm > distance) {
      return {*itr + diff * (distance / diff_norm), 0.0};
    }
    distance -= diff_norm;
    itr = next_itr;
  }
  return {path.back(), distance};
}

inline std::deque<Eigen::Vector2d> smooth_path(
  const std::deque<Eigen::Vector2d> & path,
  const double smooth_distance)
{
  std::deque<Eigen::Vector2d> smoothed_path;
  for (auto itr = path.begin(); itr != path.end(); ++itr) {
    auto [prev_point, prev_distance_remain] = calculate_prev_point_with_distance(path, itr, smooth_distance);
    auto [next_point, next_distance_remain] = calculate_next_point_with_distance(path, itr, smooth_distance);
    auto prev_distance = smooth_distance - prev_distance_remain;
    auto next_distance = smooth_distance - next_distance_remain;
    if (prev_distance + next_distance == 0.0) {
      smoothed_path.push_back(*itr);
      continue;
    }
    smoothed_path.push_back((next_distance * prev_point + prev_distance * next_point) / (prev_distance + next_distance));
  }
  return smoothed_path;
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__SMOOTH_PATH_HPP_
