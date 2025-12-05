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

#ifndef D2__CONTROLLER__CALCULATE_OBSTACLE_LEFT_RIGHT_POINTS_HPP_
#define D2__CONTROLLER__CALCULATE_OBSTACLE_LEFT_RIGHT_POINTS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <chrono>
#include <optional>

#include "Eigen/Dense"

namespace d2::controller
{

namespace
{

std::pair<const Eigen::Vector2d &, const Eigen::Vector2d &> calculate_obstacle_left_right_points(
    const std::vector<Eigen::Vector2d> & obstacle_polygon)
{
  if (obstacle_polygon.empty()) {
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    return {
      Eigen::Vector2d(nan, nan),
      Eigen::Vector2d(nan, nan)
    };
  }

  auto left_point_ptr = &obstacle_polygon[0];
  auto right_point_ptr = left_point_ptr;
  std::for_each(std::next(obstacle_polygon.begin()), obstacle_polygon.end(),
    [&](const Eigen::Vector2d & point) {
      const auto left_cross = left_point_ptr->x() * point.y() - left_point_ptr->y() * point.x();
      if (left_cross > 0) {
        left_point_ptr = &point;
      }
      const auto right_cross = right_point_ptr->x() * point.y() - right_point_ptr->y() * point.x();
      if (right_cross < 0) {
        right_point_ptr = &point;
      }
    });
  return {*left_point_ptr, *right_point_ptr};
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_OBSTACLE_LEFT_RIGHT_POINTS_HPP_
