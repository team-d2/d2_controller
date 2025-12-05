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

#ifndef D2__CONTROLLER__CALCULATE_IN_CIRCLE_POINTS_HPP_
#define D2__CONTROLLER__CALCULATE_IN_CIRCLE_POINTS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include <chrono>
#include <optional>

#include "Eigen/Dense"
#include "d2/controller/calculate_circle_line_cross_point.hpp"

namespace d2::controller
{

namespace
{

std::vector<Eigen::Vector2d> calculate_in_circle_points(
    const double circle_radius,
    const std::vector<Eigen::Vector2d> & obstacle_polygon)
{
  if (obstacle_polygon.empty()) {
    return {};
  }

  std::vector<Eigen::Vector2d> in_circle_points;
  in_circle_points.reserve(obstacle_polygon.size());

  bool prev_is_in_circle = obstacle_polygon.back().norm() < circle_radius;
  auto prev_point_ptr = &obstacle_polygon.back();

  for (const auto & point : obstacle_polygon) {
    const bool is_in_circle = point.norm() < circle_radius;

    if (is_in_circle) {
      if (!prev_is_in_circle) {
        // 外から内へ
        auto & prev_point = *prev_point_ptr;
        in_circle_points.push_back(calculate_circle_line_cross_point_from_inner_outer_points(
          circle_radius, point, prev_point));
      }
      in_circle_points.push_back(point);
    } else if (prev_is_in_circle) {
      // 内から外へ
      auto & prev_point = *prev_point_ptr;
      in_circle_points.push_back(calculate_circle_line_cross_point_from_inner_outer_points(
        circle_radius, prev_point, point));
    }

    prev_is_in_circle = is_in_circle;
    prev_point_ptr = &point;
  }

  return in_circle_points;
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_IN_CIRCLE_POINTS_HPP_
