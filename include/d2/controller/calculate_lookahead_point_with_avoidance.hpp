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

#ifndef D2__CONTROLLER__CALCULATE_LOOKAHEAD_POINT_WITH_AVOIDANCE_HPP_
#define D2__CONTROLLER__CALCULATE_LOOKAHEAD_POINT_WITH_AVOIDANCE_HPP_

#include "Eigen/Geometry"

#include "d2/controller/stamped.hpp"
#include "d2/controller/calculate_in_circle_points.hpp"
#include "d2/controller/calculate_obstacle_left_right_points.hpp"
#include "d2/controller/calculate_lookahead_point.hpp"
#include "d2/controller/calculate_is_polygon_contains_origin.hpp"

namespace d2::controller
{
namespace
{

inline Stamped<Eigen::Vector3d> calculate_lookahead_point_with_avoidance(
  const std::vector<Stamped<Eigen::Isometry3d>> & stamped_points, const double lookahead_distance,
  std::vector<std::vector<Eigen::Vector2d>> obstacle_polygons,
  const Eigen::Vector3d & last_lookahead_point)
{
  auto lookahead_point = calculate_lookahead_point(stamped_points, lookahead_distance);
  if (lookahead_point.data.hasNaN()) {
    return lookahead_point;
  }

  auto tmp = 0;
  // std::cout << ++tmp << ": " << lookahead_distance << std::endl;

  const auto relative_lookahead_point = (lookahead_point.data.head<2>() - stamped_points.front().data.translation().head<2>()).eval();

  auto left_point = relative_lookahead_point.head<2>().eval();
  auto right_point = left_point;
  for (const auto & obstacle_polygon : obstacle_polygons) {
    if (obstacle_polygon.empty()) {
      continue;
    }
  // std::cout << ++tmp << ": " << lookahead_distance << std::endl;

    std::vector<Eigen::Vector2d> relative_obstacle_polygon;
    relative_obstacle_polygon.reserve(obstacle_polygon.size());
    for (const auto & point : obstacle_polygon) {
      relative_obstacle_polygon.push_back(point - stamped_points.front().data.translation().head<2>());
    }

    if (calculate_is_polygon_contains_origin(relative_obstacle_polygon)) {
      auto prev_point_ptr = &relative_obstacle_polygon.back();
      auto prev_dist_sq = prev_point_ptr->squaredNorm();
      auto nearest_dist_sq_sum = std::numeric_limits<double>::infinity();
      auto nearest_point_ptr = &relative_obstacle_polygon.front();
      auto nearest_prev_point_ptr = nearest_point_ptr;
      for (auto & point : relative_obstacle_polygon) {
        auto dist_sq = point.squaredNorm();
        auto dist_sq_sum = prev_dist_sq + dist_sq;
        if (dist_sq_sum < nearest_dist_sq_sum) {
          nearest_point_ptr = &point;
          nearest_prev_point_ptr = prev_point_ptr;
          nearest_dist_sq_sum = dist_sq_sum;
        }
        prev_point_ptr = &point;
        prev_dist_sq = dist_sq;
      }
      auto abs_prev_point = (nearest_prev_point_ptr->normalized() * lookahead_distance).eval();
      auto abs_point = (nearest_point_ptr->normalized() * lookahead_distance).eval();
      auto prev_point_dist_sq = (abs_prev_point - last_lookahead_point.head<2>()).squaredNorm();
      auto point_dist_sq = (abs_point - last_lookahead_point.head<2>()).squaredNorm();
      auto lookahead_point = ((prev_point_dist_sq < point_dist_sq ? abs_prev_point : abs_point).normalized() * lookahead_distance).eval();
      // std::cout << "Obstacle contains origin, adjusted lookahead_point: " << lookahead_point.transpose() << std::endl;
      return Stamped<Eigen::Vector3d>
      {
        stamped_points.front().stamp_nanosec,
        stamped_points.front().data.translation() + Eigen::Vector3d(lookahead_point.x(), lookahead_point.y(), 0.0)
      };
    }

    auto in_circle_relative_obstacle_polygon =
      calculate_in_circle_points(lookahead_distance, relative_obstacle_polygon);
    for (const auto & point : in_circle_relative_obstacle_polygon) {
      // std::cout << "obstacle point: " << point.transpose() << std::endl;
    }

    if (in_circle_relative_obstacle_polygon.empty()) {
      continue;
    }
    
    auto [obstacle_left_point, obstacle_right_point] =
      calculate_obstacle_left_right_points(in_circle_relative_obstacle_polygon);
    // std::cout << "obstacle_left_point: " << obstacle_left_point.transpose() << std::endl;
    // std::cout << "obstacle_right_point: " << obstacle_right_point.transpose() << std::endl;
    
    if (obstacle_left_point.hasNaN() || obstacle_right_point.hasNaN()) {
      continue;
    }
    
    auto obstacle_left_cross = relative_lookahead_point.x() * obstacle_left_point.y() - relative_lookahead_point.y() * obstacle_left_point.x();
    auto obstacle_right_cross = relative_lookahead_point.x() * obstacle_right_point.y() - relative_lookahead_point.y() * obstacle_right_point.x();
    if (obstacle_left_cross <= 0 || obstacle_right_cross >= 0) {
      continue;
    }

    // std::cout << "obstacle blocks lookahead point" << std::endl;

    auto obstacle_left_left_cross = left_point.x() * obstacle_left_point.y() - left_point.y() * obstacle_left_point.x();
    if (obstacle_left_left_cross > 0) {
      left_point = obstacle_left_point;
      // std::cout << "Updated left_point to obstacle_left_point: " << left_point.transpose() << std::endl;
    }

    auto obstacle_right_right_cross = right_point.x() * obstacle_right_point.y() - right_point.y() * obstacle_right_point.x();
    if (obstacle_right_right_cross < 0) {
      right_point = obstacle_right_point;
      // std::cout << "Updated right_point to obstacle_right_point: " << right_point.transpose() << std::endl;
    }
  }
  // std::cout << ++tmp << ": " << lookahead_distance << std::endl;

  auto left_point_norm_sq = (left_point + stamped_points.front().data.translation().head<2>() - last_lookahead_point.head<2>()).squaredNorm();
  auto right_point_norm_sq = (right_point + stamped_points.front().data.translation().head<2>() - last_lookahead_point.head<2>()).squaredNorm();
  auto lookahead_point_data = left_point_norm_sq < right_point_norm_sq ? left_point : right_point;
  auto lookahead_point_data_n = lookahead_point_data.normalized();
  // std::cout << "lookahead distance: " << lookahead_distance << std::endl;
  auto lookahead_point_data_normalized = lookahead_point_data_n * lookahead_distance;
  // std::cout << "Final lookahead_point_data: " << lookahead_point_data.transpose() << std::endl;
  // std::cout << "Final lookahead_point_data_n: " << lookahead_point_data_n.transpose() << std::endl;
  // std::cout << "Final lookahead_point_data_normalized: " << lookahead_point_data_normalized.transpose() << std::endl;
  return Stamped<Eigen::Vector3d>
  {
    stamped_points.front().stamp_nanosec,
    stamped_points.front().data.translation() + Eigen::Vector3d(lookahead_point_data_normalized.x(), lookahead_point_data_normalized.y(), 0.0)
  };
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_LOOKAHEAD_POINT_WITH_AVOIDANCE_HPP_