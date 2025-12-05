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

#ifndef D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH3_HPP_
#define D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH3_HPP_

#include <deque>
#include <limits>
#include <list>
#include <vector>

#include "d2/controller/calculate_obstacle_avoidance_path.hpp"
#include "Eigen/Dense"

namespace d2::controller
{

namespace
{

struct ObstacleAvoidancePathParams
{
  bool is_avoidance_direction_forward;
  double penetration_limit;
};


inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  const std::deque<Eigen::Vector2d> & original_path_points,
  const std::vector<Line> & obstacle_polygon, const ObstacleAvoidancePathParams & param)
{
  // path_size_check
  if (original_path_points.empty()) {
    return {};
  }

  auto avoidance_path = std::deque<Eigen::Vector2d>{original_path_points.front()};
  std::vector<Line>::const_iterator intersection_obstacle_itr;
  double intersection_obstacle_rate = 0.0;
  // std::cout << "z" << std::endl;
  // check start point in obstacle
  if (is_point_in_polygon(original_path_points.front(), obstacle_polygon)) {
    // std::cout << "a" << std::endl;
    const auto [nearest_point, itr, distance, rate] =
      calculate_nearest_point_on_polygon_with_itr_and_distance_and_rate(
      original_path_points.front(), obstacle_polygon);
    // std::cout << "b" << std::endl;
    if (distance > param.penetration_limit) {
      return {};
    }
    // std::cout << distance << std::endl;

    intersection_obstacle_itr = itr;
    intersection_obstacle_rate = rate;
    avoidance_path.push_back(nearest_point);
  }
  else {
    intersection_obstacle_rate = 0.0;
    intersection_obstacle_itr = obstacle_polygon.end();
  }

  std::vector<Line> original_path;
  original_path.reserve(original_path_points.size() - 1);
  for (auto itr = original_path_points.begin(); std::next(itr) != original_path_points.end(); ++itr) {
    original_path.push_back(Line{
      *itr,
      *(std::next(itr)) - *itr});
  }

  for (const auto & original_line : original_path) {

    auto intersection_infos = create_intersection_info_list(original_line, obstacle_polygon);
    
    for (const auto & intersection_info : intersection_infos) {
      // std::cout << "d" << std::endl;
      if (intersection_obstacle_itr == obstacle_polygon.end()) {
        avoidance_path.push_back(original_line.point + original_line.direction * intersection_info.path_rate);
        intersection_obstacle_itr = intersection_info.obstacle_itr;
        intersection_obstacle_rate = intersection_info.obstacle_rate;
      // std::cout << "e" << std::endl;
      }
      else if (intersection_obstacle_itr < intersection_info.obstacle_itr) {
      // std::cout << "f" << std::endl;
        if (param.is_avoidance_direction_forward) {
          for (auto itr = std::next(intersection_obstacle_itr); itr != std::next(intersection_info.obstacle_itr); ++itr) {
            avoidance_path.push_back(itr->point);
          }
        }
        else {
          for (auto itr = intersection_obstacle_itr; itr != obstacle_polygon.begin(); --itr) {
            avoidance_path.push_back(itr->point);
          }
          avoidance_path.push_back(obstacle_polygon.front().point);
          for (auto itr = std::prev(obstacle_polygon.end()); itr != intersection_info.obstacle_itr; --itr) {
            avoidance_path.push_back(itr->point);
          }
        }
      // std::cout << "g" << std::endl;
        avoidance_path.push_back(
          original_line.point + original_line.direction * intersection_info.path_rate);
        intersection_obstacle_itr = obstacle_polygon.end();
      }
      else {
      // std::cout << "h" << std::endl;
        if (!param.is_avoidance_direction_forward) {
          for (auto itr = intersection_obstacle_itr; itr != intersection_info.obstacle_itr; --itr) {
            avoidance_path.push_back(itr->point);
          }
        }
        else {
          for (auto itr = std::next(intersection_obstacle_itr); itr != obstacle_polygon.end(); ++itr) {
            avoidance_path.push_back(itr->point);
          }
          for (auto itr = obstacle_polygon.begin(); itr != std::next(intersection_info.obstacle_itr); ++itr) {
            avoidance_path.push_back(itr->point);
          }
        }
      // std::cout << "i" << std::endl;
        avoidance_path.push_back(
          original_line.point + original_line.direction * intersection_info.path_rate);
        intersection_obstacle_itr = obstacle_polygon.end();
      }
    }
      // std::cout << "j" << std::endl;

    if (intersection_obstacle_itr == obstacle_polygon.end()) {
      avoidance_path.push_back(original_line.point + original_line.direction);
    }
      // std::cout << "k" << std::endl;
  }

      // std::cout << "l" << std::endl;
  return avoidance_path;
}

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  std::deque<Eigen::Vector2d> path,
  const std::vector<std::vector<Line>> & obstacle_polygons, const ObstacleAvoidancePathParams & param )
{
  for (const auto & obstacle_polygon : obstacle_polygons) {
    path = calculate_obstacle_avoidance_path(path, obstacle_polygon, param);
  }
  return path;
}

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  std::deque<Eigen::Vector2d> path,
  const std::vector<Eigen::Vector2d> & obstacle_polygon, const ObstacleAvoidancePathParams & param )
{
  std::vector<Line> polygon_lines;
  polygon_lines.reserve(obstacle_polygon.size());
  auto obstacle_prev_itr = obstacle_polygon.begin();
  for (auto obstacle_itr = std::next(obstacle_polygon.begin());
       obstacle_itr != obstacle_polygon.end(); obstacle_prev_itr = obstacle_itr, ++obstacle_itr) {
    polygon_lines.push_back(Line{
      *obstacle_prev_itr,
      *obstacle_itr - *obstacle_prev_itr});
  }
  // closing line
  polygon_lines.push_back(Line{
    *obstacle_prev_itr,
    *(obstacle_polygon.begin()) - *obstacle_prev_itr}); 

  return calculate_obstacle_avoidance_path(path, polygon_lines, param);
}

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  std::deque<Eigen::Vector2d> path,
  const std::vector<std::vector<Eigen::Vector2d>> & obstacle_polygons, const ObstacleAvoidancePathParams & param  )
{
  for (const auto & obstacle_polygon_points : obstacle_polygons) {
    path = calculate_obstacle_avoidance_path(path, obstacle_polygon_points, param);
  }
  return path;
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH3_HPP_
