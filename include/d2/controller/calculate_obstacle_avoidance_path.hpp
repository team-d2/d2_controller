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

#ifndef D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH_HPP_
#define D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH_HPP_

#include <deque>
#include <limits>
#include <list>
#include <vector>

#include "Eigen/Dense"

namespace d2::controller
{

namespace
{

struct Line
{
  Eigen::Vector2d point;
  Eigen::Vector2d direction;
};

inline std::pair<double, double> calculate_line_cross_point_rates(
  Line line_a,
  Line line_b)
{
  const auto det = line_a.direction.x() * line_b.direction.y() - line_a.direction.y() * line_b.direction.x();
  if (det == 0.0) {
    return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }
  const auto inv_det = 1.0 / det;
  const auto p_diff = line_b.point - line_a.point;
  const auto a_rate = inv_det * (line_b.direction.y() * p_diff.x() - line_b.direction.x() * p_diff.y());
  const auto b_rate = inv_det * (line_a.direction.y() * p_diff.x() - line_a.direction.x() * p_diff.y());
  return {a_rate, b_rate};
}

inline bool is_point_in_polygon(
  const Eigen::Vector2d & point,
  const std::vector<Line> & polygon)
{
    const auto is_intersect = [&point](const Line & line) {
      const auto & p0 = line.point;
      const auto p1 = line.point + line.direction;
      return ((p0.y() > point.y()) != (p1.y() > point.y())) &&
             (point.x() < (p1.x() - p0.x()) * (point.y() - p0.y()) / (p1.y() - p0.y()) + p0.x());
  };

  bool is_inside = false;
  for (const auto & line : polygon) {
    if (is_intersect(line)) {
      is_inside = !is_inside;
    }
  }
  return is_inside;
}

inline double calculate_path_length(
  std::vector<Line>::const_iterator start_itr,
  std::vector<Line>::const_iterator end_itr)
{
  double length = 0.0;
  for (auto itr = start_itr; itr != end_itr; ++itr) {
    length += itr->direction.norm();
  }
  return length;
}

struct IntersectionInfo
{
  std::vector<Line>::const_iterator obstacle_itr;
  double obstacle_rate;
  double path_rate;
};

std::list<IntersectionInfo> create_intersection_info_list(
  const Line & path_line,
  const std::vector<Line> & obstacle_polygon)
{
  std::list<IntersectionInfo> intersection_infos;
  for (auto obstacle_line_itr = obstacle_polygon.begin();
       obstacle_line_itr != obstacle_polygon.end(); ++obstacle_line_itr) {
    auto [path_rate, obstacle_rate] = calculate_line_cross_point_rates(path_line, *obstacle_line_itr);
    if (path_rate < 0.0 || 1.0 <= path_rate || obstacle_rate < 0.0 || 1.0 <= obstacle_rate) {
      continue;
    }
    auto intersection_infos_itr = intersection_infos.begin();
    while (intersection_infos_itr != intersection_infos.end()) {
      if (path_rate < intersection_infos_itr->path_rate) {
        break;
      }
      ++intersection_infos_itr;
    }
    intersection_infos.insert(intersection_infos_itr, IntersectionInfo{
      obstacle_line_itr,
      obstacle_rate,
      path_rate});
  }

  return intersection_infos;
}

inline std::tuple<Eigen::Vector2d, double> calculate_nearest_point_on_line_with_rate(
  const Eigen::Vector2d & point,
  const Line & line)
{
  const auto direction_sq = line.direction.squaredNorm();
  if (direction_sq == 0.0) {
    return {line.point, 0.0};
  }
  const auto dot = line.direction.dot(point - line.point);
  const auto is_dot_backward = dot < 0.0;
  const auto is_dot_forward = dot > direction_sq;
  if (!is_dot_backward && !is_dot_forward) {
    const auto rate = dot / direction_sq;
    return {line.point + line.direction * rate, rate};
  }
  else if (is_dot_backward) {
    return {line.point, 0.0};
  }
  else {
    return {line.point + line.direction, 1.0};
  }
}

inline std::tuple<Eigen::Vector2d, std::vector<Line>::const_iterator, double, double> calculate_nearest_point_on_polygon_with_itr_and_distance_and_rate(
  const Eigen::Vector2d & point,
  const std::vector<Line> & polygon)
{
  auto min_distance_sq = std::numeric_limits<double>::infinity();
  std::vector<Line>::const_iterator nearest_line_itr;
  Eigen::Vector2d nearest_point;
  double nearest_rate = 0.0;
  for (auto itr = polygon.begin(); itr != polygon.end(); ++itr) {
    const auto [candidate_point, rate] = calculate_nearest_point_on_line_with_rate(point, *itr);
    const auto distance_sq = (candidate_point - point).squaredNorm();
    if (distance_sq >= min_distance_sq) {
      continue;
    }
    min_distance_sq = distance_sq;
    nearest_point = candidate_point;
    nearest_line_itr = itr;
    nearest_rate = rate;
  }
  std::cout << min_distance_sq << std::endl;
  return {nearest_point, nearest_line_itr, std::sqrt(min_distance_sq), nearest_rate};
}

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  const std::deque<Eigen::Vector2d> & original_path_points,
  const std::vector<Line> & obstacle_polygon, double threshold_distance)
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
    if (distance > threshold_distance) {
      return {};
    }
    std::cout << distance << std::endl;

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
        auto start_line_length = intersection_obstacle_itr->direction.norm();
        auto end_line_length = intersection_info.obstacle_itr->direction.norm();
        auto obstacle_dist1 = calculate_path_length(intersection_obstacle_itr, intersection_info.obstacle_itr)
          - start_line_length * intersection_obstacle_rate
          + end_line_length * intersection_info.obstacle_rate;
        auto obstacle_dist2 = calculate_path_length(intersection_info.obstacle_itr, obstacle_polygon.end())
          + calculate_path_length(obstacle_polygon.begin(), intersection_obstacle_itr)
          + start_line_length * intersection_obstacle_rate
          - end_line_length * intersection_info.obstacle_rate;
      // std::cout << "f" << std::endl;
        if (obstacle_dist1 < obstacle_dist2) {
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
        auto start_line_length = intersection_obstacle_itr->direction.norm();
        auto end_line_length = intersection_info.obstacle_itr->direction.norm();
        auto obstacle_dist1 = calculate_path_length(intersection_info.obstacle_itr, intersection_obstacle_itr)
          - end_line_length * intersection_info.obstacle_rate
          + start_line_length * intersection_obstacle_rate;
        auto obstacle_dist2 = calculate_path_length(intersection_obstacle_itr, obstacle_polygon.end())
          + calculate_path_length(obstacle_polygon.begin(), intersection_info.obstacle_itr)
          + end_line_length * intersection_info.obstacle_rate
          - start_line_length * intersection_obstacle_rate;
        if (obstacle_dist1 < obstacle_dist2) {
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
  const std::vector<std::vector<Line>> & obstacle_polygons, double threshold_distance)
{
  for (const auto & obstacle_polygon : obstacle_polygons) {
    path = calculate_obstacle_avoidance_path(path, obstacle_polygon, threshold_distance);
  }
  return path;
}

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  std::deque<Eigen::Vector2d> path,
  const std::vector<Eigen::Vector2d> & obstacle_polygon, double threshold_distance)
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

  return calculate_obstacle_avoidance_path(path, polygon_lines, threshold_distance);
}

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  std::deque<Eigen::Vector2d> path,
  const std::vector<std::vector<Eigen::Vector2d>> & obstacle_polygons, double threshold_distance  )
{
  for (const auto & obstacle_polygon_points : obstacle_polygons) {
    path = calculate_obstacle_avoidance_path(path, obstacle_polygon_points, threshold_distance);
  }
  return path;
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH_HPP_
