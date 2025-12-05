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

#ifndef D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH_WITH_DIRECTION_HPP_
#define D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH_WITH_DIRECTION_HPP_

#include <deque>
#include <limits>
#include <list>
#include <vector>
#include <forward_list>

#include "d2/controller/calculate_obstacle_avoidance_path.hpp"
#include "Eigen/Dense"

namespace d2::controller
{

namespace
{

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  const std::deque<Eigen::Vector2d> & original_path_points,
  const std::vector<Line> & obstacle_polygon, double threshold_distance,
  bool forward)
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

  auto cnt = 0;
  std::deque<std::pair<IntersectionInfo, const Line *>> intersection_infos;
  for (const auto & original_line : original_path) {
    auto intersection_info_parts = create_intersection_info_list(original_line, obstacle_polygon);
    for (const auto & intersection_info_part : intersection_info_parts) {
      intersection_infos.emplace_back(intersection_info_part, &original_line);
    }
    // for (const auto & intersection_info : intersection_info_parts) {
    //   intersection_info_itr = intersection_infos.insert_after(
    //     intersection_info_itr,
    //     {intersection_info, &original_line});
    //   ++cnt;
    // }
  }

  if (intersection_infos.empty()) {
    return original_path_points;
  }

  std::sort(intersection_infos.begin(), intersection_infos.end(), [](const auto & a, const auto & b){
    return a.second < b.second || (a.second == b.second && a.first.path_rate < b.first.path_rate);
  });

  std::deque<Eigen::Vector2d> acoidance_path;

  if (intersection_obstacle_itr == obstacle_polygon.end()) {
    for (auto p = original_path.data(); p <= intersection_infos.front().second; ++p) {
      acoidance_path.emplace_back(p->point);
    }

    // acoidance_path.emplace_back(intersection_infos.front().second->point + intersection_infos.front().second->direction * 
    //   intersection_infos.front().first.path_rate);

    if (intersection_infos.size() % 2 == 1) {
      return acoidance_path;
    }
    intersection_obstacle_itr = intersection_infos.front().first.obstacle_itr;
  }
  else {
    if (intersection_infos.size() % 2 == 0) {
      return {};
    }
    acoidance_path.push_back(intersection_obstacle_itr->point + intersection_obstacle_itr->direction * intersection_obstacle_rate);

    if (intersection_infos.size() == 1) {
      auto intersection_infos_back = intersection_infos.back();

      for (auto p = intersection_infos_back.second; p < original_path.end().base(); ++p) {
        acoidance_path.emplace_back(p->point + p->direction);
      }

      return acoidance_path;
    }
  }

  auto intersection_infos_back = intersection_infos.back();
  if (forward) {
    auto itr = intersection_obstacle_itr;
    auto end = intersection_infos_back.first.obstacle_itr;
    if (itr < end) {
      for (;itr < end; ++itr) {
        acoidance_path.emplace_back(itr->point + itr->direction);
      }
    }
    else {
      for (;itr < obstacle_polygon.end(); ++itr) {
        acoidance_path.emplace_back(itr->point + itr->direction);
      }
      for (itr = obstacle_polygon.begin(); itr < end; ++itr) {
        acoidance_path.emplace_back(itr->point + itr->direction);
      }
    }
  }
  else {
    auto itr = std::prev(intersection_obstacle_itr);
    auto end = intersection_infos_back.first.obstacle_itr;
    if (itr >= end) {
      for (; itr >= end; --itr) {
        acoidance_path.emplace_back(itr->point + itr->direction);
      }
    }
    else {
      for (;itr >= obstacle_polygon.begin(); --itr) {
        acoidance_path.emplace_back(itr->point + itr->direction);
      }
      for (itr = std::prev(obstacle_polygon.end()); itr >= end; --itr) {
        acoidance_path.emplace_back(itr->point + itr->direction);
      }
    }
  }

  // acoidance_path.emplace_back(intersection_infos_back.second->point + intersection_infos_back.second->direction * 
  //   intersection_infos_back.first.path_rate);

  for (auto p = intersection_infos_back.second; p < original_path.end().base(); ++p) {
    acoidance_path.emplace_back(p->point + p->direction);
  }

  return acoidance_path;

  // auto original_path_ptr = original_path.data();
  // auto original_path_rate = 0.0;
  // // std::deque<Eigen::Vector2d> avoidanced_path;
  // // std::cout << "iisize: " << cnt << std::endl;
  // // std::cout << "opsize: " << original_path.size() << std::endl;
  // while (true) {
  //   if (intersection_obstacle_itr == obstacle_polygon.end()) {
  //     // std::cout << "a" << std::endl;
  //     auto nearest_intersection_info_prev_itr = intersection_infos.end();
  //     IntersectionInfo nearest_intersection_info = {};
  //     auto nearest_line_ptr = original_path.cend().base();
  //     for (auto intersection_info_itr = intersection_infos.before_begin();
  //          std::next(intersection_info_itr) != intersection_infos.end(); ++intersection_info_itr) {

  //       auto [intersection_info, line_ptr] = *std::next(intersection_info_itr);
  //       bool is_after_current_point = line_ptr > original_path_ptr ||
  //         (line_ptr == original_path_ptr && intersection_info.path_rate > original_path_rate);
  //       bool is_before_nearest_point = line_ptr < nearest_line_ptr ||
  //         (line_ptr == nearest_line_ptr && intersection_info.path_rate < nearest_line_ptr->direction.norm());
  //       if (is_after_current_point && is_before_nearest_point) {
  //         nearest_intersection_info_prev_itr = intersection_info_itr;
  //         nearest_intersection_info = std::next(nearest_intersection_info_prev_itr)->first;
  //         nearest_line_ptr = std::next(nearest_intersection_info_prev_itr)->second;
  //       }
  //     }
  //     // std::cout << "b" << std::endl;

  //     if (nearest_intersection_info_prev_itr == intersection_infos.end()) {
  //       for(auto p = original_path_ptr; p < nearest_line_ptr; ++p) {
  //         avoidance_path.push_back(p->point + p->direction);
  //       }
  //       break;
  //     }

  //     // auto [nearest_intersection_info, nearest_line_ptr] = *std::next(nearest_intersection_info_prev_itr);
  //     for(auto p = original_path_ptr; p < nearest_line_ptr; ++p) {
  //       avoidance_path.push_back(p->point + p->direction);
  //     }

  //     // std::cout << "c" << std::endl;
  //     intersection_obstacle_itr = nearest_intersection_info.obstacle_itr;
  //     intersection_obstacle_rate = nearest_intersection_info.obstacle_rate;
  //     intersection_infos.erase_after(nearest_intersection_info_prev_itr);
  //   }
  //   else if (forward) {
  //     if (intersection_infos.empty()) {
  //       break;
  //     }
  //     // std::cout << "d" << std::endl;
  //     auto nearest_intersection_info_prev_itr = intersection_infos.before_begin();
  //     for (auto intersection_info_itr = intersection_infos.before_begin();
  //          std::next(intersection_info_itr) != intersection_infos.end(); ++intersection_info_itr) {

  //       auto [intersection_info, line_ptr] = *std::next(intersection_info_itr);
  //       auto [nearest_intersection_info, nearest_line_ptr] = *std::next(nearest_intersection_info_prev_itr);
  //       bool is_after_current_point = intersection_obstacle_itr < intersection_info.obstacle_itr ||
  //         (intersection_obstacle_itr == intersection_info.obstacle_itr && intersection_obstacle_rate < intersection_info.obstacle_rate);
  //       bool is_after_nearest_point = nearest_intersection_info.obstacle_itr < intersection_info.obstacle_itr ||
  //         (nearest_intersection_info.obstacle_itr == intersection_info.obstacle_itr && nearest_intersection_info.path_rate < intersection_info.path_rate);
  //       bool is_nearest_after_current_point = intersection_obstacle_itr < nearest_intersection_info.obstacle_itr ||
  //         (intersection_obstacle_itr == nearest_intersection_info.obstacle_itr && intersection_obstacle_rate < nearest_intersection_info.path_rate);
  //       if (!is_after_current_point ^ is_after_nearest_point ^ is_nearest_after_current_point) {
  //         nearest_intersection_info_prev_itr = intersection_info_itr;
  //       }
  //     }
  //     // std::cout << "e" << std::endl;
  //     auto [nearest_intersection_info, nearest_line_ptr] = *std::next(nearest_intersection_info_prev_itr);
  //     bool is_after_current_point = intersection_obstacle_itr < nearest_intersection_info.obstacle_itr ||
  //       (intersection_obstacle_itr == nearest_intersection_info.obstacle_itr && intersection_obstacle_rate < nearest_intersection_info.path_rate);
  //     if (is_after_current_point) {
  //       for (auto p = intersection_obstacle_itr; p < nearest_intersection_info.obstacle_itr; ++p) {
  //         avoidance_path.push_back(p->point + p->direction);
  //       }
  //     }
  //     else {
  //       for (auto p = intersection_obstacle_itr; p < obstacle_polygon.end(); ++p) {
  //         avoidance_path.push_back(p->point + p->direction);
  //       }
  //       for (auto p = obstacle_polygon.begin(); p < nearest_intersection_info.obstacle_itr; ++p) {
  //         avoidance_path.push_back(p->point + p->direction);
  //       }
  //     }

  //     // std::cout << "f" << std::endl;
  //     intersection_obstacle_itr = obstacle_polygon.end();
  //     intersection_infos.erase_after(nearest_intersection_info_prev_itr);
  //   }
  //   else {
  //     if (intersection_infos.empty()) {
  //       break;
  //     }
  //     auto nearest_intersection_info_prev_itr = intersection_infos.before_begin();
  //     for (auto intersection_info_itr = intersection_infos.before_begin();
  //          std::next(intersection_info_itr) != intersection_infos.end(); ++intersection_info_itr) {

  //       auto [intersection_info, line_ptr] = *std::next(intersection_info_itr);
  //       auto [nearest_intersection_info, nearest_line_ptr] = *std::next(nearest_intersection_info_prev_itr);
  //       bool is_before_current_point = intersection_info.obstacle_itr < intersection_obstacle_itr ||
  //         (intersection_info.obstacle_itr == intersection_obstacle_itr && intersection_info.path_rate < intersection_obstacle_rate);
  //       bool is_before_nearest_point = intersection_info.obstacle_itr < nearest_intersection_info.obstacle_itr ||
  //         (intersection_info.obstacle_itr == nearest_intersection_info.obstacle_itr && intersection_info.path_rate < nearest_intersection_info.path_rate);
  //       bool is_nearest_before_current_point = nearest_intersection_info.obstacle_itr < intersection_obstacle_itr ||
  //         (nearest_intersection_info.obstacle_itr == intersection_obstacle_itr && nearest_intersection_info.path_rate < intersection_obstacle_rate);
  //       if (!is_before_current_point ^ is_before_nearest_point ^ is_nearest_before_current_point) {
  //         nearest_intersection_info_prev_itr = intersection_info_itr;
  //       }
  //     }
  //     auto [nearest_intersection_info, nearest_line_ptr] = *std::next(nearest_intersection_info_prev_itr);
  //     bool is_before_current_point = nearest_intersection_info.obstacle_itr < intersection_obstacle_itr ||
  //       (nearest_intersection_info.obstacle_itr == intersection_obstacle_itr && nearest_intersection_info.path_rate < intersection_obstacle_rate);
  //     if (is_before_current_point) {
  //       for (auto p = std::prev(nearest_intersection_info.obstacle_itr); p >= intersection_obstacle_itr; --p) {
  //         avoidance_path.push_back(p->point + p->direction);
  //       }
  //     }
  //     else {
  //       for (auto p = std::prev(intersection_obstacle_itr); p >= obstacle_polygon.begin(); --p) {
  //         avoidance_path.push_back(p->point + p->direction);
  //       }
  //       for (auto p = std::prev(obstacle_polygon.end()); p >= nearest_intersection_info.obstacle_itr; --p) {
  //         avoidance_path.push_back(p->point + p->direction);
  //       }
  //     }
  //     intersection_obstacle_itr = obstacle_polygon.end();
  //     intersection_infos.erase_after(nearest_intersection_info_prev_itr);
  //   }
  // }

  // return avoidance_path;
}

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  std::deque<Eigen::Vector2d> path,
  const std::vector<std::vector<Line>> & obstacle_polygons, double threshold_distance, bool forward)
{
  for (const auto & obstacle_polygon : obstacle_polygons) {
    path = calculate_obstacle_avoidance_path(path, obstacle_polygon, threshold_distance, forward);
  }
  return path;
}

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  std::deque<Eigen::Vector2d> path,
  const std::vector<Eigen::Vector2d> & obstacle_polygon, double threshold_distance, bool forward)
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

  return calculate_obstacle_avoidance_path(path, polygon_lines, threshold_distance, forward);
}

inline std::deque<Eigen::Vector2d> calculate_obstacle_avoidance_path(
  std::deque<Eigen::Vector2d> path,
  const std::vector<std::vector<Eigen::Vector2d>> & obstacle_polygons, double threshold_distance, bool forward)
{
  int a = 0;
  for (const auto & obstacle_polygon_points : obstacle_polygons) {
    // std::cout << ++a << std::endl;
    path = calculate_obstacle_avoidance_path(path, obstacle_polygon_points, threshold_distance, forward);
    a += obstacle_polygon_points.size();
    std::cout << "path size: " << path.size() << std::endl;
  }
  std::cout << "total obstacle points: " << a << std::endl;
  return path;
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH_WITH_DIRECTION_HPP_
