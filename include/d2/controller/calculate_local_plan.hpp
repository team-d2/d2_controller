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

#ifndef D2__CONTROLLER__CALCULATE_LOCAL_PLAN_HPP_
#define D2__CONTROLLER__CALCULATE_LOCAL_PLAN_HPP_

#include <cstdint>
#include <iterator>
#include <limits>
#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "stamped.hpp"

namespace d2::controller::ros2
{

std::vector<Stamped<Eigen::Isometry3d>> calculate_local_plan(
  const std::vector<Stamped<Eigen::Isometry3d>> & global_plan, const Stamped<Eigen::Isometry3d> & pose)
{
  const auto global_plan_size = std::size(global_plan);

  if (global_plan_size == 0) {
    return {};
  }

  const auto global_plan_begin = std::begin(global_plan);
  const auto global_plan_end = std::end(global_plan);
  if (global_plan_size == 1) {
    return {pose, *global_plan_begin};
  }

  // find position to global plan
  auto global_plan_start_itr = global_plan_begin;
  double min_distance2 = (global_plan_begin->data.translation() - pose.data.translation()).squaredNorm();
  for (auto global_plan_last_itr = global_plan_begin,
    global_plan_itr = std::next(global_plan_begin);
    global_plan_itr != global_plan_end;
    global_plan_last_itr = global_plan_itr, global_plan_itr = std::next(global_plan_itr))
  {
    // パスの点とpositionの距離を計算
    const auto path_point = global_plan_itr->data.translation();
    const auto position_vec = pose.data.translation() - path_point;
    const auto position_vec_length2 = position_vec.squaredNorm();

    // パスの点とパスの前回の点の距離を計算
    const auto path_point_last = global_plan_last_itr->data.translation();
    const auto reverce_vec = path_point_last - path_point;
    const auto reverce_vec_length2 = reverce_vec.squaredNorm();
    if (reverce_vec_length2 == 0) {
      continue;  // パスの点が同一の場合はスキップ
    }
    const auto reverce_vec_length = std::sqrt(reverce_vec_length2);
    const auto reverce_vec_norm = reverce_vec / reverce_vec_length;

    // パス上にpositionを投影
    const auto on_line_distance = reverce_vec_norm.dot(position_vec);

    // 投影した点がパスの始点よりも前にある場合はスキップ
    if (on_line_distance >= reverce_vec_length) {
      continue;
    }

    // パスとpositionの距離を計算
    const auto distance2 = on_line_distance > 0 ?
      reverce_vec_norm.cross(position_vec).squaredNorm() // 現在地とパスの距離
      : position_vec_length2;                         // 現在地とパスの終点の距離

    // 更新
    if (distance2 < min_distance2) {
      min_distance2 = distance2;
      global_plan_start_itr = global_plan_itr;
    }
  }

  // ローカルプランを作成
  std::vector<Stamped<Eigen::Isometry3d>> local_plan;
  local_plan.reserve(std::distance(global_plan_start_itr, global_plan_end) + 1);
  local_plan.push_back(pose);
  for (auto global_plan_itr = global_plan_start_itr; global_plan_itr != global_plan_end;
    ++global_plan_itr)
  {
    local_plan.push_back(*global_plan_itr);
  }
  return local_plan;
}

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__CALCULATE_LOCAL_PLAN_HPP_
