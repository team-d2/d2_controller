#ifndef D2__CONTROLLER__LOCAL_PLANNER_HPP_
#define D2__CONTROLLER__LOCAL_PLANNER_HPP_

#include <vector>
#include <optional>

#include "vector3.hpp"
#include "stamped.hpp"

namespace d2::controller::ros2
{

template <class Path>
std::vector<Stamped<Vector3>> create_local_plan(const Path & global_plan, const Stamped<Vector3> & position)
{
  const auto global_plan_size = std::size(global_plan);

  if (global_plan_size == 0) {
    return {};
  }

  const auto global_plan_begin = std::begin(global_plan);
  const auto global_plan_end = std::end(global_plan);
  if (global_plan_size == 1) {
    return { position, *global_plan_begin };
  }

  // find position to global plan
  auto global_plan_start_itr = global_plan_begin;
  double min_distance2 = distance2(global_plan_begin->data, position.data);
  for (auto global_plan_last_itr = global_plan_begin, global_plan_itr = std::next(global_plan_begin); 
        global_plan_itr != global_plan_end;
        global_plan_last_itr = global_plan_itr, global_plan_itr = std::next(global_plan_itr))
  {
    // パスの点とpositionの距離を計算
    const auto & point = global_plan_itr->data;
    const auto position_vec = position.data - point;
    const auto position_vec_length2 = length2(position_vec);

    // パスの点とパスの前回の点の距離を計算
    const auto & last_point = global_plan_last_itr->data;
    const auto reverce_vec = last_point - point;
    const auto reverce_vec_length2 = length2(reverce_vec);
    if (reverce_vec_length2 == 0) {
      continue; // パスの点が同一の場合はスキップ
    }
    const auto reverce_vec_length = std::sqrt(reverce_vec_length2);
    const auto reverce_vec_norm = reverce_vec / reverce_vec_length;

    // パス上にpositionを投影
    const auto on_line_distance = dot(reverce_vec_norm, position_vec);

    // 投影した点がパスの始点よりも前にある場合はスキップ
    if (on_line_distance >= reverce_vec_length) {
      continue;
    }

    // パスとpositionの距離を計算
    const auto distance2 = on_line_distance > 0 ? 
      length2(rot(reverce_vec_norm, position_vec)) : // 現在地とパスの距離
      position_vec_length2; // 現在地とパスの終点の距離

    // 更新
    if (distance2 < min_distance2) {
      min_distance2 = distance2;
      global_plan_start_itr = global_plan_itr;
    }
  }

  // ローカルプランを作成
  std::vector<Stamped<Vector3>> local_plan;
  local_plan.reserve(std::distance(global_plan_start_itr, global_plan_end) + 1);
  local_plan.push_back(position);
  for (auto global_plan_itr = global_plan_start_itr; global_plan_itr != global_plan_end; ++global_plan_itr) {
    local_plan.push_back(*global_plan_itr);
  }
  return local_plan;
}

}

#endif // D2__CONTROLLER__LOCAL_PLANNER_HPP_
