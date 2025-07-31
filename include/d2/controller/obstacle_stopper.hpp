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

#ifndef D2__CONTROLLER__OBSTACLE_STOPPER_HPP_
#define D2__CONTROLLER__OBSTACLE_STOPPER_HPP_

#include <algorithm>
#include <chrono>
#include <optional>
#include <tuple>
#include <vector>

#include "bounding_box_3d.hpp"

namespace d2::controller::ros2
{

template<class Vector, class Matrix>
class ObstacleStopper
{
public:
  using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

  ObstacleStopper()
  : plan_() {}

  template<class TfStampeds>
  explicit ObstacleStopper(const TfStampeds & point_tfs)
  : plan_(create_plan(point_tfs))
  {
  }

  template<class Vectors>
  inline std::vector<std::tuple<TimePoint, Vector, Matrix>> clip_plan_by_obstacle_points(
    const BoundingBox3D & robot_bb, const Vectors & point_vecs) const
  {
    // check plan
    if (plan_.empty()) {
      return {};
    }

    auto plan_end = plan_.end();
    auto plan_back_length = plan_.back().length;
    for (const auto & point_vec : point_vecs) {
      // check point
      for (auto itr = plan_.begin(); itr != plan_end; ++itr) {
        const auto vec = itr->position_vec - point_vec;
        const auto vec_norm = vec.normalized();

        // check height
        {
          const auto point_height = vec_norm.dot(itr->rotation_mat.getColumn(2));
          if (point_height < robot_bb.z_min || point_height > robot_bb.z_max) {
            continue;
          }
        }

        // check width
        {
          const auto point_width = vec_norm.dot(itr->rotation_mat.getColumn(1));
          if (point_width < robot_bb.y_min || point_width > robot_bb.y_max) {
            continue;
          }
        }

        // check front length
        const auto point_front_length = vec_norm.dot(itr->rotation_mat.getColumn(0));
        if (
          point_front_length < robot_bb.x_min ||
          point_front_length > robot_bb.x_max + itr->length)
        {
          continue;
        }

        // update plan_back_length
        if (itr != std::prev(plan_end)) {
          plan_back_length = itr->length;
        }

        plan_back_length = std::clamp(0.0, plan_back_length, point_front_length - robot_bb.x_max);

        // update plan_end
        plan_end = std::next(itr);

        break;
      }
    }

    // create clipped plan
    std::vector<std::tuple<TimePoint, Vector, Matrix>> clipped_plan;
    clipped_plan.reserve(std::distance(plan_.begin(), plan_end) + 1);
    for (auto itr = plan_.begin(); itr != plan_end; ++itr) {
      clipped_plan.emplace_back(itr->stamp, itr->position_vec, itr->rotation_mat);
    }
    if (plan_back_length > 0.0) {
      const auto plan_back_itr = std::prev(plan_end);
      clipped_plan.emplace_back(
        plan_back_itr->stamp,
        plan_back_itr->position_vec + plan_back_itr->rotation_mat.getColumn(0) * plan_back_length,
        plan_back_itr->rotation_mat);
    }

    return clipped_plan;
  }

private:
  struct PlanPoint
  {
    TimePoint stamp;
    Vector position_vec;
    Matrix rotation_mat;
    double length;
  };

  template<class TfStampeds>
  static inline std::vector<PlanPoint> create_plan(const TfStampeds & point_tfs)
  {
    // get size
    const auto size = std::size(point_tfs);

    // check size
    if (size == 0) {
      return {};
    }

    // get itr
    std::vector<PlanPoint> plan;
    plan.reserve(size - 1);
    auto itr = std::begin(point_tfs);
    for (auto next_itr = std::next(itr), end = std::end(point_tfs); next_itr != end;
      itr = next_itr, next_itr = std::next(next_itr))
    {
      plan.push_back(
        {itr->stamp_, itr->getOrigin(), Matrix(itr->getRotation()),
          (next_itr->getOrigin() - itr->getOrigin()).length()});
    }

    return plan;
  }

  std::vector<PlanPoint> plan_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__OBSTACLE_STOPPER_HPP_
