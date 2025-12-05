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

#ifndef D2__CONTROLLER__CALCULATE_LOOKAHEAD_POINT_HPP_
#define D2__CONTROLLER__CALCULATE_LOOKAHEAD_POINT_HPP_

#include "Eigen/Geometry"

#include "d2/controller/stamped.hpp"

namespace d2::controller
{
namespace
{

inline Stamped<Eigen::Vector3d> calculate_lookahead_point(
  const std::vector<Stamped<Eigen::Isometry3d>> & stamped_points, double lookahead_distance)
{
  // create iterators
  const auto begin = std::begin(stamped_points);
  const auto end = std::end(stamped_points);

  // check empty
  if (begin == end || begin->data.translation().hasNaN()) {
    constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
    return Stamped<Eigen::Vector3d>{
      0,
      Eigen::Vector3d(kNan, kNan, kNan),
    };
  }

  // search lookahead point
  const auto lookahead_distance_sq = lookahead_distance * lookahead_distance;
  auto itr_relative = Eigen::Vector3d::Zero().eval();
  auto itr_relative_sq = 0.0;
  auto itr = begin;
  auto next_itr = std::next(itr);
  while (next_itr != end) {
    if (next_itr->data.translation().hasNaN()) {
      continue;
    }

    auto next_itr_relative = (next_itr->data.translation() - begin->data.translation()).eval();
    auto next_itr_relative_sq = next_itr_relative.squaredNorm();

    if (next_itr_relative_sq < lookahead_distance_sq) {
      itr_relative = next_itr_relative;
      itr_relative_sq = next_itr_relative_sq;
      itr = next_itr;
      next_itr = std::next(next_itr);
      continue;
    }

    auto line = (next_itr->data.translation() - itr->data.translation()).eval();
    auto line_sq = line.squaredNorm();
    auto dot = itr_relative.dot(line);
    auto line_rate = (-dot + std::sqrt(dot * dot - line_sq * (itr_relative_sq - lookahead_distance_sq))) / line_sq;
    auto dt = next_itr->stamp_nanosec - itr->stamp_nanosec;
    return Stamped<Eigen::Vector3d>
    {
      itr->stamp_nanosec + static_cast<std::uint64_t>(dt * line_rate),
      itr->data.translation() + line_rate * line,
    };
  }

  return Stamped<Eigen::Vector3d>
  {
    itr->stamp_nanosec,
    itr->data.translation(),
  };
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_LOOKAHEAD_POINT_HPP_
