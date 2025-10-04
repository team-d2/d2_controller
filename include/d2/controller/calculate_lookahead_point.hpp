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
  if (begin == end) {
    constexpr auto kNan = std::numeric_limits<double>::quiet_NaN();
    return Stamped<Eigen::Vector3d>{
      0,
      Eigen::Vector3d(kNan, kNan, kNan),
    };
  }

  // search lookahead point
  auto itr = begin;
  auto itr_next = std::next(itr);
  while (itr_next != end) {
    // check itr_next nan
    if (itr_next->data.translation().array().isNaN().any() || itr_next->data.linear().array().isNaN().any()) {
      break;
    }

    const Eigen::Vector3d vec = itr_next->data.translation() - itr->data.translation();
    const auto section_distance = vec.norm();

    if (section_distance >= lookahead_distance) {
      const auto rate = lookahead_distance / section_distance;
      return Stamped<Eigen::Vector3d>
      {
        itr->stamp_nanosec + static_cast<std::uint64_t>(rate * (itr_next->stamp_nanosec - itr->stamp_nanosec)),
        itr->data.translation() + rate * vec,
      };
    }

    // decrement lookahead_distance
    lookahead_distance -= section_distance;

    // increment itr
    itr = itr_next;
    itr_next = std::next(itr);
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
