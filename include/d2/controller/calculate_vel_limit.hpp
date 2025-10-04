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

#ifndef D2__CONTROLLER__CALCULATE_OBSTACLE_VEL_LIMIT_HPP_
#define D2__CONTROLLER__CALCULATE_OBSTACLE_VEL_LIMIT_HPP_

#include <limits>

#include "Eigen/Dense"

#include "d2/controller/stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace d2::controller
{

namespace
{

inline Eigen::Vector<double, 6> calculate_vel_limit(const Eigen::Vector<double, 6> & vel_raw, const Eigen::Vector<double, 6> & vel_limit)
{
  const auto vel_raw2 = vel_raw.array().square();
  const auto vel_limit2 = vel_limit.array().square();
  const auto rate_inv2_vec = vel_raw2.binaryExpr(vel_limit2, [](double raw2, double limit2) {
      return raw2 == 0.0 ? 0.0 : raw2 / limit2;
    });
  const auto rate_inv2_raw = rate_inv2_vec.sum();
  const auto rate = rate_inv2_raw <= 1.0 ? 1.0 : 1.0 / std::sqrt(rate_inv2_raw);
  return vel_raw * rate;
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_OBSTACLE_VEL_LIMIT_HPP_
