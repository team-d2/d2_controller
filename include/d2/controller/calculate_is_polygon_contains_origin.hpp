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

#ifndef D2__CONTROLLER__CALCULATE_IS_POLYGON_CONTAINS_ORIGIN_HPP_
#define D2__CONTROLLER__CALCULATE_IS_POLYGON_CONTAINS_ORIGIN_HPP_

#include <limits>

#include "Eigen/Dense"

#include "d2/controller/stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace d2::controller
{

namespace
{

bool calculate_is_polygon_contains_origin(const std::vector<Eigen::Vector2d>& poly) {
    int n = poly.size();
    if (n < 3) return false;

    bool inside = false;

    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Eigen::Vector2d& pi = poly[i];
        const Eigen::Vector2d& pj = poly[j];

        // y座標が原点をまたいでいるかチェック
        bool intersect =
            ((pi.y() > 0) != (pj.y() > 0)) &&
            (0 < (pj.x() - pi.x()) * (-pi.y()) / (pj.y() - pi.y()) + pi.x());

        if (intersect)
            inside = !inside;
    }

    return inside;
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_IS_POLYGON_CONTAINS_ORIGIN_HPP_