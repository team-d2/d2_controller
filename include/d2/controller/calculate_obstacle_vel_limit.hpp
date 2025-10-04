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

struct ObstacleSpeedLimitParam
{
  double speed_per_distance;
  double offset_distance;
  double robot_width_harf;
  double min_turning_radius;
  double lidar_ignore_x_min, lidar_ignore_x_max, lidar_ignore_y_min, lidar_ignore_y_max;
  double z_min, z_max;
};

namespace
{

inline double calculate_obstacle_speed_limit(const pcl::PointCloud<pcl::PointXYZ> & lidar_points, const ObstacleSpeedLimitParam & param)
{
  // 物体までの距離を計算
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto & lidar_point : lidar_points) {
    {
      // 境界外とnanを除外
      const bool is_out_of_z_range = lidar_point.z < param.z_min || param.z_max < lidar_point.z;
      const bool is_too_close = param.lidar_ignore_x_min < lidar_point.x && lidar_point.x < param.lidar_ignore_x_max &&
        param.lidar_ignore_y_min < lidar_point.y && lidar_point.y < param.lidar_ignore_y_max;
      const bool is_nan = std::isnan(lidar_point.x) || std::isnan(lidar_point.y) || std::isnan(lidar_point.z);
      if (is_out_of_z_range || is_too_close || is_nan) {
        continue;
      }
    }

    // 計算のため，右半分の点は左側に移す
    const auto lidar_point_mirrored_vec = Eigen::Vector2d{lidar_point.x, std::abs(lidar_point.y)};

    // 最小旋回中心との距離で分岐
    const auto min_turning_centor_distance = (lidar_point_mirrored_vec - Eigen::Vector2d{0, param.min_turning_radius}).norm();

    // ぶつからない
    if (min_turning_centor_distance < param.min_turning_radius - param.robot_width_harf) {
      continue;
    }

    // 最小旋回時にぶつかる
    if (min_turning_centor_distance < param.min_turning_radius + param.robot_width_harf) {
      const auto angle = std::atan2(lidar_point_mirrored_vec[0], param.min_turning_radius - lidar_point_mirrored_vec[1]);
      const auto angle_normalized = angle < 0 ? angle + M_PI * 2 : angle;
      if (param.min_turning_radius < param.offset_distance) {
        return 0;
      }
      const auto angle_offset = std::asin(param.offset_distance / param.min_turning_radius);
      min_distance = std::min((angle_normalized - angle_offset) * param.min_turning_radius, min_distance);

      if (min_distance <= 0.0) {
        return 0.0;
      }
      continue;
    }

    // ロボットが旋回する最も外側がぶつかることを想定
    const auto lidar_opint_mirrored_from_robot_right = Eigen::Vector2d{lidar_point_mirrored_vec[0], lidar_point_mirrored_vec[1] + param.robot_width_harf};
    const auto angle = std::atan2(lidar_opint_mirrored_from_robot_right[1], lidar_opint_mirrored_from_robot_right[0]) * 2;
    const auto angle_normalized = angle < 0 ? angle + M_PI * 2 : angle;
    const auto turning_radius = 0.5 * lidar_opint_mirrored_from_robot_right.squaredNorm() / lidar_opint_mirrored_from_robot_right[1] - param.robot_width_harf;
    if (turning_radius < param.offset_distance) {
      return 0;
    }
    const auto angle_offset = std::asin(param.offset_distance / turning_radius);
    min_distance = std::min((angle_normalized - angle_offset) * turning_radius, min_distance);
  }

  // 速度を計算
  return std::max(param.speed_per_distance * min_distance, 0.0);
}

}  // namespace
}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_OBSTACLE_VEL_LIMIT_HPP_
