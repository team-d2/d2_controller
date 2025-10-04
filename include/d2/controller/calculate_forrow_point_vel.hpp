#ifndef D2__CONTROLLER__CALCULATE_FORROW_POINT_VEL_HPP_
#define D2__CONTROLLER__CALCULATE_FORROW_POINT_VEL_HPP_

#include "Eigen/Geometry"
#include "Eigen/Core"

namespace d2::controller
{

struct FollowPointParam
{
  double margin_distance;
  double vel_per_distance;
};

namespace
{

inline Eigen::Vector<double, 6> calculate_forrow_point_vel(
  const Eigen::Vector3d & target_point,
  const Eigen::Isometry3d & pose,
  const FollowPointParam & param)
{
  //
  const auto direction_x = pose.linear().block<3, 1>(0, 0);
  const auto direction_y = pose.linear().block<3, 1>(0, 1);
  const auto direction_z = pose.linear().block<3, 1>(0, 2);

  const auto error_vec = target_point - pose.translation();
  const auto rot_vec = direction_x.cross(error_vec);
  const auto rot_vec_length = rot_vec.norm();
  if (rot_vec_length == 0) {
    // 直進の場合
    const auto error_distance = error_vec.norm();
    const auto lookahead_distance = std::max(0.0, error_distance - param.margin_distance);
    const auto linear_speed = lookahead_distance * param.vel_per_distance;
    return {linear_speed, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  const auto rot_vec_length_inv = 1.0 / rot_vec_length;
  const auto dot = direction_x.dot(error_vec);
  const auto error_angle = std::atan2(rot_vec_length, dot) * 2.0;
  const auto error_radius = error_vec.squaredNorm() * rot_vec_length_inv * 0.5;
  const auto error_distance = error_radius * error_angle;
  if (error_distance <= param.margin_distance) {
    // 近すぎる場合
    return Eigen::Vector<double, 6>::Zero();
  }
  const auto lookahead_distance = error_distance - param.margin_distance;
  const auto margine_angle = param.margin_distance / error_radius;
  const auto lookahead_angle = error_angle - margine_angle;
  const auto linear_speed = lookahead_distance * param.vel_per_distance;
  const auto angular_speed = lookahead_angle * param.vel_per_distance;
  return {
    linear_speed,
    0.0,
    0.0,
    0.0,
    angular_speed * rot_vec_length_inv * direction_y.dot(rot_vec),
    angular_speed * rot_vec_length_inv * direction_z.dot(rot_vec),};
}

}

}

#endif  // D2__CONTROLLER__CALCULATE_FORROW_POINT_VEL_HPP_