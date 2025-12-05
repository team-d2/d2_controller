#ifndef D2__CONTROLLER__CALCULATE_CIRCLE_LINE_CROSS_POINT_HPP_
#define D2__CONTROLLER__CALCULATE_CIRCLE_LINE_CROSS_POINT_HPP_

#include <iostream>

#include "Eigen/Geometry"
#include "Eigen/Core"

namespace d2::controller
{

namespace
{

inline Eigen::Vector2d calculate_circle_line_cross_point_from_inner_outer_points(
  double circle_radius,
  const Eigen::Vector2d & inner_point,
  const Eigen::Vector2d & outer_point)
{
    auto line = (outer_point - inner_point).eval();
    auto line_sq = line.squaredNorm();
    auto dot = inner_point.dot(line);
    auto line_rate = (-dot + std::sqrt(dot * dot - line_sq * (inner_point.squaredNorm() - circle_radius * circle_radius))) / line_sq;
    return inner_point + line_rate * line;
}

}

}

#endif  // D2__CONTROLLER__CALCULATE_CIRCLE_LINE_CROSS_POINT_HPP_