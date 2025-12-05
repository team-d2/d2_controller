#include "d2/controller/obstacle_avoidance_planner.hpp"

namespace d2::controller
{

ObstacleAvoidancePlanner::ObstacleAvoidancePlanner(const Parameters & parameters)
: threshold_distance_(parameters.threshold_distance)
{
}

ObstacleAvoidancePlanner::PathPair ObstacleAvoidancePlanner::plan(
  const std::vector<Eigen::Vector2d> & base_plan,
  const std::vector<std::vector<Eigen::Vector2d>> & obstacles) const
{
  return PathPair{
    this->plan_forward(base_plan, obstacles),
    this->plan_backward(base_plan, obstacles)
  };
}

std::vector<Eigen::Vector2d> ObstacleAvoidancePlanner::plan_forward(
  const std::vector<Eigen::Vector2d> & base_plan,
  const std::vector<std::vector<Eigen::Vector2d>> & obstacles) const
{
  std::vector<Eigen::Vector2d> avoidance_path;

  return avoidance_path;
}

std::vector<Eigen::Vector2d> ObstacleAvoidancePlanner::plan_backward(
  const std::vector<Eigen::Vector2d> & base_plan,
  const std::vector<std::vector<Eigen::Vector2d>> & obstacles) const
{
  std::vector<Eigen::Vector2d> avoidance_path;

  return avoidance_path;
}

std::vector<Eigen::Vector2d> ObstacleAvoidancePlanner::plan_impl(
  const std::vector<Line> & base_plan,
  const std::vector<Line> & obstacle) const
{
  if (base_plan.empty()) {
    return {};
  }

  bool is_initial_point_in_obstacle = is_point_in_obstacle(base_plan.front().p1(), obstacle);
  auto cross_points = this->cross_points(base_plan, obstacle);
  if (is_initial_point_in_obstacle) {
    auto init_cross_point = point_obstacle_nearest_cross_point(base_plan.front().p1(), obstacle);
    if ((init_cross_point.point - base_plan.front().p1()).norm() > threshold_distance_) {
      return {};
    }
    cross_points.push_back(init_cross_point);
  }

  



  return avoidance_path;

}  // namespace d2::controller
