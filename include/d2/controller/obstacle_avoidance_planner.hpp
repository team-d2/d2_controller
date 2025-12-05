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

#ifndef D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH_HPP_
#define D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH_HPP_

#include <deque>
#include <limits>
#include <list>
#include <vector>

#include "Eigen/Dense"

namespace d2::controller
{

class ObstacleAvoidancePlanner
{
public:
  struct Parameters
  {
    double threshold_distance = 0.5;
  };

  struct PathPair
  {
    std::vector<Eigen::Vector2d> forward_avoidance_path;
    std::vector<Eigen::Vector2d> backward_avoidance_path;
  };

  ObstacleAvoidancePlanner(const Parameters & parameters);

  PathPair plan(
    const std::vector<Eigen::Vector2d> & base_plan,
    const std::vector<std::vector<Eigen::Vector2d>> & obstacles) const;
  
  std::vector<Eigen::Vector2d> plan_forward(
    const std::vector<Eigen::Vector2d> & base_plan,
    const std::vector<std::vector<Eigen::Vector2d>> & obstacles) const;

  std::vector<Eigen::Vector2d> plan_backward(
    const std::vector<Eigen::Vector2d> & base_plan,
    const std::vector<std::vector<Eigen::Vector2d>> & obstacles) const;

private:
  class Line
  {
  public:
    Line(Eigen::Vector2d p1, Eigen::Vector2d p2) : p1_(p1), direction_(p2 - p1) {}

    Eigen::Vector2d & p1() { return p1_; }
    const Eigen::Vector2d & p1() const { return p1_; }
    Eigen::Vector2d p2() const { return p1_ + direction_; }
    Eigen::Vector2d & direction() { return direction_; }
    const Eigen::Vector2d & direction() const { return direction_; }

  private:
    Eigen::Vector2d p1_;
    Eigen::Vector2d direction_;
  };

  struct CrossPoint
  {
    Eigen::Vector2d point;
    std::vector<Eigen::Vector2d>::const_iterator plan_iterator;
    std::vector<Line>::const_iterator obstacle_iterator;
  };

  std::vector<Eigen::Vector2d> plan_impl(
    const std::vector<Eigen::Vector2d> & base_plan,
    const std::vector<Eigen::Vector2d> & obstacles) const;

  std::vector<Eigen::Vector2d> plan_impl(
    const std::vector<Line> & base_plan,
    const std::vector<Line> & obstacles) const;
  
  static bool is_point_in_obstacle(
    const Eigen::Vector2d & point,
    const std::vector<Line> & obstacle);
  
  static CrossPoint point_obstacle_nearest_cross_point(
    const Eigen::Vector2d & point,
    const std::vector<Line> & obstacle);
  
  static std::vector<CrossPoint> cross_points(
    const std::vector<Line> & base_plan,
    const std::vector<Line> & obstacle);

  double threshold_distance_;
};

}  // namespace d2::controller

#endif  // D2__CONTROLLER__CALCULATE_OBSTACLE_AVOIDANCE_PATH_HPP_
