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

#ifndef D2__CONTROLLER__ROS2__OBSTACLE_AVOIDANCE_PLANNER_NODE_HPP_
#define D2__CONTROLLER__ROS2__OBSTACLE_AVOIDANCE_PLANNER_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <type_traits>
#include <vector>

#include "rclcpp/node.hpp"
#include "d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp"
#include "nav_msgs/msg/path.hpp"
#include "d2/controller/calculate_obstacle_avoidance_path3.hpp"
#include "d2/controller/smooth_path.hpp"
#include "d2/controller/remove_path_self_intersection.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class ObstacleAvoidancePlannerNode : public rclcpp::Node
{
public:
  using ObstacleArrayMsg = d2_costmap_converter_msgs::msg::ObstacleArrayMsg;
  using PathMsg = nav_msgs::msg::Path;

  static constexpr auto kDefaultNodeName = "obstacle_vel_limitter";

  D2__CONTROLLER__ROS2_PUBLIC
  inline ObstacleAvoidancePlannerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(this->declare_parameter("frame_id", "map")),
    path_smoothing_count_(this->declare_parameter("path_smoothing_count", 2)),
    path_smoothing_distance_(this->declare_parameter("path_smoothing_distance", 0.5)),
    threshold_distance_(this->declare_parameter("threshold_distance", 0.2)),
    was_avoidance_direction_forward_(true),
    avoidanced_plan_publisher_(this->create_avoidanced_plan_publisher()),
    plan_subscription_(this->create_plan_subscription()),
    obstacles_subscription_(this->create_obstacles_subscription())
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline ObstacleAvoidancePlannerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ObstacleAvoidancePlannerNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline ObstacleAvoidancePlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ObstacleAvoidancePlannerNode(kDefaultNodeName, "", options)
  {
  }

  ~ObstacleAvoidancePlannerNode() override {}

private:

  void publish_avoidanced_plan(const builtin_interfaces::msg::Time & stamp)
  {
    ObstacleAvoidancePathParams param;
    param.penetration_limit = threshold_distance_;
    param.is_avoidance_direction_forward = true;
    auto f_avoidanced_path = calculate_obstacle_avoidance_path(original_plan_, obstacles_, param);

    for (int i = 0; i < path_smoothing_count_; ++i) {
      f_avoidanced_path = smooth_path(f_avoidanced_path, path_smoothing_distance_);
      f_avoidanced_path = remove_path_self_intersection(f_avoidanced_path);
      f_avoidanced_path = calculate_obstacle_avoidance_path(f_avoidanced_path, obstacles_, param);
    }


    param.is_avoidance_direction_forward = false;
    auto b_avoidanced_path = calculate_obstacle_avoidance_path(original_plan_, obstacles_, param);

    for (int i = 0; i < path_smoothing_count_; ++i) {
      b_avoidanced_path = smooth_path(b_avoidanced_path, path_smoothing_distance_);
      b_avoidanced_path = remove_path_self_intersection(b_avoidanced_path);
      b_avoidanced_path = calculate_obstacle_avoidance_path(b_avoidanced_path, obstacles_, param);
    }

    auto f_avoidanced_path_length = 0.0;
    if (f_avoidanced_path.empty()) {
      f_avoidanced_path_length = std::numeric_limits<double>::infinity();
    }
    else {
      for (auto itr = std::next(f_avoidanced_path.begin()); itr != f_avoidanced_path.end(); ++itr) {
        f_avoidanced_path_length += (*(itr) - *(std::prev(itr))).norm();
      }
      f_avoidanced_path_length += (original_plan_.back() - f_avoidanced_path.back()).norm() * 3.0;
      if (!was_avoidance_direction_forward_) {
        f_avoidanced_path_length += 10.0;
      }
    }

    auto b_avoidanced_path_length = 0.0;
    if (b_avoidanced_path.empty()) {
      b_avoidanced_path_length = std::numeric_limits<double>::infinity();
    }
    else {
      for (auto itr = std::next(b_avoidanced_path.begin()); itr != b_avoidanced_path.end(); ++itr) {
        b_avoidanced_path_length += (*(itr) - *(std::prev(itr))).norm();
      }
      b_avoidanced_path_length += (original_plan_.back() - b_avoidanced_path.back()).norm() * 3.0;
      if (was_avoidance_direction_forward_) {
        b_avoidanced_path_length += 10.0;
      }
    }

    const auto & avoidanced_path = f_avoidanced_path_length < b_avoidanced_path_length ? f_avoidanced_path : b_avoidanced_path;
    was_avoidance_direction_forward_ = f_avoidanced_path_length < b_avoidanced_path_length;

    if (avoidanced_path.empty()) {
      return;
    }

    auto avoidanced_path_msg = std::make_unique<PathMsg>();
    avoidanced_path_msg->header.frame_id = frame_id_;
    avoidanced_path_msg->header.stamp = stamp;
    avoidanced_path_msg->poses.reserve(avoidanced_path.size());
    for (const auto & point : avoidanced_path) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose.position.x = point.x();
      pose_stamped.pose.position.y = point.y();
      avoidanced_path_msg->poses.push_back(std::move(pose_stamped));
    }

    if (!avoidanced_path_msg->poses.empty()) {
      avoidanced_path_msg->poses.front().pose.orientation.w = initial_orientation_.w();
      avoidanced_path_msg->poses.front().pose.orientation.x = initial_orientation_.x();
      avoidanced_path_msg->poses.front().pose.orientation.y = initial_orientation_.y();
      avoidanced_path_msg->poses.front().pose.orientation.z = initial_orientation_.z();
    }

    avoidanced_plan_publisher_->publish(std::move(avoidanced_path_msg));
  }

  void update_path(PathMsg::ConstSharedPtr plan_msg)
  {
    if (plan_msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(
        this->get_logger(), "frame_id mismatch: plan frame_id = %s, node frame_id = %s",
        plan_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    last_plan_time_ = rclcpp::Time(plan_msg->header.stamp);

    original_plan_.clear();
    for (const auto & pose_stamped : plan_msg->poses) {
      original_plan_.emplace_back(
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y);
    }

    if (original_plan_.empty()) {
      return;
    }

    initial_orientation_.w() = plan_msg->poses.front().pose.orientation.w;
    initial_orientation_.x() = plan_msg->poses.front().pose.orientation.x;
    initial_orientation_.y() = plan_msg->poses.front().pose.orientation.y;
    initial_orientation_.z() = plan_msg->poses.front().pose.orientation.z;

    this->publish_avoidanced_plan(plan_msg->header.stamp);
  }

  void update_obstacles(ObstacleArrayMsg::ConstSharedPtr obstacles_msg)
  {
    if (obstacles_msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(
        this->get_logger(), "frame_id mismatch: plan frame_id = %s, node frame_id = %s",
        obstacles_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    obstacles_.clear();
    obstacles_.reserve(obstacles_msg->obstacles.size());
    for (const auto & obstacle_msg : obstacles_msg->obstacles) {
      if (obstacle_msg.header.frame_id != frame_id_ && obstacle_msg.header.frame_id != "") {
        RCLCPP_WARN(
          this->get_logger(), "frame_id mismatch: obstacle frame_id = %s, node frame_id = %s",
          obstacle_msg.header.frame_id.c_str(), frame_id_.c_str());
        continue;
      }
      std::vector<Eigen::Vector2d> obstacle;
      obstacle.reserve(obstacle_msg.polygon.points.size());
      for (const auto & point : obstacle_msg.polygon.points) {
        obstacle.emplace_back(point.x, point.y);
      }
      obstacles_.emplace_back(std::move(obstacle));
    }

    if (original_plan_.empty()) {
      return;
    }

    this->publish_avoidanced_plan(obstacles_msg->header.stamp);
  }

  inline rclcpp::Publisher<PathMsg>::SharedPtr create_avoidanced_plan_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability};
    return this->create_publisher<PathMsg>("avoidanced_plan", rclcpp::QoS(10).best_effort(), options);
  }

  inline rclcpp::Subscription<PathMsg>::SharedPtr create_plan_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability};
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_subscription<PathMsg>(
      "plan", rclcpp::QoS(10).best_effort(),
      [this](PathMsg::ConstSharedPtr msg) {this->update_path(std::move(msg));}, options);
  }

  inline rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr create_obstacles_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability};
    return this->create_subscription<ObstacleArrayMsg>(
      "obstacles", rclcpp::QoS(10).best_effort(),
      [this](ObstacleArrayMsg::ConstSharedPtr msg) {this->update_obstacles(std::move(msg));}, options);
  }

  // parameters
  std::string frame_id_;
  int path_smoothing_count_;
  double path_smoothing_distance_;
  double threshold_distance_;
  bool was_avoidance_direction_forward_;

  std::vector<std::vector<Eigen::Vector2d>> obstacles_;
  std::deque<Eigen::Vector2d> original_plan_;
  Eigen::Quaterniond initial_orientation_;
  rclcpp::Time last_plan_time_;

  // vel_limit publisher
  rclcpp::Publisher<PathMsg>::SharedPtr avoidanced_plan_publisher_;

  // plan & lidar_points subscription
  rclcpp::Subscription<PathMsg>::SharedPtr plan_subscription_;
  rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr obstacles_subscription_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__OBSTACLE_AVOIDANCE_PLANNER_NODE_HPP_
