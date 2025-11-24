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
#include "d2/controller/calculate_obstacle_avoidance_path.hpp"
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
    auto avoidanced_path = calculate_obstacle_avoidance_path(original_plan_, obstacles_);
    auto avoidanced_path_msg = std::make_unique<PathMsg>();
    avoidanced_path_msg->header.frame_id = frame_id_;
    avoidanced_path_msg->header.stamp = stamp;
    avoidanced_path_msg->poses.reserve(avoidanced_path.size());
    for (const auto & point : avoidanced_path) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose.position.x = point.x();
      pose_stamped.pose.position.y = point.y();
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

    original_plan_.clear();
    for (const auto & pose_stamped : plan_msg->poses) {
      original_plan_.emplace_back(
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y);
    }

    if (original_plan_.empty() || obstacles_.empty()) {
      return;
    }

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
      rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<PathMsg>("avoidanced_plan", rclcpp::QoS(10).best_effort(), options);
  }

  inline rclcpp::Subscription<PathMsg>::SharedPtr create_plan_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
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
      rclcpp::QosPolicyKind::Reliability};
    return this->create_subscription<ObstacleArrayMsg>(
      "obstacles", rclcpp::QoS(10).best_effort(),
      [this](ObstacleArrayMsg::ConstSharedPtr msg) {this->update_obstacles(std::move(msg));}, options);
  }

  // parameters
  std::string frame_id_;

  std::vector<std::vector<Eigen::Vector2d>> obstacles_;
  std::deque<Eigen::Vector2d> original_plan_;

  // vel_limit publisher
  rclcpp::Publisher<PathMsg>::SharedPtr avoidanced_plan_publisher_;

  // plan & lidar_points subscription
  rclcpp::Subscription<PathMsg>::SharedPtr plan_subscription_;
  rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr obstacles_subscription_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__OBSTACLE_AVOIDANCE_PLANNER_NODE_HPP_
