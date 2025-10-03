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

#ifndef D2__CONTROLLER__ROS2__PURE_PURSUIT_NODE_HPP_
#define D2__CONTROLLER__ROS2__PURE_PURSUIT_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "d2/controller/calculate_forrow_point_vel.hpp"
#include "d2/controller/calculate_lookahead_point.hpp"
#include "d2/controller/ros2/adapter/path.hpp"
#include "d2/controller/ros2/adapter/point_stamped.hpp"
#include "d2/controller/ros2/adapter/twist.hpp"
#include "rclcpp/node.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class PurePursuitNode : public rclcpp::Node
{
  using Path = adapter::Path::custom_type;
  using PointStamped = adapter::PointStamped::custom_type;
  using Twist = adapter::Twist::custom_type;

public:
  static constexpr auto kDefaultNodeName = "pure_pursuit";

  D2__CONTROLLER__ROS2_PUBLIC
  inline PurePursuitNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    lookahead_distance_(this->declare_parameter<double>("purepursuit.lookahead_distance", 1.0)),
    follow_point_param_{
      this->declare_parameter<double>("follow_point.margin_distance", 1.0),
      this->declare_parameter<double>("follow_point.vel_per_distance", 1.0)},
    cmd_vel_pub_(this->create_cmd_vel_publisher()),
    target_point_pub_(this->create_target_point_publisher()),
    local_plan_sub_(this->create_local_plan_subscription())
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline PurePursuitNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PurePursuitNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline PurePursuitNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PurePursuitNode(kDefaultNodeName, "", options)
  {
  }

  ~PurePursuitNode() override {}

private:
  inline rclcpp::Publisher<adapter::Twist>::SharedPtr create_cmd_vel_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<adapter::Twist>("cmd_vel", rclcpp::QoS(10).best_effort(), options);
  }

  inline rclcpp::Publisher<adapter::PointStamped>::SharedPtr create_target_point_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<adapter::PointStamped>("target_point", rclcpp::QoS(10).best_effort(), options);
  }
  
  inline rclcpp::Subscription<adapter::Path>::SharedPtr create_local_plan_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_subscription<adapter::Path>(
      "local_plan", rclcpp::QoS(10).transient_local(),
      [this](std::shared_ptr<const Path> path) {this->calculate(std::move(path));}, options);
  }

  void calculate(std::shared_ptr<const Path> path)
  {
    // calcurate the target point
    auto point_stamped = std::make_unique<PointStamped>();
    point_stamped->frame_id = path->frame_id;
    point_stamped->data = calculate_lookahead_point(path->data.data, lookahead_distance_);

    // calcurate cmd_vel
    auto cmd_vel = std::make_unique<Twist>();
    *cmd_vel = calculate_forrow_point_vel(
      point_stamped->data.data, path->data.data.front().data,
      follow_point_param_);
    
    // publish cmd_vel
    cmd_vel_pub_->publish(std::move(cmd_vel));
    
    // publish target point
    target_point_pub_->publish(std::move(point_stamped));
  }

  // parameter
  double lookahead_distance_;
  FollowPointParam follow_point_param_;

  // publisher
  rclcpp::Publisher<adapter::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<adapter::PointStamped>::SharedPtr target_point_pub_;

  // subscriptions
  rclcpp::Subscription<adapter::Path>::SharedPtr local_plan_sub_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__PURE_PURSUIT_NODE_HPP_
