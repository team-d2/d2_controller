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

#ifndef D2__CONTROLLER__ROS2__OBSTACLE_VEL_LIMITTER_NODE_NODE_HPP_
#define D2__CONTROLLER__ROS2__OBSTACLE_VEL_LIMITTER_NODE_NODE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/node.hpp"
#include "d2/controller/ros2/adapter/point_cloud2.hpp"
#include "d2/controller/ros2/adapter/twist.hpp"
#include "d2/controller/calculate_obstacle_vel_limit.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class ObstaceleVelLimitterNode : public rclcpp::Node
{
public:
  static constexpr auto kDefaultNodeName = "obstacle_vel_limitter";

  D2__CONTROLLER__ROS2_PUBLIC
  inline ObstaceleVelLimitterNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(this->declare_parameter("frame_id", "map")),
    limitter_param_{
      this->declare_parameter<double>("obstacle_speed_limitter.speed_per_distance", 1.0),
      this->declare_parameter<double>("obstacle_speed_limitter.offset_distance", 0.8),
      this->declare_parameter<double>("robot.width", 0.7) * 0.5,
      this->declare_parameter<double>("robot.min_turning_radius", 1.0),
      this->declare_parameter<double>("lidar.ignore_x_min", -0.65),
      this->declare_parameter<double>("lidar.ignore_x_max", +0.65),
      this->declare_parameter<double>("lidar.ignore_y_min", -0.42),
      this->declare_parameter<double>("lidar.ignore_y_max", +0.42),
      this->declare_parameter<double>("lidar.obstacle.z_min", -1.0),
      this->declare_parameter<double>("lidar.obstacle.z_max", 1.0),
    },
    vel_limit_pub_(this->create_vel_limit_publisher()),
    lidar_points_sub_(this->create_lidar_points_subscription())
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline ObstaceleVelLimitterNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ObstaceleVelLimitterNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline ObstaceleVelLimitterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ObstaceleVelLimitterNode(kDefaultNodeName, "", options)
  {
  }

  ~ObstaceleVelLimitterNode() override {}

private:
  inline rclcpp::Publisher<adapter::Twist>::SharedPtr create_vel_limit_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<adapter::Twist>("obstacle_vel_limit", rclcpp::QoS(10).best_effort(), options);
  }

  inline rclcpp::Subscription<adapter::PointCloud2>::SharedPtr create_lidar_points_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    return this->create_subscription<adapter::PointCloud2>(
      "lidar/points", rclcpp::QoS(10).best_effort(),
      [this](std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> pc) {this->limit(std::move(pc));}, options);
  }

  void limit(std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> lidar_points)
  {
    const auto speed_limit = calculate_obstacle_speed_limit(*lidar_points, limitter_param_);
    const auto angular_limit = speed_limit / limitter_param_.min_turning_radius;
    auto vel_limit = std::make_unique<Eigen::Vector<double, 6>>(Eigen::Vector<double, 6>{
      speed_limit, speed_limit, speed_limit, angular_limit, angular_limit, angular_limit});
    vel_limit_pub_->publish(std::move(vel_limit));
  }

  // parameters
  std::string frame_id_;
  ObstacleSpeedLimitParam limitter_param_;

  // vel_limit publisher
  rclcpp::Publisher<adapter::Twist>::SharedPtr vel_limit_pub_;

  // plan & lidar_points subscription
  rclcpp::Subscription<adapter::PointCloud2>::SharedPtr lidar_points_sub_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__OBSTACLE_VEL_LIMITTER_NODE_NODE_HPP_
