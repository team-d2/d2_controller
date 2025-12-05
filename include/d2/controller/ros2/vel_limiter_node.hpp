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

#ifndef D2__CONTROLLER__ROS2__VEL_LIMITER_NODE_HPP_
#define D2__CONTROLLER__ROS2__VEL_LIMITER_NODE_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <variant>

#include "Eigen/Core"
#include "rclcpp/node.hpp"
#include "d2/controller/calculate_vel_limit.hpp"
#include "d2/controller/ros2/adapter/accel.hpp"
#include "d2/controller/ros2/adapter/twist.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class VelLimiterNode : public rclcpp::Node
{
public:
  static constexpr auto kDefaultNodeName = "vel_limiter";

  D2__CONTROLLER__ROS2_PUBLIC
  inline VelLimiterNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    vel_limit_param_{
      this->declare_parameter<double>("vel_limit.linear.x", 1.0),
      this->declare_parameter<double>("vel_limit.linear.y", 1.0),
      this->declare_parameter<double>("vel_limit.linear.z", 1.0),
      this->declare_parameter<double>("vel_limit.angular.x", 1.0),
      this->declare_parameter<double>("vel_limit.angular.y", 1.0),
      this->declare_parameter<double>("vel_limit.angular.z", 1.0),
    },
    accel_limit_param_{
      this->declare_parameter<double>("accel_limit.linear.x", 1.0),
      this->declare_parameter<double>("accel_limit.linear.y", 1.0),
      this->declare_parameter<double>("accel_limit.linear.z", 1.0),
      this->declare_parameter<double>("accel_limit.angular.x", 1.0),
      this->declare_parameter<double>("accel_limit.angular.y", 1.0),
      this->declare_parameter<double>("accel_limit.angular.z", 1.0),
    },
    timeout_duration_(
      rclcpp::Duration::from_seconds(this->declare_parameter<double>("vel_timeout", 1.0))),
    rate_(this->declare_parameter<double>("vel_limiter.rate", 60.0)),
    accel_limit_(accel_limit_param_),
    current_vel_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    current_time_(this->now()),
    timeout_time_(this->now()),
    cmd_vel_pub_(this->create_cmd_vel_publisher()),
    target_vel_nav_sub_(this->create_target_vel_subscription()),
    vel_limit_vec_subs_(this->create_vel_limit_subscriptions(this->declare_parameter("vel_limit.topics", std::vector<std::string>{"vel_limit"}))),
    accel_limit_sub_(this->create_accel_limit_subscription()),
    publish_timer_(this->create_publish_timer())
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline VelLimiterNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : VelLimiterNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline VelLimiterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : VelLimiterNode(kDefaultNodeName, "", options)
  {
  }

private:
  inline rclcpp::Publisher<adapter::Twist>::SharedPtr create_cmd_vel_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<adapter::Twist>("cmd_vel", rclcpp::QoS(10), options);
  }

  inline rclcpp::Subscription<adapter::Accel>::SharedPtr create_accel_limit_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_subscription<adapter::Accel>(
      "accel_limit", rclcpp::QoS(10),
      [this](std::shared_ptr<const Eigen::Vector<double, 6>> accel_ptr) {this->set_accel_limit(std::move(accel_ptr));}, options);
  }

  inline std::vector<rclcpp::Subscription<adapter::Twist>::SharedPtr> create_vel_limit_subscriptions(const std::vector<std::string> & topic_names)
  {
    std::vector<rclcpp::Subscription<adapter::Twist>::SharedPtr> subs;
    vel_limit_ptrs_.reserve(topic_names.size());
    for (const auto & topic_name : topic_names) {
      const auto vel_limit_ptr_ptr = &vel_limit_ptrs_.emplace_back(std::make_shared<Eigen::Vector<double, 6>>(vel_limit_param_));
      subs.push_back(this->create_vel_limit_subscription(topic_name, vel_limit_ptr_ptr));
    }
    return subs;
  }

  inline rclcpp::Subscription<adapter::Twist>::SharedPtr create_vel_limit_subscription(const std::string & topic_name, std::shared_ptr<const Eigen::Vector<double, 6>> *vel_limit_ptr_ptr)
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_subscription<adapter::Twist>(
      topic_name, rclcpp::QoS(10),
      [vel_limit_ptr_ptr, topic_name](std::shared_ptr<const Eigen::Vector<double, 6>> vel_ptr) {
        // std::cout << "Received vel_limit from topic: " << topic_name << std::endl;
        *vel_limit_ptr_ptr = std::move(vel_ptr);
        // std::cout << "vel_limit: " << **vel_limit_ptr_ptr << std::endl;
      }, options);
  }

  inline rclcpp::Subscription<adapter::Twist>::SharedPtr create_target_vel_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    if (rate_ <= 0.0) {
      return this->create_subscription<adapter::Twist>(
        "target_vel", rclcpp::QoS(10).best_effort(),
        [this](std::shared_ptr<const Eigen::Vector<double, 6>> vel_ptr) {this->limit(std::move(vel_ptr));}, options);
    }
    else {
      return this->create_subscription<adapter::Twist>(
        "target_vel", rclcpp::QoS(10).best_effort(),
        [this](std::shared_ptr<const Eigen::Vector<double, 6>> vel_ptr) {this->set_target_vel(std::move(vel_ptr));}, options);
    }
  }

  inline rclcpp::TimerBase::SharedPtr create_publish_timer()
  {
    if (rate_ <= 0.0) {
      return nullptr;
    }
    return this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      [this]{this->limit(target_vel_ptr_);});
  }

  void limit(std::shared_ptr<const Eigen::Vector<double, 6>> target_vel_ptr)
  {
    // get time
    const auto now = this->now();
    const auto dt = (now - current_time_).seconds();

    // check timeout
    if (now >= timeout_time_ || target_vel_ptr->hasNaN()) {
      // update state
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Velocity command timeout. Stopping the robot.");
      current_vel_ = Eigen::Vector<double, 6>::Zero();
      current_time_ = now;
      return;
    }

    // limit vel
    auto vel_limit = vel_limit_param_;
    for (const auto & vl : vel_limit_ptrs_) {
      vel_limit = vel_limit.cwiseMin(*vl);
    }
    const auto vel_limited_vel = calculate_vel_limit(*target_vel_ptr, vel_limit);

    // limit accel
    const auto vel_delta = vel_limited_vel - current_vel_;
    const auto vel_delta_limit = dt == 0.0 ? Eigen::Vector<double, 6>::Zero().eval() : accel_limit_ * dt;
    const auto vel_delta_limited = calculate_vel_limit(vel_delta, vel_delta_limit);
    current_vel_ += vel_delta_limited;

    // publish cmd_vel
    auto cmd_vel_ptr = std::make_unique<Eigen::Vector<double, 6>>(current_vel_);
    cmd_vel_pub_->publish(std::move(cmd_vel_ptr));

    // update state
    current_time_ = now;
  }

  void set_target_vel(std::shared_ptr<const Eigen::Vector<double, 6>> target_vel_ptr)
  {
    const auto now = this->now();
    target_vel_ptr_ = std::move(target_vel_ptr);
    timeout_time_ = now + timeout_duration_;
  }

  void set_accel_limit(std::shared_ptr<const Eigen::Vector<double, 6>> accel_limit_ptr)
  {
    accel_limit_ = accel_limit_param_.cwiseMin(*accel_limit_ptr);
  }

  // parameter
  Eigen::Vector<double, 6> vel_limit_param_;
  Eigen::Vector<double, 6> accel_limit_param_;
  rclcpp::Duration timeout_duration_;
  double rate_;

  // state
  std::shared_ptr<const Eigen::Vector<double, 6>> target_vel_ptr_;
  std::vector<std::shared_ptr<const Eigen::Vector<double, 6>>> vel_limit_ptrs_;
  Eigen::Vector<double, 6> accel_limit_;
  Eigen::Vector<double, 6> current_vel_;
  rclcpp::Time current_time_;
  rclcpp::Time timeout_time_;

  // publisher
  rclcpp::Publisher<adapter::Twist>::SharedPtr cmd_vel_pub_;

  // subscription
  rclcpp::Subscription<adapter::Twist>::SharedPtr target_vel_nav_sub_;
  std::vector<rclcpp::Subscription<adapter::Twist>::SharedPtr> vel_limit_vec_subs_;
  rclcpp::Subscription<adapter::Accel>::SharedPtr accel_limit_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__CMD_VEL_LIMITER_NODE_HPP_
