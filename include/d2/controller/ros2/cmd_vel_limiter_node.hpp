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

#ifndef D2__CONTROLLER__ROS2__CMD_VEL_LIMITER_NODE_HPP_
#define D2__CONTROLLER__ROS2__CMD_VEL_LIMITER_NODE_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <variant>

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class CmdVelLimiterNode : public rclcpp::Node
{
  using TwistMsg = geometry_msgs::msg::Twist;
  using AccelMsg = geometry_msgs::msg::Accel;

public:
  static constexpr auto kDefaultNodeName = "d2_cmd_vel_limiter";

  D2__CONTROLLER__ROS2_PUBLIC
  inline CmdVelLimiterNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    timeout_duration_(
      rclcpp::Duration::from_seconds(this->declare_parameter<double>("cmd_vel_timeout", 10.0))),
    vel_limit_linear_vec_(tf2::Vector3(
        this->declare_parameter<double>("vel_limit.linear.x", 1.0),
        this->declare_parameter<double>("vel_limit.linear.y", 1.0),
        this->declare_parameter<double>("vel_limit.linear.z", 1.0))),
    vel_limit_angular_vec_(tf2::Vector3(
        this->declare_parameter<double>("vel_limit.angular.x", 1.0),
        this->declare_parameter<double>("vel_limit.angular.y", 1.0),
        this->declare_parameter<double>("vel_limit.angular.z", 1.0))),
    accel_limit_linear_vec_(tf2::Vector3(
        this->declare_parameter<double>("accel_limit.linear.x", 1.0),
        this->declare_parameter<double>("accel_limit.linear.y", 1.0),
        this->declare_parameter<double>("accel_limit.linear.z", 1.0))),
    accel_limit_angular_vec_(tf2::Vector3(
        this->declare_parameter<double>("accel_limit.angular.x", 1.0),
        this->declare_parameter<double>("accel_limit.angular.y", 1.0),
        this->declare_parameter<double>("accel_limit.angular.z", 1.0))),
    last_vel_time_(this->now()),
    last_vel_linear_vec_(0.0, 0.0, 0.0),
    last_vel_angular_vec_(0.0, 0.0, 0.0),
    cmd_vel_pub_(this->create_publisher<TwistMsg>("cmd_vel", 10)),
    vel_limit_vec_sub_(this->create_subscription<TwistMsg>(
        "vel_limit", 10,
        [this](TwistMsg::ConstSharedPtr msg) {this->set_vel_limit(std::move(msg));})),
    accel_limit_sub_(
      this->create_subscription<AccelMsg>(
        "accel_limit", 10,
        [this](AccelMsg::ConstSharedPtr msg) {this->set_accel_limit(std::move(msg));}))
  {
    const auto rate = this->declare_parameter<double>("publish_rate", 60.0);
    if (rate <= 0.0) {
      cmd_vel_nav_sub_ = this->create_subscription<TwistMsg>(
        "cmd_vel_nav", 10,
        [this](TwistMsg::ConstSharedPtr msg) {this->limit_cmd_vel(std::move(msg));});
    } else {
      cmd_vel_nav_sub_ = this->create_subscription<TwistMsg>(
        "cmd_vel_nav", 10,
        [this](TwistMsg::ConstSharedPtr msg) {this->initialize_cmd_vel_nav(std::move(msg));});
    }
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline CmdVelLimiterNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CmdVelLimiterNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline CmdVelLimiterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CmdVelLimiterNode(kDefaultNodeName, "", options)
  {
  }

private:
  static std::tuple<tf2::Vector3, tf2::Vector3> limit_vel_vec(
    const std::tuple<tf2::Vector3, tf2::Vector3> & cmd_vel_nav_vec_tuple,
    const std::tuple<tf2::Vector3, tf2::Vector3> & vel_limit_vec_tuple)
  {
    const auto & [cmd_vel_nav_linear_vec, cmd_vel_nav_angular_vec] = cmd_vel_nav_vec_tuple;
    const auto & [vel_limit_linear_vec, vel_limit_angular_vec] = vel_limit_vec_tuple;
    const auto llx2 = vel_limit_linear_vec.x() * vel_limit_linear_vec.x();
    const auto lly2 = vel_limit_linear_vec.y() * vel_limit_linear_vec.y();
    const auto llz2 = vel_limit_linear_vec.z() * vel_limit_linear_vec.z();
    const auto lax2 = vel_limit_angular_vec.x() * vel_limit_angular_vec.x();
    const auto lay2 = vel_limit_angular_vec.y() * vel_limit_angular_vec.y();
    const auto laz2 = vel_limit_angular_vec.z() * vel_limit_angular_vec.z();
    const auto vnlx2 = cmd_vel_nav_linear_vec.x() * cmd_vel_nav_linear_vec.x();
    const auto vnly2 = cmd_vel_nav_linear_vec.y() * cmd_vel_nav_linear_vec.y();
    const auto vnlz2 = cmd_vel_nav_linear_vec.z() * cmd_vel_nav_linear_vec.z();
    const auto vnax2 = cmd_vel_nav_angular_vec.x() * cmd_vel_nav_angular_vec.x();
    const auto vnay2 = cmd_vel_nav_angular_vec.y() * cmd_vel_nav_angular_vec.y();
    const auto vnaz2 = cmd_vel_nav_angular_vec.z() * cmd_vel_nav_angular_vec.z();
    const auto lxrate2 = vnlx2 == 0 ? 0.0 : vnlx2 / llx2;
    const auto lyrate2 = vnly2 == 0 ? 0.0 : vnly2 / lly2;
    const auto lzrate2 = vnlz2 == 0 ? 0.0 : vnlz2 / llz2;
    const auto axrate2 = vnax2 == 0 ? 0.0 : vnax2 / lax2;
    const auto ayrate2 = vnay2 == 0 ? 0.0 : vnay2 / lay2;
    const auto azrate2 = vnaz2 == 0 ? 0.0 : vnaz2 / laz2;
    const auto rate2 = lxrate2 + lyrate2 + lzrate2 + axrate2 + ayrate2 + azrate2;
    const auto rate_inv_minimized = rate2 > 1.0 ? 1.0 / std::sqrt(rate2) : 1.0;

    return {
      cmd_vel_nav_linear_vec * rate_inv_minimized, cmd_vel_nav_angular_vec * rate_inv_minimized};
  }

  static std::tuple<tf2::Vector3, tf2::Vector3> limit_vel_vec(
    const std::tuple<tf2::Vector3, tf2::Vector3> & cmd_vel_nav_vec_tuple,
    const std::tuple<tf2::Vector3, tf2::Vector3> & vel_limit_vec_tuple,
    const std::tuple<tf2::Vector3, tf2::Vector3> & accel_limit_vec_tuple, const double duration,
    const std::tuple<tf2::Vector3, tf2::Vector3> & vel_last_vec_tuple)
  {
    // limit vel
    const auto [cmd_vel_vel_limited_linear_vec, cmd_vel_vel_angular_linear_vec] =
      limit_vel_vec(cmd_vel_nav_vec_tuple, vel_limit_vec_tuple);

    // limit accel
    const auto & [accel_limit_linear_vec, accel_limit_angular_vec] = accel_limit_vec_tuple;
    const auto & [vel_last_linear_vec, vel_last_angular_vec] = vel_last_vec_tuple;
    const auto vel_delta_limit_linear_vec = accel_limit_linear_vec * duration;
    const auto vel_delta_limit_angular_vec = accel_limit_angular_vec * duration;
    const auto vel_delta_vel_limited_linear_vec =
      cmd_vel_vel_limited_linear_vec - vel_last_linear_vec;
    const auto vel_delta_vel_limited_angular_vec =
      cmd_vel_vel_angular_linear_vec - vel_last_angular_vec;
    const auto [vel_delta_accel_limited_linear_vec, vel_delta_accel_limited_angular_vec] =
      limit_vel_vec(
      {vel_delta_vel_limited_linear_vec, vel_delta_vel_limited_angular_vec},
      {vel_delta_limit_linear_vec, vel_delta_limit_angular_vec});

    return {
      vel_last_linear_vec + vel_delta_accel_limited_linear_vec,
      vel_last_angular_vec + vel_delta_accel_limited_angular_vec};
  }

  void limit_cmd_vel(TwistMsg::ConstSharedPtr cmd_vel_nav_msg)
  {
    // get current time
    const auto now = this->now();
    const auto duration = (now - last_vel_time_).seconds();

    // convert cmd_vel_nav to vector
    tf2::Vector3 cmd_vel_nav_linear_vec, cmd_vel_nav_angular_vec;
    tf2::fromMsg(cmd_vel_nav_msg->linear, cmd_vel_nav_linear_vec);
    tf2::fromMsg(cmd_vel_nav_msg->angular, cmd_vel_nav_angular_vec);

    // limit velocity
    const auto [limited_linear_vec, limited_angular_vec] = limit_vel_vec(
      {cmd_vel_nav_linear_vec, cmd_vel_nav_angular_vec},
      {vel_limit_linear_vec_, vel_limit_angular_vec_},
      {accel_limit_linear_vec_, accel_limit_angular_vec_}, duration,
      {last_vel_linear_vec_, last_vel_angular_vec_});

    // publish limited cmd_vel
    auto cmd_vel_msg = std::make_unique<TwistMsg>();
    cmd_vel_msg->linear = tf2::toMsg(limited_linear_vec);
    cmd_vel_msg->angular = tf2::toMsg(limited_angular_vec);
    cmd_vel_pub_->publish(std::move(cmd_vel_msg));

    // update last velocity
    last_vel_time_ = now;
    last_vel_linear_vec_ = limited_linear_vec;
    last_vel_angular_vec_ = limited_angular_vec;
  }

  void limit_cmd_vel()
  {
    const auto now = this->now();
    if (now > timeout_time_) {
      publish_timer_.reset();
      cmd_vel_nav_sub_ = this->create_subscription<TwistMsg>(
        "cmd_vel_nav", 10,
        [this](TwistMsg::ConstSharedPtr msg) {this->initialize_cmd_vel_nav(std::move(msg));});
      return;
    }
    this->limit_cmd_vel(cmd_vel_nav_msg_);
  }

  void set_cmd_vel_nav(TwistMsg::ConstSharedPtr cmd_vel_nav_msg)
  {
    const auto now = this->now();
    cmd_vel_nav_msg_ = std::move(cmd_vel_nav_msg);
    timeout_time_ = now + timeout_duration_;
  }

  void set_vel_limit(TwistMsg::ConstSharedPtr vel_limit_msg)
  {
    tf2::fromMsg(vel_limit_msg->linear, vel_limit_linear_vec_);
    tf2::fromMsg(vel_limit_msg->angular, vel_limit_angular_vec_);
  }

  void set_accel_limit(AccelMsg::ConstSharedPtr accel_limit_msg)
  {
    tf2::fromMsg(accel_limit_msg->linear, accel_limit_linear_vec_);
    tf2::fromMsg(accel_limit_msg->angular, accel_limit_angular_vec_);
  }

  void initialize_cmd_vel_nav(TwistMsg::ConstSharedPtr cmd_vel_nav_msg)
  {
    this->set_cmd_vel_nav(std::move(cmd_vel_nav_msg));
    cmd_vel_nav_sub_ = this->create_subscription<TwistMsg>(
      "cmd_vel_nav", 10,
      [this](TwistMsg::ConstSharedPtr msg) {this->set_cmd_vel_nav(std::move(msg));});
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / this->get_parameter("publish_rate").as_double()),
      [this]() {this->limit_cmd_vel();});
  }

  // parameter
  rclcpp::Duration timeout_duration_;

  // msg
  TwistMsg::ConstSharedPtr cmd_vel_nav_msg_;
  rclcpp::Time timeout_time_;

  // limit
  tf2::Vector3 vel_limit_linear_vec_, vel_limit_angular_vec_;
  tf2::Vector3 accel_limit_linear_vec_, accel_limit_angular_vec_;

  // last cmd_vel
  rclcpp::Time last_vel_time_;
  tf2::Vector3 last_vel_linear_vec_, last_vel_angular_vec_;

  // publisher
  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_pub_;

  // subscription
  rclcpp::Subscription<TwistMsg>::SharedPtr cmd_vel_nav_sub_;
  rclcpp::Subscription<TwistMsg>::SharedPtr vel_limit_vec_sub_;
  rclcpp::Subscription<AccelMsg>::SharedPtr accel_limit_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__CMD_VEL_LIMITER_NODE_HPP_
