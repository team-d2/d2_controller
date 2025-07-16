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

#ifndef D2__CONTROLLER__ROS2__CMD_VEL_TRANSFORMER_NODE_HPP_
#define D2__CONTROLLER__ROS2__CMD_VEL_TRANSFORMER_NODE_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <variant>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/convert.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class CmdVelTransformerNode : public rclcpp::Node
{
  using TwistMsg = geometry_msgs::msg::Twist;
  using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
  using TfMsg = geometry_msgs::msg::TransformStamped;

public:
  static constexpr auto kDefaultNodeName = "d2_cmd_vel_transformer";

  D2__CONTROLLER__ROS2_PUBLIC
  inline CmdVelTransformerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    base_footprint_frame_id_(
      this->declare_parameter<std::string>("base_footprint_frame_id", "base_footprint")),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this),
    cmd_vel_pub_(this->create_publisher<TwistMsg>("cmd_vel", 10)),
    cmd_vel_stamped_sub_(this->create_subscription<TwistStampedMsg>(
        "cmd_vel/stamped", 10,
        [this](TwistStampedMsg::ConstSharedPtr msg) {this->transform(std::move(msg));}))
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline CmdVelTransformerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CmdVelTransformerNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline CmdVelTransformerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CmdVelTransformerNode(kDefaultNodeName, "", options)
  {
  }

  ~CmdVelTransformerNode() override {}

private:
  void transform(TwistStampedMsg::ConstSharedPtr cmd_vel_stamped_msg)
  {
    // get base_footprint quaternion
    TfMsg tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
        cmd_vel_stamped_msg->header.frame_id, base_footprint_frame_id_, rclcpp::Time(0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
      return;
    }
    tf2::Quaternion quat;
    tf2::fromMsg(tf_msg.transform.rotation, quat);
    auto mat = tf2::Matrix3x3(quat);

    // get linear and angular velocity vectors
    tf2::Vector3 vel_linear_vec;
    tf2::fromMsg(cmd_vel_stamped_msg->twist.linear, vel_linear_vec);
    tf2::Vector3 vel_angular_vec;
    tf2::fromMsg(cmd_vel_stamped_msg->twist.angular, vel_angular_vec);

    // publish transformed cmd_vel
    auto cmd_vel_msg = std::make_unique<TwistMsg>();
    cmd_vel_msg->linear = tf2::toMsg(mat * vel_linear_vec);
    cmd_vel_msg->angular = tf2::toMsg(mat * vel_angular_vec);
    cmd_vel_pub_->publish(std::move(cmd_vel_msg));
  }

  // parameters
  std::string base_footprint_frame_id_;

  // tf2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // publisher
  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_pub_;

  // subscription
  rclcpp::Subscription<TwistStampedMsg>::SharedPtr cmd_vel_stamped_sub_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__CMD_VEL_TRANSFORMER_NODE_HPP_
