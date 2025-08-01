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

#ifndef D2__CONTROLLER__ROS2__TARGET_POINT_FOLLOWER_NODE_HPP_
#define D2__CONTROLLER__ROS2__TARGET_POINT_FOLLOWER_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <variant>

#include "d2/controller/stamped.hpp"
#include "d2/controller/vector3.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/convert.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class TargetPointFollowerNode : public rclcpp::Node
{
  using PointMsg = geometry_msgs::msg::PointStamped;
  using PoseMsg = geometry_msgs::msg::PoseStamped;
  using TwistMsg = geometry_msgs::msg::Twist;
  using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
  using TfMsg = geometry_msgs::msg::TransformStamped;

public:
  static constexpr auto kDefaultNodeName = "d2_target_point_follower";

  D2__CONTROLLER__ROS2_PUBLIC
  inline TargetPointFollowerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(),
    pose_frame_id_(this->declare_parameter<std::string>("pose_frame_id", "base_link")),
    cmd_vel_distance_rate_(this->declare_parameter<double>("cmd_vel_distance_rate", 0.2)),
    position_vec_(),
    direction_x_vec_(),
    direction_y_vec_(),
    direction_z_vec_(),
    target_point_vec_(),
    target_point_received_(false),
    cmd_vel_pub_(this->create_publisher<TwistMsg>("cmd_vel", 10)),
    cmd_vel_stamped_pub_(
      !pose_frame_id_.empty() ? this->create_publisher<TwistStampedMsg>("cmd_vel/stamped", 10) :
      nullptr),
    pose_sub_(this->create_subscription<PoseMsg>(
        "pose", 10, [this](PoseMsg::ConstSharedPtr msg) {this->update_pose(std::move(msg));})),
    target_point_sub_(
      this->create_subscription<PointMsg>(
        "target_point", 10,
        [this](PointMsg::ConstSharedPtr msg) {this->update_target_point(std::move(msg));}))
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline TargetPointFollowerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : TargetPointFollowerNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline TargetPointFollowerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : TargetPointFollowerNode(kDefaultNodeName, "", options)
  {
  }

  ~TargetPointFollowerNode() override {}

private:
  void publish_cmd_vel(const builtin_interfaces::msg::Time & stamp)
  {
    // clculate cmd_vel
    const auto vec = target_point_vec_ - position_vec_;
    const auto rot_vec = direction_x_vec_.cross(vec);
    const auto rot_length = rot_vec.length();
    const auto rot_length_inv = 1.0 / rot_length;
    const auto dot = direction_x_vec_.dot(vec);
    const auto angle = std::atan2(rot_length, dot);
    const auto distance = rot_length != 0 ?
      direction_x_vec_.length() * vec.length2() / rot_length * angle :
      vec.length();
    const auto linear_speed = distance * cmd_vel_distance_rate_;
    const auto angular_speed = angle * cmd_vel_distance_rate_;
    const auto angular_vel_y = direction_y_vec_.dot(rot_vec) * rot_length_inv * angular_speed;
    const auto angular_vel_z = direction_z_vec_.dot(rot_vec) * rot_length_inv * angular_speed;

    // publish cmd_vel
    auto cmd_vel_msg = std::make_unique<TwistMsg>();
    cmd_vel_msg->linear.x = linear_speed;
    cmd_vel_msg->angular.y = angular_vel_y;
    cmd_vel_msg->angular.z = angular_vel_z;
    cmd_vel_pub_->publish(std::move(cmd_vel_msg));

    // publish cmd_vel_stamped
    if (cmd_vel_stamped_pub_) {
      auto cmd_vel_stamped_msg = std::make_unique<TwistStampedMsg>();
      cmd_vel_stamped_msg->header.stamp = stamp;
      cmd_vel_stamped_msg->header.frame_id = pose_frame_id_;
      cmd_vel_stamped_msg->twist.linear.x = linear_speed;
      cmd_vel_stamped_msg->twist.angular.y = angular_vel_y;
      cmd_vel_stamped_msg->twist.angular.z = angular_vel_z;
      cmd_vel_stamped_pub_->publish(std::move(cmd_vel_stamped_msg));
    }
  }

  void update_pose(const PoseMsg::ConstSharedPtr & pose_msg)
  {
    // check frame_id
    if (frame_id_.empty()) {
      frame_id_ = pose_msg->header.frame_id;
    } else if (frame_id_ != pose_msg->header.frame_id) {
      RCLCPP_WARN(
        this->get_logger(),
        "pose frame_id '%s' does not match current frame_id '%s'. Topic is ignored.",
        pose_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    // set position_vec_ and direction vecs
    tf2::fromMsg(pose_msg->pose.position, position_vec_);
    tf2::Quaternion quat;
    tf2::fromMsg(pose_msg->pose.orientation, quat);
    const auto rotation_mat = tf2::Matrix3x3(quat);
    direction_x_vec_ = rotation_mat.getColumn(0);
    direction_y_vec_ = rotation_mat.getColumn(1);
    direction_z_vec_ = rotation_mat.getColumn(2);

    // publish cmd_vel
    if (target_point_received_) {
      this->publish_cmd_vel(pose_msg->header.stamp);
    }
  }

  void update_target_point(const PointMsg::ConstSharedPtr & target_point_msg)
  {
    // check frame_id
    if (frame_id_.empty()) {
      return;
    }

    if (target_point_msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(
        this->get_logger(),
        "target_point frame_id '%s' does not match current frame_id '%s'. Topic is ignored.",
        target_point_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    // set target_point_vec_
    tf2::fromMsg(target_point_msg->point, target_point_vec_);

    // publish cmd_vel
    this->publish_cmd_vel(target_point_msg->header.stamp);
  }

  // parameters
  std::string frame_id_, pose_frame_id_;
  double cmd_vel_distance_rate_;

  // pose & target point
  tf2::Vector3 position_vec_, direction_x_vec_, direction_y_vec_, direction_z_vec_,
    target_point_vec_;
  bool target_point_received_;

  // publisher
  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<TwistStampedMsg>::SharedPtr cmd_vel_stamped_pub_;

  // subscriptions
  rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub_;
  rclcpp::Subscription<PointMsg>::SharedPtr target_point_sub_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__TARGET_POINT_FOLLOWER_NODE_HPP_
