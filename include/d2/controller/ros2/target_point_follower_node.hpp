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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
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
    frame_id_(this->declare_parameter<std::string>("default_frame_id", "map")),
    cmd_vel_distance_rate_(this->declare_parameter<double>("cmd_vel_distance_rate", 0.2)),
    initialized_flag_(InitializedFlag::kNone),
    position_vec_(),
    direction_vec_(),
    target_point_vec_(0.0, 0.0, 0.0),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this),
    cmd_vel_stamped_pub_(this->create_publisher<TwistStampedMsg>("cmd_vel/stamped", 10)),
    pose_sub_(this->create_subscription<PoseMsg>(
        "pose", 10, [this](PoseMsg::ConstSharedPtr msg) {this->initialize_pose(std::move(msg));})),
    target_point_sub_(
      this->create_subscription<PointMsg>(
        "target_point", 10,
        [this](PointMsg::ConstSharedPtr msg) {this->initialize_target_point(std::move(msg));}))
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
  void publish_cmd_vel()
  {
    // clculate cmd_vel
    const auto vec = target_point_vec_ - position_vec_;
    const auto rot_vec = direction_vec_.cross(vec);
    const auto rot_length = rot_vec.length();
    const auto dot = direction_vec_.dot(vec);
    const auto angle = std::atan2(rot_length, dot);
    const auto vel_linear_vec =
      direction_vec_ * (vec.length2() * angle / rot_length * cmd_vel_distance_rate_);
    const auto vel_angular_vec = rot_vec * (angle / rot_length * cmd_vel_distance_rate_);

    // publish cmd_vel_stamped
    auto cmd_vel_stamped_msg = std::make_unique<TwistStampedMsg>();
    cmd_vel_stamped_msg->header.stamp = this->now();
    cmd_vel_stamped_msg->header.frame_id = frame_id_;
    cmd_vel_stamped_msg->twist.linear = tf2::toMsg(vel_linear_vec);
    cmd_vel_stamped_msg->twist.angular = tf2::toMsg(vel_angular_vec);
    cmd_vel_stamped_pub_->publish(std::move(cmd_vel_stamped_msg));
  }

  void start_update()
  {
    pose_sub_ = this->create_subscription<PoseMsg>(
      "pose", 10, [this](PoseMsg::ConstSharedPtr msg) {this->update_pose(std::move(msg));});
    target_point_sub_ = this->create_subscription<PointMsg>(
      "target_point", 10,
      [this](PointMsg::ConstSharedPtr msg) {this->update_target_point(std::move(msg));});
    this->publish_cmd_vel();
  }

  void update_frame_id(const std::string & frame_id)
  {
    if (frame_id == frame_id_) {
      return;
    }

    // get tf
    TfMsg tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(frame_id_, frame_id, rclcpp::Time(0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
      return;
    }
    tf2::Transform tf;
    tf2::fromMsg(tf_msg.transform, tf);

    // transform target point
    target_point_vec_ = tf * target_point_vec_;
  }

  void set_pose(PoseMsg::ConstSharedPtr pose_msg)
  {
    this->update_frame_id(pose_msg->header.frame_id);

    frame_id_ = pose_msg->header.frame_id;
    tf2::fromMsg(pose_msg->pose.position, position_vec_);
    tf2::Quaternion orientation;
    tf2::fromMsg(pose_msg->pose.orientation, orientation);
    direction_vec_ = tf2::Matrix3x3(orientation) * tf2::Vector3(1.0, 0.0, 0.0);
  }

  void set_target_point(PointMsg::ConstSharedPtr target_point_msg)
  {
    // get target point tf
    TfMsg tf_msg;
    try {
      tf_msg =
        tf_buffer_.lookupTransform(target_point_msg->header.frame_id, frame_id_, rclcpp::Time(0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
      return;
    }
    tf2::Transform tf;
    tf2::fromMsg(tf_msg.transform, tf);

    // set target_point_vec
    tf2::Vector3 target_point_relative_vec;
    tf2::fromMsg(target_point_msg->point, target_point_relative_vec);
    target_point_vec_ = tf * target_point_relative_vec;
  }

  void initialize_pose(PoseMsg::ConstSharedPtr pose_msg)
  {
    // set pose
    this->set_pose(std::move(pose_msg));

    // set flag
    initialized_flag_ = static_cast<InitializedFlag>(
      static_cast<int>(initialized_flag_) | static_cast<int>(InitializedFlag::kPose));

    // if both pose and target point are initialized, start update
    if (initialized_flag_ == InitializedFlag::kAll) {
      this->start_update();
    }
  }

  void initialize_target_point(PointMsg::ConstSharedPtr target_point_msg)
  {
    // set target point
    this->set_target_point(std::move(target_point_msg));

    // set flag
    initialized_flag_ = static_cast<InitializedFlag>(
      static_cast<int>(initialized_flag_) | static_cast<int>(InitializedFlag::kTargetPoint));

    // if both pose and target point are initialized, start update
    if (initialized_flag_ == InitializedFlag::kAll) {
      this->start_update();
    }
  }

  void update_pose(const PoseMsg::ConstSharedPtr & pose_msg)
  {
    this->set_pose(pose_msg);
    this->publish_cmd_vel();
  }

  void update_target_point(const PointMsg::ConstSharedPtr & target_point_msg)
  {
    this->set_target_point(target_point_msg);
    this->publish_cmd_vel();
  }

  // parameters
  std::string frame_id_;
  double cmd_vel_distance_rate_;

  // initialized flag
  enum class InitializedFlag
  {
    kNone = 0,
    kPose = 1 << 0,
    kTargetPoint = 1 << 1,
    kAll = kPose | kTargetPoint,
  } initialized_flag_;

  // pose & target point
  tf2::Vector3 position_vec_, direction_vec_, target_point_vec_;

  // tf2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // publisher
  rclcpp::Publisher<TwistStampedMsg>::SharedPtr cmd_vel_stamped_pub_;

  // subscriptions
  rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub_;
  rclcpp::Subscription<PointMsg>::SharedPtr target_point_sub_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__TARGET_POINT_FOLLOWER_NODE_HPP_
