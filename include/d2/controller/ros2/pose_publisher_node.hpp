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

#ifndef D2__CONTROLLER__ROS2__POSE_PUBLISHE_NODE_HPP_
#define D2__CONTROLLER__ROS2__POSE_PUBLISHE_NODE_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <variant>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class PosePublisherNode : public rclcpp::Node
{
  using OdomMsg = nav_msgs::msg::Odometry;
  using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  static constexpr auto kDefaultNodeName = "d2_pose_publishe";

  D2__CONTROLLER__ROS2_PUBLIC
  inline PosePublisherNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    timeout_duration_(
      rclcpp::Duration::from_seconds(this->declare_parameter<double>("timeout", 10.0))),
    frame_id_(this->declare_parameter<std::string>("frame_id", "map")),
    timeout_time_(this->now()),
    pose_pub_(this->create_publisher<PoseMsg>("pose", 10)),
    odom_sub_(this->create_subscription<OdomMsg>(
        "odom", rclcpp::QoS(10).best_effort(),
        [this](OdomMsg::ConstSharedPtr msg) {this->set_odom(std::move(msg));})),
    publish_timer_(
      this->create_wall_timer(std::chrono::duration<double>(1.0/10.0), [this]{this->publish_pose();})
    )
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline PosePublisherNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PosePublisherNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline PosePublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : PosePublisherNode(kDefaultNodeName, "", options)
  {
  }

private:
  void set_odom(OdomMsg::ConstSharedPtr odom_msg)
  {
    tf2::fromMsg(odom_msg->pose.pose.position, pose_position_);
    tf2::fromMsg(odom_msg->pose.pose.orientation, pose_orientation_);
    pose_cov_ = odom_msg->pose.covariance;
    timeout_time_ = rclcpp::Time(odom_msg->header.stamp) + timeout_duration_;
    frame_id_ = odom_msg->header.frame_id;
  }

  void publish_pose()
  {
    if (this->now() > timeout_time_) {
      return;
    }
    auto pose_msg = std::make_unique<PoseMsg>();
    pose_msg->header.frame_id = frame_id_;
    tf2::toMsg(pose_position_, pose_msg->pose.pose.position);
    pose_msg->pose.pose.orientation = tf2::toMsg(pose_orientation_);
    
    // pose_msg->pose.covariance = pose_cov_;
    pose_pub_->publish(std::move(pose_msg));
  }

  // parameter
  rclcpp::Duration timeout_duration_;

  // msg
  std::string frame_id_;
  tf2::Vector3 pose_position_;
  tf2::Quaternion pose_orientation_;
  std::array<double, 36> pose_cov_;
  rclcpp::Time timeout_time_;

  // publisher
  rclcpp::Publisher<PoseMsg>::SharedPtr pose_pub_;

  // subscription
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__POSE_PUBLISHE_NODE_HPP_
