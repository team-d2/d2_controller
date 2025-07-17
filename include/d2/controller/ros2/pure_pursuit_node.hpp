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
#include <vector>

#include "d2/controller/pure_pursuit.hpp"
#include "d2/controller/stamped.hpp"
#include "d2/controller/vector3.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/convert.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class PurePursuitNode : public rclcpp::Node
{
  using PointMsg = geometry_msgs::msg::PointStamped;
  using PathMsg = nav_msgs::msg::Path;
  using TfMsg = geometry_msgs::msg::TransformStamped;

public:
  static constexpr auto kDefaultNodeName = "d2_pure_pursuit";

  D2__CONTROLLER__ROS2_PUBLIC
  inline PurePursuitNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    lookahead_distance_(this->declare_parameter<double>("lookahead_distance", 1.0)),
    target_point_pub_(this->create_publisher<PointMsg>("target_point", 10)),
    local_plan_sub_(this->create_subscription<PathMsg>(
        "local_plan", 10, [this](PathMsg::ConstSharedPtr msg) {this->run(std::move(msg));}))
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
  void run(const PathMsg::ConstSharedPtr & local_plan_msg)
  {
    // check frame_id
    for (const auto & pose_msg_data : local_plan_msg->poses) {
      if (
        !pose_msg_data.header.frame_id.empty() &&
        pose_msg_data.header.frame_id != local_plan_msg->header.frame_id)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "global_plan pose frame_id '%s' does not match global_plan frame_id '%s'. "
          "Topic is ignored.",
          pose_msg_data.header.frame_id.c_str(), local_plan_msg->header.frame_id.c_str());
        return;
      }
    }

    // create path
    std::vector<Stamped<Vector3>> path;
    path.reserve(local_plan_msg->poses.size());
    for (const auto & pose_msg_data : local_plan_msg->poses) {
      Stamped<Vector3> point_stamped;
      point_stamped.stamp_nanosec =
        static_cast<std::uint64_t>(pose_msg_data.header.stamp.sec * 1e9) +
        pose_msg_data.header.stamp.nanosec;
      point_stamped.data.x = pose_msg_data.pose.position.x;
      point_stamped.data.y = pose_msg_data.pose.position.y;
      point_stamped.data.z = pose_msg_data.pose.position.z;
      path.push_back(point_stamped);
    }

    // create purpursuit point
    auto target_point_opt = create_pure_pursuit_point(path, lookahead_distance_);
    if (!target_point_opt.has_value()) {
      RCLCPP_WARN_ONCE(this->get_logger(), "No target point found in the local plan.");
      return;
    }

    // publish the target point
    auto target_point_msg = std::make_unique<PointMsg>();
    target_point_msg->header.frame_id = local_plan_msg->header.frame_id;
    target_point_msg->header.stamp = local_plan_msg->header.stamp;
    target_point_msg->point.x = target_point_opt->x;
    target_point_msg->point.y = target_point_opt->y;
    target_point_msg->point.z = target_point_opt->z;
    target_point_pub_->publish(std::move(target_point_msg));
  }

  // parameter
  double lookahead_distance_;

  // publisher
  rclcpp::Publisher<PointMsg>::SharedPtr target_point_pub_;

  // subscriptions
  rclcpp::Subscription<PathMsg>::SharedPtr local_plan_sub_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__PURE_PURSUIT_NODE_HPP_
