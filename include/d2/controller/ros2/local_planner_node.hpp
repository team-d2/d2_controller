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

#ifndef D2__CONTROLLER__ROS2__LOCAL_PLANNER_NODE_HPP_
#define D2__CONTROLLER__ROS2__LOCAL_PLANNER_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "d2/controller/local_planner.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/node.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/convert.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class LocalPlannerNode : public rclcpp::Node
{
  using PathMsg = nav_msgs::msg::Path;
  using PoseMsg = geometry_msgs::msg::PoseStamped;
  using TfMsg = geometry_msgs::msg::TransformStamped;

public:
  static constexpr auto kDefaultNodeName = "d2_local_planner";

  D2__CONTROLLER__ROS2_PUBLIC
  inline LocalPlannerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(),
    global_plan_(),
    pose_opt_(std::nullopt),
    local_plan_pub_(this->create_publisher<PathMsg>("local_plan", 10)),
    global_plan_sub_(this->create_subscription<PathMsg>(
        "global_plan", 10,
        [this](PathMsg::SharedPtr msg) {this->update_global_plan(std::move(msg));})),
    pose_sub_(
      this->create_subscription<PoseMsg>(
        "pose", 10, [this](PoseMsg::SharedPtr msg) {this->update_pose(std::move(msg));}))
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline LocalPlannerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LocalPlannerNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LocalPlannerNode(kDefaultNodeName, "", options)
  {
  }

  ~LocalPlannerNode() override {}

private:
  void publish_local_plan()
  {
    if (!pose_opt_) {
      return;
    }
    const auto & pose = pose_opt_.value();

    const auto local_plan = create_local_plan(global_plan_, pose);
    auto local_plan_msg = std::make_unique<PathMsg>();
    local_plan_msg->header.stamp = rclcpp::Time(pose.stamp_nanosec);
    local_plan_msg->header.frame_id = frame_id_;
    local_plan_msg->poses.reserve(local_plan.size());
    for (const auto & point_stamped : local_plan) {
      auto pose_msg = PoseMsg(rosidl_runtime_cpp::MessageInitialization::SKIP);
      pose_msg.header.stamp = rclcpp::Time(point_stamped.stamp_nanosec);
      pose_msg.header.frame_id = "";
      pose_msg.pose.position.x = point_stamped.data.x;
      pose_msg.pose.position.y = point_stamped.data.y;
      pose_msg.pose.position.z = point_stamped.data.z;
      pose_msg.pose.orientation.x = 0.0;  // Default orientation
      pose_msg.pose.orientation.y = 0.0;  // Default orientation
      pose_msg.pose.orientation.z = 0.0;  // Default orientation
      pose_msg.pose.orientation.w = 1.0;  // Default orientation
      local_plan_msg->poses.push_back(pose_msg);
    }
    local_plan_pub_->publish(std::move(local_plan_msg));
  }

  void update_global_plan(const PathMsg::SharedPtr & global_plan_msg)
  {
    // check frame_id
    if (frame_id_.empty()) {
      return;
    }

    if (global_plan_msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(
        this->get_logger(),
        "global_plan frame_id '%s' does not match pose frame_id '%s'. "
        "Topic is ignored.",
        global_plan_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    for (const auto & pose_msg_data : global_plan_msg->poses) {
      if (!pose_msg_data.header.frame_id.empty() && pose_msg_data.header.frame_id != frame_id_) {
        RCLCPP_WARN(
          this->get_logger(),
          "global_plan pose frame_id '%s' does not match pose frame_id '%s'. "
          "Topic is ignored.",
          pose_msg_data.header.frame_id.c_str(), frame_id_.c_str());
        return;
      }
    }

    global_plan_.clear();
    global_plan_.reserve(global_plan_msg->poses.size());
    for (const auto & pose_msg_data : global_plan_msg->poses) {
      Stamped<Vector3> point_stamped;
      point_stamped.stamp_nanosec =
        static_cast<std::uint64_t>(pose_msg_data.header.stamp.sec * 1e9) +
        pose_msg_data.header.stamp.nanosec;
      point_stamped.data.x = pose_msg_data.pose.position.x;
      point_stamped.data.y = pose_msg_data.pose.position.y;
      point_stamped.data.z = pose_msg_data.pose.position.z;
      global_plan_.push_back(point_stamped);
    }

    // publish local plan
    this->publish_local_plan();
  }

  void update_pose(const PoseMsg::SharedPtr & pose_msg)
  {
    // get pose tf
    if (frame_id_.empty()) {
      frame_id_ = pose_msg->header.frame_id;
    } else if (frame_id_ != pose_msg->header.frame_id) {
      RCLCPP_WARN(
        this->get_logger(),
        "pose frame_id '%s' does not match current frame_id '%s'. "
        "Topic is ignored.",
        pose_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    tf2::Vector3 pose_vec;
    tf2::fromMsg(pose_msg->pose.position, pose_vec);

    // update pose
    Stamped<Vector3> pose;
    pose.stamp_nanosec =
      static_cast<std::uint64_t>(pose_msg->header.stamp.sec * 1e9) + pose_msg->header.stamp.nanosec;
    pose.data.x = pose_vec.x();
    pose.data.y = pose_vec.y();
    pose.data.z = pose_vec.z();
    pose_opt_ = pose;

    // publish local plan
    this->publish_local_plan();
  }

  // path & pose
  std::string frame_id_;
  std::vector<Stamped<Vector3>> global_plan_;
  std::optional<Stamped<Vector3>> pose_opt_;

  // publisher
  rclcpp::Publisher<PathMsg>::SharedPtr local_plan_pub_;

  // subscriptions
  rclcpp::Subscription<PathMsg>::SharedPtr global_plan_sub_;
  rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__LOCAL_PLANNER_NODE_HPP_
