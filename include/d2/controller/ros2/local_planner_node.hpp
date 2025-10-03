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

#include "rclcpp/node.hpp"
#include "d2/controller/calculate_local_plan.hpp"
#include "d2/controller/ros2/adapter/path.hpp"
#include "d2/controller/ros2/adapter/pose_with_covariance_stamped.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class LocalPlannerNode : public rclcpp::Node
{
  using Path = adapter::Path::custom_type;
  using PoseWithCovarianceStamped = adapter::PoseWithCovarianceStamped::custom_type;
  using PathMsg = nav_msgs::msg::Path;
  using PoseMsg = geometry_msgs::msg::PoseStamped;

public:
  static constexpr auto kDefaultNodeName = "local_planner";

  D2__CONTROLLER__ROS2_PUBLIC
  inline LocalPlannerNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(),
    global_plan_(),
    pose_(),
    local_plan_pub_(this->create_local_plan_publisher()),
    global_plan_sub_(this->create_global_plan_subscription()),
    pose_sub_(this->create_pose_subscription())
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
  inline rclcpp::Publisher<adapter::Path>::SharedPtr create_local_plan_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_publisher<adapter::Path>("local_plan", rclcpp::QoS(10).transient_local(), options);
  }

  inline rclcpp::Subscription<adapter::Path>::SharedPtr create_global_plan_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_subscription<adapter::Path>(
      "global_plan", rclcpp::QoS(10).transient_local(),
      [this](std::shared_ptr<const Path> path) {this->update_global_plan(std::move(path));}, options);
  }

  inline rclcpp::Subscription<adapter::PoseWithCovarianceStamped>::SharedPtr create_pose_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability};
    return this->create_subscription<adapter::PoseWithCovarianceStamped>(
      "pose", rclcpp::QoS(10).best_effort(),
      [this](std::shared_ptr<const PoseWithCovarianceStamped> path) {this->update_pose(std::move(path));}, options);
  }

  void publish_local_plan(std::uint64_t stamp_nanosec)
  {
    const auto pose_without_cov = Stamped<Eigen::Isometry3d>{
      pose_->data.stamp_nanosec, pose_->data.data.data};
    auto local_plan = std::make_unique<Path>();
    local_plan->frame_id = frame_id_;
    local_plan->data.stamp_nanosec = stamp_nanosec;
    local_plan->data.data = calculate_local_plan(global_plan_->data.data, pose_without_cov);
    local_plan_pub_->publish(std::move(local_plan));
  }

  void update_global_plan(std::shared_ptr<const Path> global_plan)
  {
    // check frame_id
    if (global_plan->frame_id != frame_id_) {
      RCLCPP_WARN(
        this->get_logger(),
        "global_plan frame_id '%s' does not match pose frame_id '%s'. "
        "Topic is ignored.",
        global_plan->frame_id.c_str(), frame_id_.c_str());
      return;
    }

    // check path has nan
    for (const auto & point_stamped : global_plan->data.data) {
      if (point_stamped.data.translation().array().isNaN().any() || point_stamped.data.linear().array().isNaN().any()) {
        RCLCPP_WARN(
          this->get_logger(),
          "global_plan has NaN. Topic is ignored. Check frame_ids.");
        return;
      }
    }

    // update global_plan
    global_plan_ = std::move(global_plan);

    // publish local plan
    this->publish_local_plan(global_plan_->data.stamp_nanosec);
  }

  void update_pose(std::shared_ptr<const PoseWithCovarianceStamped> pose)
  {
    // check frame_id
    if (frame_id_ != pose->frame_id) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "pose frame_id '%s' does not match frame_id '%s'. "
        "Topic is ignored.",
        pose->frame_id.c_str(), frame_id_.c_str());
      return;
    }

    // update pose
    pose_ = std::move(pose);

    // chack initialized global_plan
    if (!global_plan_) {
      return;
    }

    // publish local plan
    this->publish_local_plan(pose_->data.stamp_nanosec);
  }

  // parameter
  std::string frame_id_;

  // path & pose
  std::shared_ptr<const Path> global_plan_;
  std::shared_ptr<const PoseWithCovarianceStamped> pose_;

  // publisher
  rclcpp::Publisher<adapter::Path>::SharedPtr local_plan_pub_;

  // subscriptions
  rclcpp::Subscription<adapter::Path>::SharedPtr global_plan_sub_;
  rclcpp::Subscription<adapter::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__LOCAL_PLANNER_NODE_HPP_
