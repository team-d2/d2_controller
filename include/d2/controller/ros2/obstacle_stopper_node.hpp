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

#ifndef D2__CONTROLLER__ROS2__OBSTACLE_STOPPER_NODE_HPP_
#define D2__CONTROLLER__ROS2__OBSTACLE_STOPPER_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "d2/controller/obstacle_stopper.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/convert.hpp"
#include "tf2/transform_datatypes.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class ObstaceleStopperNode : public rclcpp::Node
{
  using PathMsg = nav_msgs::msg::Path;
  using PointCloudMsg = sensor_msgs::msg::PointCloud2;
  using ObstacleStopperTf2 = ObstacleStopper<tf2::Vector3, tf2::Matrix3x3>;

public:
  static constexpr auto kDefaultNodeName = "d2_obstacle_stopper";

  D2__CONTROLLER__ROS2_PUBLIC
  inline ObstaceleStopperNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    robot_bb_{
      this->declare_parameter<double>("robot.x_min", -0.75),
      this->declare_parameter<double>("robot.x_max", 0.35),
      this->declare_parameter<double>("robot.y_min", -0.35),
      this->declare_parameter<double>("robot.y_max", 0.75),
      this->declare_parameter<double>("robot.z_min", 0.1),
      this->declare_parameter<double>("robot.z_max", 1.5)},
    frame_id_(this->declare_parameter<std::string>("frame_id", "")),
    obstacle_stopper_(),
    lidar_point_vecs_(),
    clipped_by_obstacle_plan_publisher_(this->create_qos_overridable_publisher<PathMsg>(
        "local_plan_clipped_by_obstacle", rclcpp::QoS(1))),
    plan_subscription_(this->create_qos_overridable_subscription<PathMsg>(
        "local_plan", rclcpp::QoS(1),
        [this](PathMsg::ConstSharedPtr msg) {update_plan(std::move(msg));})),
    lidar_points_subscription_(
      this->create_qos_overridable_subscription<PointCloudMsg>(
        "lidar/points", rclcpp::QoS(1),
        [this](PointCloudMsg::ConstSharedPtr msg) {update_lidar_points(std::move(msg));}))
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline ObstaceleStopperNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ObstaceleStopperNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline ObstaceleStopperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ObstaceleStopperNode(kDefaultNodeName, "", options)
  {
  }

  ~ObstaceleStopperNode() override {}

private:
  template<class Msg>
  inline typename rclcpp::Publisher<Msg>::SharedPtr create_qos_overridable_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos)
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_publisher<Msg>(topic_name, qos, options);
  }

  template<class Msg, class Func>
  inline typename rclcpp::Subscription<Msg>::SharedPtr create_qos_overridable_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, Func && callback)
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    return this->create_subscription<Msg>(topic_name, qos, std::forward<Func>(callback), options);
  }

  void clip_plan_by_obstacle_points(const std_msgs::msg::Header & header_msg_data)
  {
    // clip plan
    const auto clipped_plan =
      obstacle_stopper_.clip_plan_by_obstacle_points(robot_bb_, lidar_point_vecs_);

    // create clipped plan msg
    auto plan_msg = std::make_unique<PathMsg>();
    plan_msg->header = header_msg_data;
    plan_msg->poses.reserve(clipped_plan.size());
    for (const auto & [stamp, position, rotation] : clipped_plan) {
      geometry_msgs::msg::PoseStamped pose_msg;
      const auto stamp_count = stamp.time_since_epoch().count();
      pose_msg.header.stamp.sec = stamp_count * 1e-9;
      pose_msg.header.stamp.nanosec =
        stamp_count - (pose_msg.header.stamp.sec * static_cast<std::int64_t>(1e9));
      tf2::toMsg(position, pose_msg.pose.position);
      tf2::Quaternion rotation_quat;
      rotation.getRotation(rotation_quat);
      pose_msg.pose.orientation = tf2::toMsg(rotation_quat);
      plan_msg->poses.push_back(pose_msg);
    }

    // publish clipped plan
    clipped_by_obstacle_plan_publisher_->publish(std::move(plan_msg));
  }

  void update_plan(PathMsg::ConstSharedPtr plan_msg)
  {
    // check frame_id
    if (frame_id_.empty()) {
      if (plan_msg->header.frame_id.empty()) {
        RCLCPP_ERROR(this->get_logger(), "plan frame_id is empty. Topic is ignored.");
        return;
      }
      frame_id_ = plan_msg->header.frame_id;
    } else if (plan_msg->header.frame_id != frame_id_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "plan frame_id '%s' does not match current frame_id '%s'. "
        "Topic is ignored.",
        plan_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    for (const auto & pose_msg_data : plan_msg->poses) {
      if (
        pose_msg_data.header.frame_id != plan_msg->header.frame_id &&
        !pose_msg_data.header.frame_id.empty())
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "plan frame_id '%s' does not match current frame_id '%s'. "
          "Topic is ignored.",
          pose_msg_data.header.frame_id.c_str(), plan_msg->header.frame_id.c_str());
        return;
      }
    }

    // plan_msg to tfs
    std::vector<tf2::Stamped<tf2::Transform>> point_tf_stampeds;
    point_tf_stampeds.reserve(plan_msg->poses.size());
    for (const auto & pose_msg_data : plan_msg->poses) {
      tf2::Stamped<tf2::Transform> point_tf_stamped;
      tf2::fromMsg(pose_msg_data, point_tf_stamped);
      point_tf_stampeds.push_back(point_tf_stamped);
    }

    // set obstacle stopper
    obstacle_stopper_ = ObstacleStopperTf2(point_tf_stampeds);

    // clip
    this->clip_plan_by_obstacle_points(plan_msg->header);
  }

  void update_lidar_points(PointCloudMsg::ConstSharedPtr points_msg)
  {
    // check frame_id
    if (points_msg->header.frame_id != frame_id_) {
      RCLCPP_ERROR(this->get_logger(), "point cloud frame_id is empty. Topic is ignored.");
      return;
    }

    // msg to point_vecs
    {
      pcl::PointCloud<pcl::PointXYZ> points;
      pcl::fromROSMsg(*points_msg, points);
      lidar_point_vecs_.clear();
      lidar_point_vecs_.reserve(points.size());
      for (const auto & point : points.points) {
        lidar_point_vecs_.emplace_back(point.x, point.y, point.z);
      }
    }

    // publish clipped plan
    this->clip_plan_by_obstacle_points(points_msg->header);
  }

  // parameters
  BoundingBox3D robot_bb_;

  // plan info
  std::string frame_id_;
  ObstacleStopperTf2 obstacle_stopper_;

  // lidar points info
  std::vector<tf2::Vector3> lidar_point_vecs_;

  // clipped by object plan publisher
  rclcpp::Publisher<PathMsg>::SharedPtr clipped_by_obstacle_plan_publisher_;

  // plan & lidar_points subscription
  rclcpp::Subscription<PathMsg>::SharedPtr plan_subscription_;
  rclcpp::Subscription<PointCloudMsg>::SharedPtr lidar_points_subscription_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__OBSTACLE_STOPPER_NODE_HPP_
