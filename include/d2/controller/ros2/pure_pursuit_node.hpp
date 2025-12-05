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

#include "d2/controller/calculate_forrow_point_vel.hpp"
#include "d2/controller/calculate_lookahead_point.hpp"
#include "d2/controller/ros2/adapter/path.hpp"
#include "d2/controller/ros2/adapter/point_stamped.hpp"
#include "d2/controller/ros2/adapter/twist.hpp"
// #include "d2/controller/calculate_lookahead_point_with_avoidance.hpp"
#include "d2/controller/calculate_obstacle_avoidance_path_with_direction.hpp"
#include "d2/controller/smooth_path.hpp"
#include "d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp"
#include "rclcpp/node.hpp"
#include "visibility.hpp"

namespace d2::controller::ros2
{

class PurePursuitNode : public rclcpp::Node
{
  using Path = adapter::Path::custom_type;
  using PointStamped = adapter::PointStamped::custom_type;
  using Twist = adapter::Twist::custom_type;
  using ObstacleArrayMsg = d2_costmap_converter_msgs::msg::ObstacleArrayMsg;

public:
  static constexpr auto kDefaultNodeName = "pure_pursuit";

  D2__CONTROLLER__ROS2_PUBLIC
  inline PurePursuitNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    frame_id_(this->declare_parameter<std::string>("frame_id", "map")),
    lookahead_distance_(this->declare_parameter<double>("purepursuit.lookahead_distance", 1.0)),
    follow_point_param_{
      this->declare_parameter<double>("follow_point.margin_distance", 1.0),
      this->declare_parameter<double>("follow_point.vel_per_distance", 1.0)},
    enable_avoidance_(this->declare_parameter<bool>("enable_avoidance", false)),
    path_smoothing_count_(this->declare_parameter<int>("path_smoothing.count", 5)),
    path_smoothing_distance_(this->declare_parameter<double>("path_smoothing.distance", 0.1)),
    last_lookahead_point_(Eigen::Vector3d(
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()
    )),
    cmd_vel_pub_(this->create_cmd_vel_publisher()),
    target_point_pub_(this->create_target_point_publisher()),
    forward_target_point_pub_(this->create_forward_target_point_publisher()),
    backward_target_point_pub_(this->create_backward_target_point_publisher()),
    local_plan_sub_(this->create_local_plan_subscription()),
    obstacles_subscription_(this->create_obstacles_subscription())
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
  inline rclcpp::Publisher<adapter::Twist>::SharedPtr create_cmd_vel_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability};
    return this->create_publisher<adapter::Twist>("cmd_vel", rclcpp::QoS(10).best_effort(), options);
  }

  inline rclcpp::Publisher<adapter::PointStamped>::SharedPtr create_target_point_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability};
    return this->create_publisher<adapter::PointStamped>("target_point", rclcpp::QoS(10).best_effort(), options);
  }

  inline rclcpp::Publisher<adapter::PointStamped>::SharedPtr create_forward_target_point_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability};
    return this->create_publisher<adapter::PointStamped>("ftarget_point", rclcpp::QoS(10).best_effort(), options);
  }

  inline rclcpp::Publisher<adapter::PointStamped>::SharedPtr create_backward_target_point_publisher()
  {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability};
    return this->create_publisher<adapter::PointStamped>("btarget_point", rclcpp::QoS(10).best_effort(), options);
  }
  
  inline rclcpp::Subscription<adapter::Path>::SharedPtr create_local_plan_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability};
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    return this->create_subscription<adapter::Path>(
      "local_plan", rclcpp::QoS(10).transient_local(),
      [this](std::shared_ptr<const Path> path) {this->calculate(std::move(path));}, options);
  }

  inline rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr create_obstacles_subscription()
  {
    rclcpp::SubscriptionOptions options;
    options.qos_overriding_options = {
      rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability};
    return this->create_subscription<ObstacleArrayMsg>(
      "obstacles", rclcpp::QoS(10).best_effort(),
      [this](ObstacleArrayMsg::ConstSharedPtr msg) {this->update_obstacles(std::move(msg));}, options);
  }


  void calculate(std::shared_ptr<const Path> path)
  {
    if (path->data.data.size() == 0) {
      return;
    }

    std::deque<Eigen::Vector2d> base_plan;
    for (const auto & stamped_point : path->data.data) {
      base_plan.emplace_back(
        stamped_point.data.translation().head<2>());
    }

    if (last_lookahead_point_.hasNaN()) {
      last_lookahead_point_ = path->data.data.front().data.translation();
    }

    // calcurate the target point
    auto point_stamped = std::make_unique<PointStamped>();
    auto fpoint_stamped = std::make_unique<PointStamped>();
    auto bpoint_stamped = std::make_unique<PointStamped>();
    point_stamped->frame_id = path->frame_id;

    std::cout << "enable_avoidance: " << (enable_avoidance_ ? "true" : "false") << std::endl;
    if (enable_avoidance_) {
      std::cout << "a" << std::endl;
      auto forward_avoidance_path_2d = calculate_obstacle_avoidance_path(
        base_plan, obstacles_, 0.4, true);
      auto backward_avoidance_path_2d = calculate_obstacle_avoidance_path(
        base_plan, obstacles_, 0.4, false);

      
      std::vector<Stamped<Eigen::Isometry3d>> forward_avoidance_path;
      std::vector<Stamped<Eigen::Isometry3d>> backward_avoidance_path;
      for (const auto & point_2d : forward_avoidance_path_2d) {
        Eigen::Isometry3d point = path->data.data.front().data;
        // std::cout << "f: " << Eigen::Quaterniond(point.rotation()) << std::endl;
        point.translation().head<2>() = point_2d;
        point.translation().z() = path->data.data.front().data.translation().z();
        point.linear() = path->data.data.front().data.linear();
        forward_avoidance_path.emplace_back(Stamped<Eigen::Isometry3d>{0, point});
      }
      for (const auto & point_2d : backward_avoidance_path_2d) {
        Eigen::Isometry3d point = path->data.data.front().data;
        // std::cout << "b: " << Eigen::Quaterniond(point.rotation()) << std::endl;
        point.translation().head<2>() = point_2d;
        point.translation().z() = path->data.data.front().data.translation().z();
        point.linear() = path->data.data.front().data.linear();
        backward_avoidance_path.emplace_back(Stamped<Eigen::Isometry3d>{0, point});
      }

      auto forward_lookahead_point = calculate_lookahead_point(
        forward_avoidance_path, lookahead_distance_);
      auto backward_lookahead_point = calculate_lookahead_point(
        backward_avoidance_path, lookahead_distance_);
      auto basic_lookahead_point = calculate_lookahead_point(
        path->data.data, lookahead_distance_);
      auto forward_dist = is_last_forward_ ? 0.0 : 2.0 + (forward_lookahead_point.data - basic_lookahead_point.data).head<2>().norm();
      auto backward_dist = is_last_forward_ ? 2.0 : 0.0 + (backward_lookahead_point.data - basic_lookahead_point.data).head<2>().norm();

      base_plan =  forward_dist < backward_dist ? forward_avoidance_path_2d : backward_avoidance_path_2d;
      
      for (int i = 0; i < path_smoothing_count_; ++i) {
        forward_avoidance_path_2d = smooth_path(
          base_plan, path_smoothing_distance_);
        forward_avoidance_path_2d = calculate_obstacle_avoidance_path(
        forward_avoidance_path_2d, obstacles_, 0.4, true);


        backward_avoidance_path_2d = smooth_path(
          base_plan, path_smoothing_distance_);
        backward_avoidance_path_2d = calculate_obstacle_avoidance_path(
        backward_avoidance_path_2d, obstacles_, 0.4, false);


      for (const auto & point_2d : forward_avoidance_path_2d) {
        Eigen::Isometry3d point = path->data.data.front().data;
        // std::cout << "f: " << Eigen::Quaterniond(point.rotation()) << std::endl;
        point.translation().head<2>() = point_2d;
        point.translation().z() = path->data.data.front().data.translation().z();
        point.linear() = path->data.data.front().data.linear();
        forward_avoidance_path.emplace_back(Stamped<Eigen::Isometry3d>{0, point});
      }
      for (const auto & point_2d : backward_avoidance_path_2d) {
        Eigen::Isometry3d point = path->data.data.front().data;
        // std::cout << "b: " << Eigen::Quaterniond(point.rotation()) << std::endl;
        point.translation().head<2>() = point_2d;
        point.translation().z() = path->data.data.front().data.translation().z();
        point.linear() = path->data.data.front().data.linear();
        backward_avoidance_path.emplace_back(Stamped<Eigen::Isometry3d>{0, point});
      }

        forward_lookahead_point = calculate_lookahead_point(
          forward_avoidance_path, lookahead_distance_);
        backward_lookahead_point = calculate_lookahead_point(
          backward_avoidance_path, lookahead_distance_);
        // auto basic_lookahead_point = calculate_lookahead_point(
        //   path->data.data, lookahead_distance_);
        forward_dist = (forward_lookahead_point.data - last_lookahead_point_).head<2>().norm();
        backward_dist = (backward_lookahead_point.data - last_lookahead_point_).head<2>().norm();

        base_plan =  forward_dist < backward_dist ? forward_avoidance_path_2d : backward_avoidance_path_2d;


      }
      point_stamped->data = forward_dist < backward_dist ? forward_lookahead_point : backward_lookahead_point;
      // for (int i = 0; i < path_smoothing_count_ && !backward_avoidance_path_2d.empty(); ++i) {
      // }
      // std::vector<Stamped<Eigen::Isometry3d>> forward_avoidance_path;
      // std::vector<Stamped<Eigen::Isometry3d>> backward_avoidance_path;
      // for (const auto & point_2d : forward_avoidance_path_2d) {
      //   Eigen::Isometry3d point = path->data.data.front().data;
      //   // std::cout << "f: " << Eigen::Quaterniond(point.rotation()) << std::endl;
      //   point.translation().head<2>() = point_2d;
      //   point.translation().z() = path->data.data.front().data.translation().z();
      //   point.linear() = path->data.data.front().data.linear();
      //   forward_avoidance_path.emplace_back(Stamped<Eigen::Isometry3d>{0, point});
      // }
      // for (const auto & point_2d : backward_avoidance_path_2d) {
      //   Eigen::Isometry3d point = path->data.data.front().data;
      //   // std::cout << "b: " << Eigen::Quaterniond(point.rotation()) << std::endl;
      //   point.translation().head<2>() = point_2d;
      //   point.translation().z() = path->data.data.front().data.translation().z();
      //   point.linear() = path->data.data.front().data.linear();
      //   backward_avoidance_path.emplace_back(Stamped<Eigen::Isometry3d>{0, point});
      // }
      // auto forward_lookahead_point = calculate_lookahead_point(
      //   forward_avoidance_path, lookahead_distance_);
      // auto backward_lookahead_point = calculate_lookahead_point(
      //   backward_avoidance_path, lookahead_distance_);
      // auto basic_lookahead_point = calculate_lookahead_point(
      //   path->data.data, lookahead_distance_);
      // auto forward_dist = (forward_lookahead_point.data - last_lookahead_point_).head<2>().norm(); // + (forward_lookahead_point.data - basic_lookahead_point.data).head<2>().norm() * 1.5;
      // auto backward_dist = (backward_lookahead_point.data - last_lookahead_point_).head<2>().norm(); // + (backward_lookahead_point.data - basic_lookahead_point.data).head<2>().norm() * 1.5;
      
      // if (forward_dist <= backward_dist) {
      //   point_stamped->data = forward_lookahead_point;
      //   last_lookahead_point_ = forward_lookahead_point.data;
      // } else {
      //   point_stamped->data = backward_lookahead_point;
      //   last_lookahead_point_ = backward_lookahead_point.data;
      // }
      fpoint_stamped->frame_id = bpoint_stamped->frame_id = path->frame_id;
      fpoint_stamped->data.stamp_nanosec = bpoint_stamped->data.stamp_nanosec = this->now().nanoseconds();
      fpoint_stamped->data = forward_lookahead_point;
      bpoint_stamped->data = backward_lookahead_point;
      forward_target_point_pub_->publish(std::move(fpoint_stamped));
      backward_target_point_pub_->publish(std::move(bpoint_stamped));
    }
    else {
      point_stamped->data = calculate_lookahead_point(
        path->data.data, lookahead_distance_);
    }

    // calcurate cmd_vel
    auto cmd_vel = std::make_unique<Twist>();
    *cmd_vel = calculate_forrow_point_vel(
      point_stamped->data.data, path->data.data.front().data,
      follow_point_param_);
    
    // publish cmd_vel
    cmd_vel_pub_->publish(std::move(cmd_vel));
    
    // publish target point
    point_stamped->data.stamp_nanosec = this->now().nanoseconds();
    target_point_pub_->publish(std::move(point_stamped));
  }

  void update_obstacles(ObstacleArrayMsg::ConstSharedPtr obstacles_msg)
  {
    if (obstacles_msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(
        this->get_logger(), "frame_id mismatch: plan frame_id = %s, node frame_id = %s",
        obstacles_msg->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    obstacles_.clear();
    obstacles_.reserve(obstacles_msg->obstacles.size());
    for (const auto & obstacle_msg : obstacles_msg->obstacles) {
      if (obstacle_msg.header.frame_id != frame_id_ && obstacle_msg.header.frame_id != "") {
        RCLCPP_WARN(
          this->get_logger(), "frame_id mismatch: obstacle frame_id = %s, node frame_id = %s",
          obstacle_msg.header.frame_id.c_str(), frame_id_.c_str());
        continue;
      }
      std::vector<Eigen::Vector2d> obstacle;
      obstacle.reserve(obstacle_msg.polygon.points.size());
      for (const auto & point : obstacle_msg.polygon.points) {
        obstacle.emplace_back(point.x, point.y);
      }
      obstacles_.emplace_back(std::move(obstacle));
    }
  }

  // parameter
  std::string frame_id_;
  double lookahead_distance_;
  bool is_last_forward_;
  FollowPointParam follow_point_param_;
  bool enable_avoidance_;
  int path_smoothing_count_;
  double path_smoothing_distance_;

  Eigen::Vector3d last_lookahead_point_;
  std::vector<std::vector<Eigen::Vector2d>> obstacles_;

  // publisher
  rclcpp::Publisher<adapter::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<adapter::PointStamped>::SharedPtr target_point_pub_;
  rclcpp::Publisher<adapter::PointStamped>::SharedPtr forward_target_point_pub_;
  rclcpp::Publisher<adapter::PointStamped>::SharedPtr backward_target_point_pub_;

  // subscriptions
  rclcpp::Subscription<adapter::Path>::SharedPtr local_plan_sub_;
  rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr obstacles_subscription_;
};

}  // namespace d2::controller::ros2

#endif  // D2__CONTROLLER__ROS2__PURE_PURSUIT_NODE_HPP_
