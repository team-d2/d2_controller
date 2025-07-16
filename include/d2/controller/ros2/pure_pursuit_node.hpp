#ifndef D2__CONTROLLER__ROS2__PURE_PURSUIT_NODE_HPP_
#define D2__CONTROLLER__ROS2__PURE_PURSUIT_NODE_HPP_

#include <memory>
#include <string>
#include <variant>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visibility.hpp"
#include "d2/controller/pure_pursuit.hpp"
#include "d2/controller/stamped.hpp"
#include "d2/controller/vector3.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/convert.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
    default_frame_id_(this->declare_parameter<std::string>("default_frame_id", "map")),
    lookahead_distance_(this->declare_parameter<double>("lookahead_distance", 1.0)),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    target_point_pub_(this->create_publisher<PointMsg>("target_point", 10)),
    local_plan_sub_(
      this->create_subscription<PathMsg>(
        "local_plan", 10, [this](PathMsg::ConstSharedPtr msg) {
          this->run(std::move(msg));
        }))
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
    // frame_id
    const auto frame_id = local_plan_msg->header.frame_id.empty() ? default_frame_id_ : local_plan_msg->header.frame_id;
    
    // get tfs
    std::map<std::string, tf2::Transform> tf_map = {{"", tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0))}};
    for (const auto & pose_msg_data : local_plan_msg->poses) {
      auto & section_frame_id = pose_msg_data.header.frame_id;
      if (tf_map.count(section_frame_id) > 0) {
        continue;
      }
      TfMsg tf_msg;
      try {
        tf_msg = tf_buffer_.lookupTransform(
          section_frame_id, frame_id, rclcpp::Time(0));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
        return;
      }
      tf2::Transform tf;
      tf2::fromMsg(tf_msg.transform, tf);
      tf_map[section_frame_id] = tf;
    }

    // create path
    std::vector<Stamped<Vector3>> path;
    path.reserve(local_plan_msg->poses.size());
    for (const auto & pose_msg_data : local_plan_msg->poses) {
      tf2::Vector3 pose_relative_vec;
      tf2::fromMsg(pose_msg_data.pose.position, pose_relative_vec);
      const auto pose_vec = tf_map[pose_msg_data.header.frame_id] * pose_relative_vec;

      Stamped<Vector3> point_stamped;
      point_stamped.stamp_nanosec = static_cast<std::uint64_t>(pose_msg_data.header.stamp.sec * 1e9) + pose_msg_data.header.stamp.nanosec;
      point_stamped.data.x = pose_vec.x();
      point_stamped.data.y = pose_vec.y();
      point_stamped.data.z = pose_vec.z();
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
    target_point_msg->header.frame_id = frame_id;
    target_point_msg->header.stamp = local_plan_msg->header.stamp;
    target_point_msg->point.x = target_point_opt->x;
    target_point_msg->point.y = target_point_opt->y;
    target_point_msg->point.z = target_point_opt->z;
    target_point_pub_->publish(std::move(target_point_msg));
  }

  // parameter
  std::string default_frame_id_;
  double lookahead_distance_;

  // tf2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // publisher
  rclcpp::Publisher<PointMsg>::SharedPtr target_point_pub_;

  // subscriptions
  rclcpp::Subscription<PathMsg>::SharedPtr local_plan_sub_;
};

}  // namespace d2__controller::ros2

#endif  // D2__CONTROLLER__ROS2__PURE_PURSUIT_NODE_HPP_
