#ifndef D2__CONTROLLER__ROS2__ADAPTOR__OBSTACLE_ARRAY_HPP_
#define D2__CONTROLLER__ROS2__ADAPTOR__OBSTACLE_ARRAY_HPP_

#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>
#include <limits>

#include "Eigen/Core"
#include "d2/controller/stamped.hpp"
#include "d2/controller/with_frame_id.hpp"
#include "d2/controller/calculate_obstacle_avoidance_path.hpp"
#include "rclcpp/type_adapter.hpp"
#include "d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp"
  

template <>
class rclcpp::TypeAdapter<
  d2::controller::WithFrameId<d2::controller::Stamped<std::vector<d2::controller::Stamped<std::vector<d2::controller::Line>>>>>,
  d2_costmap_converter_msgs::msg::ObstacleArrayMsg>
{
public:
  using is_specialized = std::true_type;
  using custom_type = d2::controller::WithFrameId<d2::controller::Stamped<std::vector<std::vector<d2::controller::Line>>>>;
  using ros_message_type = d2_costmap_converter_msgs::msg::ObstacleArrayMsg;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    static constexpr std::uint64_t kU1e9 = 1e9;
    destination.header.stamp.sec = source.data.stamp_nanosec / kU1e9;
    destination.header.stamp.nanosec = source.data.stamp_nanosec - (destination.header.stamp.sec * kU1e9);
    destination.header.frame_id = source.frame_id;
    destination.obstacles.clear();
    destination.obstacles.reserve(source.data.data.size());
    auto cnt = 0;
    for (const auto & obstacle_polygon : source.data.data) {
      d2_costmap_converter_msgs::msg::ObstacleMsg obstacle_msg;
      obstacle_msg.header = destination.header;
      obstacle_msg.id = cnt;
      obstacle_msg.polygon.points.clear();
      obstacle_msg.polygon.points.reserve(obstacle_polygon.size());
      for (const auto & obstacle_line : obstacle_polygon) {
        geometry_msgs::msg::Point32 point;
        point.x = obstacle_line.point.x();
        point.y = obstacle_line.point.y();
        point.z = 0.0f;
        obstacle_msg.polygon.points.push_back(point);
      }
      destination.obstacles.push_back(obstacle_msg);
      ++cnt;
    }
  }

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    static constexpr std::uint64_t kU1e9 = 1e9;
    destination.frame_id = source.header.frame_id;
    destination.data.stamp_nanosec = source.header.stamp.sec * kU1e9 + source.header.stamp.nanosec;
    destination.data.data.clear();
    destination.data.data.reserve(source.obstacles.size());
    for (const auto & obstacle_msg_data : source.obstacles) {
      // check frame_id
      if (!obstacle_msg_data.header.frame_id.empty() && obstacle_msg_data.header.frame_id != source.header.frame_id) {
        const auto nan_line = d2::controller::Line{
          Eigen::Vector2d::Zero() * std::numeric_limits<double>::quiet_NaN(),
          Eigen::Vector2d::Zero() * std::numeric_limits<double>::quiet_NaN()};
        destination.data.data.emplace_back(nan_line, obstacle_msg_data.polygon.points.size());
        continue;
      }

      std::vector<d2::controller::Line> obstacle_polygon;
      obstacle_polygon.reserve(obstacle_msg_data.polygon.points.size());
      for (std::size_t i = 0; i < obstacle_msg_data.polygon.points.size(); ++i) {
        const auto & point = obstacle_msg_data.polygon.points[i];
        const auto & next_point = obstacle_msg_data.polygon.points[(i + 1) % obstacle_msg_data.polygon.points.size()];
        obstacle_polygon.push_back(d2::controller::Line{
          Eigen::Vector2d(point.x, point.y),
          Eigen::Vector2d(next_point.x - point.x, next_point.y - point.y)});
      }
      destination.data.data.emplace_back(obstacle_polygon);
    }
  }
};

namespace d2::controller::ros2::adapter
{

using ObstacleArray = rclcpp::TypeAdapter<
  d2::controller::WithFrameId<d2::controller::Stamped<std::vector<std::vector<d2::controller::Line>>>>,
  d2_costmap_converter_msgs::msg::ObstacleArrayMsg>;

}


#endif