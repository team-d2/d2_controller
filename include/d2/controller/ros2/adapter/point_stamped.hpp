#ifndef D2__CONTROLLER__ROS2__ADAPTOR__POINT_STAMPED_HPP_
#define D2__CONTROLLER__ROS2__ADAPTOR__POINT_STAMPED_HPP_

#include "Eigen/Core"
#include "d2/controller/stamped.hpp"
#include "d2/controller/with_frame_id.hpp"
#include "rclcpp/type_adapter.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

template <>
class rclcpp::TypeAdapter<
  d2::controller::WithFrameId<d2::controller::Stamped<Eigen::Vector3d>>,
  geometry_msgs::msg::PointStamped>
{
public:
  using is_specialized = std::true_type;
  using custom_type = d2::controller::WithFrameId<d2::controller::Stamped<Eigen::Vector3d>>;
  using ros_message_type = geometry_msgs::msg::PointStamped;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    static constexpr std::uint64_t kU1e9 = 1e9;
    destination.header.frame_id = source.frame_id;
    destination.header.stamp.sec = source.data.stamp_nanosec / kU1e9;
    destination.header.stamp.nanosec = source.data.stamp_nanosec - destination.header.stamp.sec * kU1e9;
    destination.point.x = source.data.data.x();
    destination.point.y = source.data.data.y();
    destination.point.z = source.data.data.z();
  }

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    static constexpr std::uint64_t kU1e9 = 1e9;
    destination.frame_id = source.header.frame_id;
    destination.data.stamp_nanosec = source.header.stamp.sec * kU1e9 + source.header.stamp.nanosec;
    destination.data.data = Eigen::Vector3d(
      source.point.x, source.point.y, source.point.z);
  }
};

namespace d2::controller::ros2::adapter
{

using PointStamped = rclcpp::TypeAdapter<
  d2::controller::WithFrameId<d2::controller::Stamped<Eigen::Vector3d>>,
  geometry_msgs::msg::PointStamped>;

}


#endif