#ifndef D2__CONTROLLER__ROS2__ADAPTOR__TWIST_HPP_
#define D2__CONTROLLER__ROS2__ADAPTOR__TWIST_HPP_

#include "Eigen/Core"
#include "rclcpp/type_adapter.hpp"
#include "geometry_msgs/msg/twist.hpp"

template <>
class rclcpp::TypeAdapter<
  Eigen::Vector<double, 6>,
  geometry_msgs::msg::Twist>
{
public:
  using is_specialized = std::true_type;
  using custom_type = Eigen::Vector<double, 6>;
  using ros_message_type = geometry_msgs::msg::Twist;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.linear.x = source[0];
    destination.linear.y = source[1];
    destination.linear.z = source[2];
    destination.angular.x = source[3];
    destination.angular.y = source[4];
    destination.angular.z = source[5];
  }

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination[0] = source.linear.x;
    destination[1] = source.linear.y;
    destination[2] = source.linear.z;
    destination[3] = source.angular.x;
    destination[4] = source.angular.y;
    destination[5] = source.angular.z;
  }
};

namespace d2::controller::ros2::adapter
{

using Twist = rclcpp::TypeAdapter<Eigen::Vector<double, 6>, geometry_msgs::msg::Twist>;

}


#endif