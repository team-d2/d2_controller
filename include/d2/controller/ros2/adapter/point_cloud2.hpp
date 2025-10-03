#ifndef D2__CONTROLLER__ROS2__ADAPTOR__POINT_CLOUD2_HPP_
#define D2__CONTROLLER__ROS2__ADAPTOR__POINT_CLOUD2_HPP_

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/type_adapter.hpp"

template <>
class rclcpp::TypeAdapter<
  pcl::PointCloud<pcl::PointXYZ>,
  sensor_msgs::msg::PointCloud2>
{
public:
  using is_specialized = std::true_type;
  using custom_type = pcl::PointCloud<pcl::PointXYZ>;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    pcl::toROSMsg(source, destination);
  }

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    pcl::fromROSMsg(source, destination);
  }
};

namespace d2::controller::ros2::adapter
{

using PointCloud2 = rclcpp::TypeAdapter<
  pcl::PointCloud<pcl::PointXYZ>,
  sensor_msgs::msg::PointCloud2>;

}


#endif