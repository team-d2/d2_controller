#ifndef D2__CONTROLLER__ROS2__ADAPTOR__POINT_STAMPED_HPP_
#define D2__CONTROLLER__ROS2__ADAPTOR__POINT_STAMPED_HPP_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "d2/controller/stamped.hpp"
#include "d2/controller/with_frame_id.hpp"
#include "d2/controller/with_covariance.hpp"
#include "rclcpp/type_adapter.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

template <>
class rclcpp::TypeAdapter<
  d2::controller::WithFrameId<d2::controller::Stamped<d2::controller::WithCovariance<6, Eigen::Isometry3d>>>,
  geometry_msgs::msg::PoseWithCovarianceStamped>
{
public:
  using is_specialized = std::true_type;
  using custom_type = d2::controller::WithFrameId<d2::controller::Stamped<d2::controller::WithCovariance<6, Eigen::Isometry3d>>>;
  using ros_message_type = geometry_msgs::msg::PoseWithCovarianceStamped;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    static constexpr std::uint64_t kU1e9 = 1e9;
    destination.header.frame_id = source.frame_id;
    destination.header.stamp.sec = source.data.stamp_nanosec / kU1e9;
    destination.header.stamp.nanosec = source.data.stamp_nanosec - destination.header.stamp.sec * kU1e9;
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(destination.pose.covariance.data()) = source.data.data.covariance;
    destination.pose.pose.position.x = source.data.data.data.translation().x();
    destination.pose.pose.position.y = source.data.data.data.translation().y();
    destination.pose.pose.position.z = source.data.data.data.translation().z();
    const Eigen::Quaterniond q(source.data.data.data.linear());
    destination.pose.pose.orientation.x = q.x();
    destination.pose.pose.orientation.y = q.y();
    destination.pose.pose.orientation.z = q.z();
    destination.pose.pose.orientation.w = q.w();
  }

  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    static constexpr std::uint64_t kU1e9 = 1e9;
    destination.frame_id = source.header.frame_id;
    destination.data.stamp_nanosec = source.header.stamp.sec * kU1e9 + source.header.stamp.nanosec;
    destination.data.data.covariance = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(source.pose.covariance.data());
    destination.data.data.data = Eigen::Isometry3d::Identity();
    destination.data.data.data.translation() = Eigen::Vector3d(
      source.pose.pose.position.x, source.pose.pose.position.y, source.pose.pose.position.z);
    destination.data.data.data.linear() = Eigen::Quaterniond(
      source.pose.pose.orientation.w,
      source.pose.pose.orientation.x,
      source.pose.pose.orientation.y,
      source.pose.pose.orientation.z).toRotationMatrix();
  }
};

namespace d2::controller::ros2::adapter
{

using PoseWithCovarianceStamped = rclcpp::TypeAdapter<
  d2::controller::WithFrameId<d2::controller::Stamped<d2::controller::WithCovariance<6, Eigen::Isometry3d>>>,
  geometry_msgs::msg::PoseWithCovarianceStamped>;

}


#endif