#ifndef D2__CONTROLLER__ROS2__ADAPTOR__PATH_HPP_
#define D2__CONTROLLER__ROS2__ADAPTOR__PATH_HPP_

#include "Eigen/Geometry"
#include "d2/controller/stamped.hpp"
#include "d2/controller/with_frame_id.hpp"
#include "rclcpp/type_adapter.hpp"
#include "nav_msgs/msg/path.hpp"

template <>
class rclcpp::TypeAdapter<
  d2::controller::WithFrameId<d2::controller::Stamped<std::vector<d2::controller::Stamped<Eigen::Isometry3d>>>>,
  nav_msgs::msg::Path>
{
public:
  using is_specialized = std::true_type;
  using custom_type = d2::controller::WithFrameId<d2::controller::Stamped<std::vector<d2::controller::Stamped<Eigen::Isometry3d>>>>;
  using ros_message_type = nav_msgs::msg::Path;

  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    static constexpr std::uint64_t kU1e9 = 1e9;
    destination.header.frame_id = source.frame_id;
    destination.header.stamp.sec = source.data.stamp_nanosec / kU1e9;
    destination.header.stamp.nanosec = source.data.stamp_nanosec - destination.header.stamp.sec * kU1e9;
    destination.poses.clear();
    destination.poses.reserve(source.data.data.size());
    for (const auto & stamped_point : source.data.data) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp.sec = stamped_point.stamp_nanosec / kU1e9;
      pose_stamped.header.stamp.nanosec = stamped_point.stamp_nanosec - pose_stamped.header.stamp.sec * kU1e9;
      pose_stamped.header.frame_id = source.frame_id;
      pose_stamped.pose.position.x = stamped_point.data.translation().x();
      pose_stamped.pose.position.y = stamped_point.data.translation().y();
      pose_stamped.pose.position.z = stamped_point.data.translation().z();
      const Eigen::Quaterniond q(stamped_point.data.linear());
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();
      destination.poses.emplace_back(pose_stamped);
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
    destination.data.data.reserve(source.poses.size());
    for (const auto & pose_stamped : source.poses) {
      // check frame_id
      if (!pose_stamped.header.frame_id.empty() && pose_stamped.header.frame_id != source.header.frame_id) {
        static const auto kNanTf = Eigen::Isometry3d(Eigen::Matrix4d::Identity() * std::numeric_limits<double>::quiet_NaN());
        destination.data.data.emplace_back(d2::controller::Stamped<Eigen::Isometry3d>{0, kNanTf});
        continue;
      }

      d2::controller::Stamped<Eigen::Isometry3d> stamped_point;
      stamped_point.stamp_nanosec = pose_stamped.header.stamp.sec * kU1e9 + pose_stamped.header.stamp.nanosec;
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation().x() = pose_stamped.pose.position.x;
      tf.translation().y() = pose_stamped.pose.position.y;
      tf.translation().z() = pose_stamped.pose.position.z;
      const Eigen::Quaterniond q(
        pose_stamped.pose.orientation.w,
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z);
      tf.linear() = q.toRotationMatrix();
      stamped_point.data = tf;
      destination.data.data.emplace_back(stamped_point);
    }
  }
};

namespace d2::controller::ros2::adapter
{

using Path = rclcpp::TypeAdapter<
  d2::controller::WithFrameId<d2::controller::Stamped<std::vector<d2::controller::Stamped<Eigen::Isometry3d>>>>,
  nav_msgs::msg::Path>;

}


#endif