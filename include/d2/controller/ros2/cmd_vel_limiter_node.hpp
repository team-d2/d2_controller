#ifndef D2__CONTROLLER__ROS2__CMD_VEL_LIMITER_NODE_HPP_
#define D2__CONTROLLER__ROS2__CMD_VEL_LIMITER_NODE_HPP_

#include <memory>
#include <string>
#include <variant>

#include "visibility.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/node.hpp"

namespace d2::controller::ros2
{

class CmdVelLimiterNode : public rclcpp::Node
{
  using TwistMsg = geometry_msgs::msg::Twist;
  using AccelMsg = geometry_msgs::msg::Accel;

public:
  static constexpr auto kDefaultNodeName = "d2_cmd_vel_limiter";

  D2__CONTROLLER__ROS2_PUBLIC
  inline CmdVelLimiterNode(
    const std::string & node_name, const std::string node_namespace,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, options),
    timeout_duration_(rclcpp::Duration::from_seconds(
          this->declare_parameter<double>("cmd_vel_timeout", 10.0))),
    vel_limit_linear_vec_(tf2::Vector3(
      this->declare_parameter<double>("vel_limit.linear.x", 1.0),
      this->declare_parameter<double>("vel_limit.linear.y", 1.0),
      this->declare_parameter<double>("vel_limit.linear.z", 1.0)
    )),
    vel_limit_angular_vec_(tf2::Vector3(
      this->declare_parameter<double>("vel_limit.angular.x", 1.0),
      this->declare_parameter<double>("vel_limit.angular.y", 1.0),
      this->declare_parameter<double>("vel_limit.angular.z", 1.0)
    )),
    accel_limit_linear_vec_(tf2::Vector3(
      this->declare_parameter<double>("accel_limit.linear.x", 1.0),
      this->declare_parameter<double>("accel_limit.linear.y", 1.0),
      this->declare_parameter<double>("accel_limit.linear.z", 1.0)
    )),
    accel_limit_angular_vec_(tf2::Vector3(
      this->declare_parameter<double>("accel_limit.angular.x", 1.0),
      this->declare_parameter<double>("accel_limit.angular.y", 1.0),
      this->declare_parameter<double>("accel_limit.angular.z", 1.0)
    )),
    last_vel_time_(this->now()),
    last_vel_linear_vec_(0.0, 0.0, 0.0),
    last_vel_angular_vec_(0.0, 0.0, 0.0),
    cmd_vel_pub_(this->create_publisher<TwistMsg>("cmd_vel", 10)),
    vel_limit_vec_sub_(
      this->create_subscription<TwistMsg>(
        "vel_limit", 10, [this](TwistMsg::ConstSharedPtr msg) {
          this->set_vel_limit(std::move(msg));
        })),
    accel_limit_sub_(
      this->create_subscription<AccelMsg>(
        "accel_limit", 10, [this](AccelMsg::ConstSharedPtr msg) {
          this->set_accel_limit(std::move(msg));
        }))
  {
    const auto rate = this->declare_parameter<double>("publish_rate", 60.0);
    if (rate <= 0.0) {
      cmd_vel_nav_sub_ =
        this->create_subscription<TwistMsg>(
          "cmd_vel_nav", 10, [this](TwistMsg::ConstSharedPtr msg) {
            this->limit_cmd_vel(std::move(msg));
          });
    }
    else {
      cmd_vel_nav_sub_ =
        this->create_subscription<TwistMsg>(
          "cmd_vel_nav", 10, [this](TwistMsg::ConstSharedPtr msg) {
            this->initialize_cmd_vel_nav(std::move(msg));
          });
    }
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline CmdVelLimiterNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CmdVelLimiterNode(node_name, "", options)
  {
  }

  D2__CONTROLLER__ROS2_PUBLIC
  explicit inline CmdVelLimiterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CmdVelLimiterNode(kDefaultNodeName, "", options)
  {
  }

private:
  static tf2::Vector3 limit_vel_vec(const tf2::Vector3 & cmd_vel_nav_vec, const tf2::Vector3 & vel_limit_vec)
  {
    const auto vnx2 = vel_limit_vec.x() * vel_limit_vec.x();
    const auto vny2 = vel_limit_vec.y() * vel_limit_vec.y();
    const auto vnz2 = vel_limit_vec.z() * vel_limit_vec.z();
    const auto lx2 = cmd_vel_nav_vec.x() * cmd_vel_nav_vec.x();
    const auto ly2 = cmd_vel_nav_vec.y() * cmd_vel_nav_vec.y();
    const auto lz2 = cmd_vel_nav_vec.z() * cmd_vel_nav_vec.z();
    const auto xrate2 = vnx2 == 0 ? 0.0 : vnx2 / lx2;
    const auto yrate2 = vny2 == 0 ? 0.0 : vny2 / ly2;
    const auto zrate2 = vnz2 == 0 ? 0.0 : vnz2 / lz2;
    const auto rate2 = xrate2 + yrate2 + zrate2;

    return rate2 > 1.0 ?
      cmd_vel_nav_vec :
      cmd_vel_nav_vec / std::sqrt(rate2);
  }

  static tf2::Vector3 limit_vel_vec(const tf2::Vector3 & cmd_vel_nav_vec, const tf2::Vector3 & vel_limit_vec, const tf2::Vector3 & accel_limit_vec, const double duration, const tf2::Vector3 & vel_last)
  {
    // limit vel
    const auto cmd_vel_vel_limited_vec = limit_vel_vec(cmd_vel_nav_vec, vel_limit_vec);

    // limit accel
    const auto vel_delta_limit_vec = accel_limit_vec * duration;
    const auto vel_delta_vel_limited_vec = cmd_vel_vel_limited_vec - vel_last;
    const auto vel_delta_accel_limited_vec = limit_vel_vec(vel_delta_vel_limited_vec, vel_delta_limit_vec);

    return vel_last + vel_delta_accel_limited_vec;
  }

  void limit_cmd_vel(TwistMsg::ConstSharedPtr cmd_vel_nav_msg)
  {
    // get current time
    const auto now = this->now();
    const auto duration = (now - last_vel_time_).seconds();

    // convert cmd_vel_nav to vector
    tf2::Vector3 cmd_vel_nav_linear_vec, cmd_vel_nav_angular_vec;
    tf2::fromMsg(cmd_vel_nav_msg->linear, cmd_vel_nav_linear_vec);
    tf2::fromMsg(cmd_vel_nav_msg->angular, cmd_vel_nav_angular_vec);

    // limit velocity
    const auto limited_linear_vec = limit_vel_vec(
      cmd_vel_nav_linear_vec, vel_limit_linear_vec_, accel_limit_linear_vec_, duration, last_vel_linear_vec_);
    const auto limited_angular_vec = limit_vel_vec(
      cmd_vel_nav_angular_vec, vel_limit_angular_vec_, accel_limit_angular_vec_, duration, last_vel_angular_vec_);

    // publish limited cmd_vel
    auto cmd_vel_msg = std::make_unique<TwistMsg>();
    cmd_vel_msg->linear = tf2::toMsg(limited_linear_vec);
    cmd_vel_msg->angular = tf2::toMsg(limited_angular_vec);
    cmd_vel_pub_->publish(std::move(cmd_vel_msg));

    // update last velocity
    last_vel_time_ = now;
    last_vel_linear_vec_ = limited_linear_vec;
    last_vel_angular_vec_ = limited_angular_vec;
  }

  void limit_cmd_vel()
  {
    const auto now = this->now();
    if (now > timeout_time_) {
      publish_timer_.reset();
      cmd_vel_nav_sub_ = this->create_subscription<TwistMsg>(
        "cmd_vel_nav", 10, [this](TwistMsg::ConstSharedPtr msg) {
          this->initialize_cmd_vel_nav(std::move(msg));
        });
      return;
    }
    this->limit_cmd_vel(cmd_vel_nav_msg_);
  }

  void set_cmd_vel_nav(TwistMsg::ConstSharedPtr cmd_vel_nav_msg)
  {
    const auto now = this->now();
    cmd_vel_nav_msg_ = std::move(cmd_vel_nav_msg);
    timeout_time_ = now + timeout_duration_;
  }

  void set_vel_limit(TwistMsg::ConstSharedPtr vel_limit_msg)
  {
    tf2::fromMsg(vel_limit_msg->linear, vel_limit_linear_vec_);
    tf2::fromMsg(vel_limit_msg->angular, vel_limit_angular_vec_);
  }

  void set_accel_limit(AccelMsg::ConstSharedPtr accel_limit_msg)
  {
    tf2::fromMsg(accel_limit_msg->linear, accel_limit_linear_vec_);
    tf2::fromMsg(accel_limit_msg->angular, accel_limit_angular_vec_);
  }

  void initialize_cmd_vel_nav(TwistMsg::ConstSharedPtr cmd_vel_nav_msg)
  {
    this->set_cmd_vel_nav(std::move(cmd_vel_nav_msg));
    cmd_vel_nav_sub_ = this->create_subscription<TwistMsg>(
      "cmd_vel_nav", 10, [this](TwistMsg::ConstSharedPtr msg) {
        this->set_cmd_vel_nav(std::move(msg));
      });
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / this->get_parameter("publish_rate").as_double()),
      [this]() { this->limit_cmd_vel(); });
  }

  // parameter
  rclcpp::Duration timeout_duration_;

  // msg
  TwistMsg::ConstSharedPtr cmd_vel_nav_msg_;
  rclcpp::Time timeout_time_;

  // limit
  tf2::Vector3 vel_limit_linear_vec_, vel_limit_angular_vec_;
  tf2::Vector3 accel_limit_linear_vec_, accel_limit_angular_vec_;

  // last cmd_vel
  rclcpp::Time last_vel_time_;
  tf2::Vector3 last_vel_linear_vec_, last_vel_angular_vec_;

  // publisher
  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_pub_;

  // subscription
  rclcpp::Subscription<TwistMsg>::SharedPtr cmd_vel_nav_sub_;
  rclcpp::Subscription<TwistMsg>::SharedPtr vel_limit_vec_sub_;
  rclcpp::Subscription<AccelMsg>::SharedPtr accel_limit_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace d2__controller::ros2

#endif  // D2__CONTROLLER__ROS2__CMD_VEL_LIMITER_NODE_HPP_
