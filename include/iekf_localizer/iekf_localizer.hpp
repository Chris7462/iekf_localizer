#pragma once

// C++ header
#include <queue>
#include <mutex>
#include <memory>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Eigen header
#include <Eigen/Core>

// manif header
#include <manif/SE3.h>


namespace iekf_localizer
{

class IEKFLocalizer : public rclcpp::Node
{
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Array6d = Eigen::Array<double, 6, 1>;
  using Matrix3x6d = Eigen::Matrix<double, 3, 6>;
  using Matrix6x3d = Eigen::Matrix<double, 6, 3>;

public:
  IEKFLocalizer();
  ~IEKFLocalizer() = default;

private:
  double freq_;
  double dt_;

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gps_buff_;

  std::mutex mtx_;

  // State & Error covariance
  manif::SE3d X_;
  Matrix6d P_;

  // Control
  manif::SE3Tangentd u_;
  Vector6d u_noisy_;
  Array6d u_sigmas_;
  Matrix6d Q_;

  // Declare the Jacobians of the motion wrt robot and control
  manif::SE3d::Jacobian J_x_, J_u_;

  void run_ekf();
};

} // namespace iekf_localizer
