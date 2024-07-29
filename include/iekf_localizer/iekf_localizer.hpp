#pragma once

// C++ header
#include <queue>
#include <mutex>
#include <memory>

// ROS header
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

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

public:
  IEKFLocalizer();
  ~IEKFLocalizer() = default;

private:
  double freq_;

  // void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buff_;
  std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gps_buff_;
  std::queue<geometry_msgs::msg::TwistStamped::SharedPtr> vel_buff_;

  std::mutex mtx_;

  // State & Error covariance
  manif::SE3d X_;
  Matrix6d P_;

  // Control
  manif::SE3Tangentd u_;
  Vector6d u_noisy_;
  Array6d u_sigmas_;
  Matrix6d U_;

  // Declare the Jacobians of the motion wrt robot and control
  manif::SE3d::Jacobian J_x, J_u;

  void run_ekf();
};

} // namespace ekf_localizer
