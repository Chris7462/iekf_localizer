#pragma once

// C++ header
#include <queue>
#include <mutex>
#include <memory>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Eigen header
#include <Eigen/Core>

// manif header
#include <manif/SE_2_3.h>


namespace iekf_localizer
{

class IEKFLocalizer : public rclcpp::Node
{

using Matrix9d = Eigen::Matrix<double, 9, 9>;
using Array9d = Eigen::Array<double, 9, 1>;

public:
  IEKFLocalizer();
  ~IEKFLocalizer() = default;

private:
  double freq_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buff_;
  std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gps_buff_;

  std::mutex mtx_;

  // State & Error covariance
  manif::SE_2_3d X_;
  Matrix9d P_;

  // Control
  manif::SE_2_3Tangentd u_;
  Array9d u_sigmas_;
  Matrix9d U_;

  // Gravity in the world frame
  const Eigen::Vector3d g_;

  // some value that will be used throught the program
  Eigen::Vector3d omega_prev_;
  Eigen::Vector3d alpha_prev_;

  // time
  rclcpp::Time time_prev_;
  bool init_;

  // Declare the Jacobians of the motion wrt robot and control
  manif::SE_2_3d::Jacobian F_;  // F_ = J_x_x_ + (J_x_u_ * J_u_x_)
  manif::SE_2_3d::Jacobian J_x_x_;  // d(X * exp(u)) / dX
  manif::SE_2_3d::Jacobian J_x_u_;  // d(X * exp(u)) / du
  manif::SE_2_3d::Jacobian J_u_x_;  // du / dX, since u is a state-dependent vector
  Eigen::Matrix<double, 3, 9> J_e_xi_;  // Jacobian

  void run_ekf();
};

} // namespace ekf_localizer
