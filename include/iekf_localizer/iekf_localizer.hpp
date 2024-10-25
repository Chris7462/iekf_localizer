#pragma once

// C++ header
#include <queue>
#include <mutex>
#include <memory>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

// Eigen header
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

// local header
#include <kitti_msgs/msg/geo_plane_point.hpp>

namespace iekf_localizer
{

using Matrix5d = Eigen::Matrix<double, 5, 5>; // State matrix

class IEKFLocalizer : public rclcpp::Node
{
  using Matrix9d = Eigen::Matrix<double, 9, 9>; // Error covariance
  using Array9d = Eigen::Array<double, 9, 1>;
  using Vector9d = Eigen::Matrix<double, 9, 1>;
  using Matrix3x9d = Eigen::Matrix<double, 3, 9>; // Jacobian
  using Matrix9x3d = Eigen::Matrix<double, 9, 3>; // Kalman gain

public:
  IEKFLocalizer();
  ~IEKFLocalizer() = default;

private:
  double freq_;
  double dt_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void gps_callback(const kitti_msgs::msg::GeoPlanePoint::SharedPtr msg);
  void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<kitti_msgs::msg::GeoPlanePoint>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::queue<kitti_msgs::msg::GeoPlanePoint::SharedPtr> gps_buff_;
  std::queue<geometry_msgs::msg::TwistStamped::SharedPtr> vel_buff_;

  std::mutex mtx_;

  // State & Error covariance
  Matrix5d X_;
  Matrix9d P_;

  // Control
  Array9d u_sigmas_;
  Matrix9d Q_, I_;

  // Gravity in the world frame
  Eigen::Vector3d g_;

  // some value that will be used throught the program
  Eigen::Vector3d omega_prev_;
  Eigen::Vector3d alpha_prev_;

  // Declare the Jacobians of the motion wrt robot and control
  Matrix9d F_, W_;  // F = J_f_x and W = J_f_epsilon

  Matrix3x9d H_;
  Eigen::Matrix3d V_;

  void run_ekf();

  Eigen::Matrix3d skew(const Eigen::Vector3d & u);
  Eigen::Matrix3d gamma0(const Eigen::Vector3d & phi);
  Eigen::Matrix3d gamma1(const Eigen::Vector3d & phi);
  Eigen::Matrix3d gamma2(const Eigen::Vector3d & phi);
  Matrix5d makeTwist(const Vector9d & u);
};

} // namespace iekf_localizer
