// C++ header
#include <chrono>

// ROS header
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// local header
#include "iekf_localizer/iekf_localizer.hpp"


namespace iekf_localizer
{

IEKFLocalizer::IEKFLocalizer()
: Node("iekf_localizer_node"), freq_{40.0}, dt_{1.0 / freq_}
{
  rclcpp::QoS qos(10);

  // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
  //   "kitti/oxts/imu_rotated", qos,
  //   std::bind(&IEKFLocalizer::imu_callback, this, std::placeholders::_1));

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "kitti/oxts/gps_shifted", qos,
    std::bind(&IEKFLocalizer::gps_callback, this, std::placeholders::_1));

  vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "kitti/oxts/gps/vel", qos,
    std::bind(&IEKFLocalizer::vel_callback, this, std::placeholders::_1));

  // Initial the state and covariance.
  X_.setIdentity();
  P_.setZero();

  // Initial control
  u_noisy_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  u_sigmas_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  U_ = (u_sigmas_ * u_sigmas_).matrix().asDiagonal();

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1.0 / freq_ * 1000)),
    std::bind(&IEKFLocalizer::run_ekf, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

// void IEKFLocalizer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(mtx_);
//   imu_buff_.push(msg);
//   RCLCPP_INFO(get_logger(), "Get IMU");
// }

void IEKFLocalizer::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  gps_buff_.push(msg);
}

void IEKFLocalizer::vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  u_noisy_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
              msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
}

void IEKFLocalizer::run_ekf()
{
  RCLCPP_INFO_ONCE(get_logger(), "Running IEKF!");
  rclcpp::Time time_current = rclcpp::Node::now();
  u_ = (u_noisy_ * dt_);

  // State estimation
  X_ = X_.rplus(u_, J_x_, J_u_);
  P_ = J_x_ * P_ * J_x_.transpose() + J_u_ * U_ * J_u_.transpose();

  // Run gps update
  if (!gps_buff_.empty()) {
    mtx_.lock();
    if ((time_current - rclcpp::Time(gps_buff_.front()->header.stamp)).seconds() > 0.1) {  // time sync has problem
      gps_buff_.pop();
      RCLCPP_WARN(get_logger(), "Timestamp unaligned, please check your GPS data.");
      mtx_.unlock();
    } else {
      auto msg = gps_buff_.front();
      gps_buff_.pop();
      mtx_.unlock();

      // measurement
      Eigen::Vector3d y(msg->latitude , msg->longitude, msg->altitude);
      Eigen::Matrix3d R;
      R.setZero();
      R.diagonal() << msg->position_covariance.at(0), msg->position_covariance.at(4), msg->position_covariance.at(8);

      // expection
      Eigen::Matrix<double, 3, 6> J_e_x;
      Eigen::Vector3d e = X_.act(Eigen::Vector3d::Zero(), J_e_x);
      Eigen::Matrix<double, 3, 6> H = J_e_x;

      //// Change the covariance from right to left
      //Matrix6d LP = X_.adj().inverse() * P_ * X_.adj().transpose().inverse();
      //Eigen::Matrix3d E = H * LP * H.transpose();
      Eigen::Matrix3d E = H * P_ * H.transpose();

      // innovation
      Eigen::Vector3d z = y - e;  // innovation
      Eigen::Matrix3d Z = E + R;  // innovation cov

      // Kalman gain
      //Eigen::Matrix<double, 6, 3> K = LP * H.transpose() * Z.inverse();
      Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * Z.inverse();

      // Correction step
      manif::SE3Tangentd dx = K * z;

      // Update
      //X_ = X_.lplus(dx);
      //LP = LP - K * Z * K.transpose();

      X_ = X_.rplus(dx);
      P_ = P_ - K * Z * K.transpose();

      //// Change the covariance from the left to right
      //P_ = X_.adj() * LP * X_.adj().transpose();
    }
  }

  // publish map to base link TF
  Eigen::Vector3d t_current(X_.translation());
  Eigen::Quaterniond q_current(X_.rotation());
  q_current.normalize();

  geometry_msgs::msg::TransformStamped map_base_link_tf;
  map_base_link_tf.header.stamp = rclcpp::Node::now();
  map_base_link_tf.header.frame_id = "map";
  map_base_link_tf.child_frame_id = "base_link";
  map_base_link_tf.transform.translation = tf2::toMsg2(t_current);
  map_base_link_tf.transform.rotation = tf2::toMsg(q_current);

  // send the transformation
  tf_broadcaster_->sendTransform(map_base_link_tf);
}

}  // namespace iekf_localizer
