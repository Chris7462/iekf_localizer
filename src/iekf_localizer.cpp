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
: Node("iekf_localizer_node")
{
  rclcpp::QoS qos(10);

  gps_sub_ = this->create_subscription<kitti_msgs::msg::GeoPlanePoint>(
    "kitti/vehicle/gps_local", qos,
    std::bind(&IEKFLocalizer::gps_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "kitti/vehicle/imu_local", qos,
    std::bind(&IEKFLocalizer::imu_callback, this, std::placeholders::_1));

  freq_ = declare_parameter("freq", 40.0);
  dt_ = 1.0 / freq_;

  // Init the state
  std::vector<double> xyz = declare_parameter("init.xyz", std::vector<double>());
  std::vector<double> rpy = declare_parameter("init.rpy", std::vector<double>());
  std::vector<double> vxyz = declare_parameter("init.vxyz", std::vector<double>());
  X_ = manif::SE_2_3d(
    Eigen::Vector3d(xyz.data()),
    manif::SO3d(rpy[0], rpy[1], rpy[2]),
    Eigen::Vector3d(vxyz.data()));

  // Init the state covariance
  std::vector<double> vec_P = declare_parameter("init.P", std::vector<double>());
  P_ = Matrix9d(vec_P.data());

  // no control inputs in the beginning
  u_noisy_.setZero();
  // Init control covariance
  std::vector<double> vec_sigmas = declare_parameter("u_sigmas", std::vector<double>());
  u_sigmas_ = Array9d(vec_sigmas.data());
  Q_ = (u_sigmas_ * u_sigmas_).matrix().asDiagonal();

  // gravity
  std::vector<double> g = declare_parameter("g", std::vector<double>({0.0, 0.0, -9.80665}));
  g_ = Eigen::Vector3d(g.data());
  omega_prev_.setZero();
  alpha_prev_ = -g_;

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
    std::bind(&IEKFLocalizer::run_ekf, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void IEKFLocalizer::gps_callback(const kitti_msgs::msg::GeoPlanePoint::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  gps_buff_.push(msg);
}

void IEKFLocalizer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  alpha_prev_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  omega_prev_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
}

void IEKFLocalizer::run_ekf()
{
  RCLCPP_INFO_ONCE(get_logger(), "Running IEKF!");
  rclcpp::Time time_current = rclcpp::Node::now();

  // State estimation
  // get current state
  auto R_k = X_.rotation();
  auto v_k = X_.linearVelocity();
  auto acc_k = alpha_prev_ + R_k.transpose() * g_;

  // input control
  Eigen::Vector3d accLin = dt_ * ((R_k.transpose()) * v_k) + 0.5 * dt_ * dt_ * acc_k;
  Eigen::Vector3d gLin = (R_k.transpose()) * g_ * dt_;
  Eigen::Matrix3d accLinCross = manif::skew(accLin);
  Eigen::Matrix3d gCross = manif::skew(gLin);

  u_noisy_ << accLin, dt_ * omega_prev_, dt_ * acc_k;
  X_ = X_.plus(u_noisy_, J_x_x_, J_x_u_); // X * exp(u), with Jacobians

  // Prepare Jacobian of state-dependent control vector
  J_u_x_.setZero();
  J_u_x_.block<3, 3>(0, 3) = accLinCross;
  J_u_x_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt_;
  J_u_x_.block<3, 3>(6, 3) = gCross;
  F_ = J_x_x_ + J_x_u_ * J_u_x_;  // chain rule for system model Jacobian
  P_ = F_ * P_ * F_.transpose() + J_x_u_ * Q_ * J_x_u_.transpose();

  // Run gps update
  if (!gps_buff_.empty()) {
    mtx_.lock();
    if ((time_current - rclcpp::Time(gps_buff_.front()->header.stamp)).seconds() > 0.1) { // time sync has problem
      gps_buff_.pop();
      RCLCPP_WARN(get_logger(), "Timestamp unaligned, please check your GPS data.");
      mtx_.unlock();
    } else {
      auto msg = gps_buff_.front();
      gps_buff_.pop();
      mtx_.unlock();

      // measurement
      Eigen::Vector3d y(msg->local_coordinate.x, msg->local_coordinate.y, msg->local_coordinate.z);
      Eigen::Matrix3d R(msg->position_covariance.data());

      // expection
      Matrix3x9d J_e_x;
      Eigen::Vector3d e = X_.act(Eigen::Vector3d::Zero(), J_e_x);
      Matrix3x9d H = J_e_x;
      Eigen::Matrix3d E = H * P_ * H.transpose();

      // innovation
      Eigen::Vector3d z = y - e;  // innovation
      Eigen::Matrix3d Z = E + R;  // innovation cov

      // Mahalanobis distance
      // qchisq(0.95, df=3) = 7.814728
      double D = z.transpose() * Z.inverse() * z;
      if (D < 7.814728) {
        // Kalman gain
        Matrix9x3d K = P_ * H.transpose() * Z.inverse();

        // Correction step
        manif::SE_2_3Tangentd dx = K * z;

        // Update
        X_ = X_.plus(dx);
        P_ = P_ - K * Z * K.transpose();
      } else {
        RCLCPP_INFO(
          get_logger(),
          "The Mahalanobis dist = %f is greater than the threshold. No ESKF correction!", D);
      }
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
