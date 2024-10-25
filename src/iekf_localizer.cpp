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

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "kitti/vehicle/imu_local", qos,
    std::bind(&IEKFLocalizer::imu_callback, this, std::placeholders::_1));

  gps_sub_ = this->create_subscription<kitti_msgs::msg::GeoPlanePoint>(
    "kitti/vehicle/gps_local", qos,
    std::bind(&IEKFLocalizer::gps_callback, this, std::placeholders::_1));

  vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "kitti/vehicle/velocity", qos,
    std::bind(&IEKFLocalizer::vel_callback, this, std::placeholders::_1));

  freq_ = declare_parameter("freq", 40.0);
  dt_ = 1.0 / freq_;

  // Init the state
  std::vector<double> rpy = declare_parameter("init.rpy", std::vector<double>());
  std::vector<double> vxyz = declare_parameter("init.vxyz", std::vector<double>());
  std::vector<double> xyz = declare_parameter("init.xyz", std::vector<double>());

  X_.setIdentity();
  X_.block<3, 3>(0, 0) = Eigen::Quaterniond(
    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ())).matrix();
  X_.block<3, 1>(0, 3) = Eigen::Vector3d(vxyz.data());
  X_.block<3, 1>(0, 4) = Eigen::Vector3d(xyz.data());

  // Init the state covariance
  std::vector<double> vec_P = declare_parameter("init.P", std::vector<double>());
  P_ = Matrix9d(vec_P.data());

  // Init control covariance
  std::vector<double> vec_sigmas = declare_parameter("u_sigmas", std::vector<double>());
  u_sigmas_ = Array9d(vec_sigmas.data());
  Q_ = (u_sigmas_ * u_sigmas_).matrix().asDiagonal();

  I_.setIdentity();

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

void IEKFLocalizer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  alpha_prev_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  omega_prev_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
}

void IEKFLocalizer::gps_callback(const kitti_msgs::msg::GeoPlanePoint::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  gps_buff_.push(msg);
}

void IEKFLocalizer::vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  vel_buff_.push(msg);
}

void IEKFLocalizer::run_ekf()
{
  RCLCPP_INFO_ONCE(get_logger(), "Running IEKF!");
  rclcpp::Time time_current = rclcpp::Node::now();

  // State estimation
  // get current state
  auto R_k = X_.block<3, 3>(0, 0);
  auto v_k = X_.block<3, 1>(0, 3);
  auto p_k = X_.block<3, 1>(0, 4);

  X_.block<3, 3>(0, 0) = R_k * gamma0(omega_prev_ * dt_);
  X_.block<3, 1>(0, 3) = v_k + R_k * gamma1(omega_prev_ * dt_) * alpha_prev_ * dt_ + g_ * dt_;
  X_.block<3, 1>(0, 4) = p_k + v_k * dt_ + R_k * gamma2(omega_prev_ * dt_) * alpha_prev_ * dt_ * dt_  + 0.5 * g_ * dt_ * dt_;

  // Prepare Jacobian of state-dependent control vector
  Matrix9d A = Matrix9d::Zero();
  A.block<3, 3>(0, 0) = -skew(omega_prev_);
  A.block<3, 3>(3, 0) = -skew(alpha_prev_);
  A.block<3, 3>(3, 3) = -skew(omega_prev_);
  A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
  A.block<3, 3>(6, 6) = -skew(omega_prev_);

  // Left invariant Jacobians
  F_ = (A * dt_).exp();
  W_.setIdentity();

  P_ = F_ * P_ * F_.transpose() + W_ * Q_ * W_.transpose();

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

      // innovation
      auto X_inv = X_.inverse();
      Eigen::Vector3d z = X_inv.block<3, 3>(0, 0) * y + X_inv.block<3, 1>(0, 4);

      // Left Jacobians
      H_.setZero();
      H_.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity();
      V_ = X_inv.block<3, 3>(0, 0);

      // innovation covariance
      Eigen::Matrix3d S = H_ * P_ * H_.transpose() + V_ * R * V_.transpose();

      // Mahalanobis distance
      // qchisq(0.95, df=3) = 7.814728
      double D = z.transpose() * S.inverse() * z;
      if (D < 7.814728) {
        // Kalman gain
        Matrix9x3d K = P_ * H_.transpose() * S.inverse();

        // Correction step
        Vector9d dx = K * z;
        Matrix5d xi = makeTwist(dx);

        // Update
        X_ = X_ * xi.exp();
        // P_ = (I_ - K * H_) * P_; // This will have numerial issue
        P_ = (I_ - K * H_) * P_ * (I_ - K * H_).transpose()
             + K * V_ * R * V_.transpose() * K.transpose();
      } else {
        RCLCPP_INFO(
          get_logger(),
          "The Mahalanobis dist = %f is greater than the threshold. No ESKF correction!", D);
      }
    }
  }

  // publish map to base link TF
  Eigen::Vector3d t_current(X_.block<3, 1>(0, 4));
  Eigen::Quaterniond q_current(X_.block<3, 3>(0, 0));
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

Eigen::Matrix3d IEKFLocalizer::skew(const Eigen::Vector3d & u)
{
  return (Eigen::Matrix3d() <<
    0.0, -u(2), u(1),
    u(2), 0.0, -u(0),
    -u(1), u(0), 0.0).finished();
}


Eigen::Matrix3d IEKFLocalizer::gamma0(const Eigen::Vector3d & phi)
{
  const double norm_phi = phi.norm();
  if (norm_phi > 1e-8) {
    auto skew_phi = skew(phi);
    return (Eigen::Matrix3d::Identity() + sin(norm_phi) / norm_phi * skew_phi +
      (1 - cos(norm_phi)) / pow(norm_phi, 2) * skew_phi * skew_phi);
  }
  return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d IEKFLocalizer::gamma1(const Eigen::Vector3d & phi)
{
  const double norm_phi = phi.norm();
  if (norm_phi > 1e-8) {
    auto skew_phi = skew(phi);
    return (Eigen::Matrix3d::Identity() + (1 - cos(norm_phi)) / pow(norm_phi, 2) * skew_phi +
      (norm_phi - sin(norm_phi)) / pow(norm_phi, 3) * skew_phi * skew_phi);
  }
  return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d IEKFLocalizer::gamma2(const Eigen::Vector3d & phi)
{
  const double norm_phi = phi.norm();
  if (norm_phi > 1e-8) {
    auto skew_phi = skew(phi);
    return (0.5 * Eigen::Matrix3d::Identity() +
      (norm_phi - sin(norm_phi)) / pow(norm_phi, 3) * skew_phi +
      (pow(norm_phi, 2) + 2 * cos(norm_phi) - 2) / (2 * pow(norm_phi, 4)) * skew_phi * skew_phi);
  }
  return 0.5 * Eigen::Matrix3d::Identity();
}

Matrix5d IEKFLocalizer::makeTwist(const Vector9d & u)
{
  Matrix5d twist = Matrix5d::Zero();
  twist.block<3, 3>(0, 0) = skew(u.block<3, 1>(0, 0));
  twist.block<3, 1>(0, 3) = u.block<3, 1>(3, 0);
  twist.block<3, 1>(0, 4) = u.block<3, 1>(6, 0);
  return twist;
}

}  // namespace iekf_localizer
