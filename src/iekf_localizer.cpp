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
  std::vector<double> xyz = declare_parameter("init.xyz", std::vector<double>());
  std::vector<double> rpy = declare_parameter("init.rpy", std::vector<double>());
  std::vector<double> vxyz = declare_parameter("init.vxyz", std::vector<double>());
  X_ = manif::SE_2_3d(
    Eigen::Vector3d(xyz.data()),
    manif::SO3d(rpy[0], rpy[1], rpy[2]),
    Eigen::Vector3d(vxyz.data()));

  // Init the state covariance. Default is right-invariant
  std::vector<double> vec_P = declare_parameter("init.P", std::vector<double>());
  P_R_ = Matrix9d(vec_P.data());

  // no control inputs in the beginning
  u_noisy_.setZero();
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
  alpha_prev_ << msg->linear_acceleration.x, 0.0, 0.0;
  omega_prev_ << 0.0, 0.0, msg->angular_velocity.z;
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
//  u_ = (u_noisy_ * dt_);

//  // State estimation
//  X_ = X_.plus(u_, J_x_, J_u_);
//  P_ = J_x_ * P_ * J_x_.transpose() + J_u_ * Q_ * J_u_.transpose();

//  // Run gps update
//  if (!gps_buff_.empty()) {
//    mtx_.lock();
//    if ((time_current - rclcpp::Time(gps_buff_.front()->header.stamp)).seconds() > 0.1) { // time sync has problem
//      gps_buff_.pop();
//      RCLCPP_WARN(get_logger(), "Timestamp unaligned, please check your GPS data.");
//      mtx_.unlock();
//    } else {
//      auto msg = gps_buff_.front();
//      gps_buff_.pop();
//      mtx_.unlock();

//      // measurement
//      Eigen::Vector3d y(msg->local_coordinate.x, msg->local_coordinate.y, msg->local_coordinate.z);
//      Eigen::Matrix3d R(msg->position_covariance.data());

//      // innovation
//      Eigen::Vector3d z = X_.inverse().act(y);  // X.inverse().act(ybar) = 0

//      // Jacobians
//      H_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
//      H_.topRightCorner<3, 1>() = Eigen::Vector3d::Zero();
//      V_ = X_.inverse().rotation();

//      // innovation covariance
//      Eigen::Matrix3d S = H_ * P_ * H_.transpose() + V_ * R * V_.transpose();

//      // Mahalanobis distance
//      // qchisq(0.95, df=3) = 7.814728
//      double D = z.transpose() * S.inverse() * z;
//      if (D < 7.814728) {
//        // Kalman gain
//        Matrix6x3d K = P_ * H_.transpose() * S.inverse();

//        // Correction step
//        manif::SE3Tangentd dx = K * z;

//        // Update
//        X_ = X_.plus(dx);
//        // P_ = (I_ - K * H_) * P_; // This will have numerial issue
//        P_ = (I_ - K * H_) * P_ * (I_ - K * H_).transpose()
//             + K * V_ * R * V_.transpose() * K.transpose();
//      } else {
//        RCLCPP_INFO(
//          get_logger(),
//          "The Mahalanobis dist = %f is greater than the threshold. No ESKF correction!", D);
//      }
//    }
//  }

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
