// C++ header
#include <chrono>

// local header
#include "iekf_localizer/iekf_localizer.hpp"


namespace iekf_localizer
{

IEKFLocalizer::IEKFLocalizer()
: Node("iekf_localizer_node"), freq_{40.0}
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
  u_noisy_ << 5.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  u_ = u_noisy_;
  u_sigmas_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  U_ = (u_sigmas_ * u_sigmas_).matrix().asDiagonal();

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1.0 / freq_ * 1000)),
    std::bind(&IEKFLocalizer::run_ekf, this));
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
  RCLCPP_INFO(get_logger(), "Get GPS");
}

void IEKFLocalizer::vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Get Velocity");
  u_noisy_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
              msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;

  u_ = u_noisy_;
}

void IEKFLocalizer::run_ekf()
{
  RCLCPP_INFO(get_logger(), "Running IEKF!");
  rclcpp::Time time_current = rclcpp::Node::now();

  // Run velocity update
  if (!vel_buff_.empty()) {
    mtx_.lock();
    if ((time_current - rclcpp::Time(vel_buff_.front()->header.stamp)).seconds() > 0.1) {  // time sync has problem
      vel_buff_.pop();
      RCLCPP_WARN(this->get_logger(), "Timestamp unaligned, please check your Velocity data.");
      mtx_.unlock();
    } else {
      auto msg = vel_buff_.front();
      vel_buff_.pop();
      mtx_.unlock();


    //VelMeasurement z;
    //z.nu() = msg->twist.linear.x;

    //// check velocity update successful?
    //if (ekf_.update(vel_model_, z)) {
    //  ekf_.wrapStateYaw();

    //  // publish odom to base link TF
    //  tf2::Vector3 t_current(s.x(), s.y(), alt_);
    //  tf2::Quaternion q_current;
    //  q_current.setRPY(roll_, pitch_, s.theta());
    //  q_current.normalize();

    //  geometry_msgs::msg::TransformStamped odom_base_link_tf;
    //  odom_base_link_tf.header.stamp = rclcpp::Node::now();
    //  odom_base_link_tf.header.frame_id = "odom";
    //  odom_base_link_tf.child_frame_id = "base_link";
    //  odom_base_link_tf.transform.translation = tf2::toMsg(t_current);
    //  odom_base_link_tf.transform.rotation = tf2::toMsg(q_current);

    //  // send the transformation
    //  tf_broadcaster_->sendTransform(odom_base_link_tf);

    //  // save the odom->base transform
    //  tf2::fromMsg(odom_base_link_tf.transform, odom_base_link_trans_);
    //} else {
    //  RCLCPP_INFO(
    //    get_logger(), "Measurement Velocity is over the threshold. Discard this measurement.");
    //}
    }
  }


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

    //// measurement
    //Eigen::Vector3d y(msg->latitude , msg->longitude, msg->altitude);

    //Eigen::Vector3d e = X_.act(Eigen::Vector3d::Zero(), J_e_xi_);
    //RCLCPP_INFO_STREAM(get_logger(), "X tran = " << X_.translation().transpose());
    //RCLCPP_INFO_STREAM(get_logger(), "y = " << y.transpose());
    //RCLCPP_INFO_STREAM(get_logger(), "e = " << e.transpose());

    //// Innovation
    //// z = y - e;

    //// gps measurement
    //GpsMeasurement z;
    //z.x() = msg->latitude;
    //z.y() = msg->longitude;
    //alt_ = msg->altitude;

    //// use the covariance that Gps provided.
    //kalman::Covariance<GpsMeasurement> R = kalman::Covariance<GpsMeasurement>::Zero();
    //R.diagonal() << msg->position_covariance.at(0), msg->position_covariance.at(4);
    //gps_model_.setCovariance(R);

    //// check GPS update successful?
    //if (ekf_.update(gps_model_, z)) {
    //  ekf_.wrapStateYaw();
    //} else {
    //  RCLCPP_INFO(
    //    get_logger(), "Measurement GPS is over the threshold. Discard this measurement.");
    //}
    }
  }

//// publish map to odom TF
//tf2::Vector3 t_current(s.x(), s.y(), alt_);
//tf2::Quaternion q_current;
//q_current.setRPY(roll_, pitch_, s.theta());
//q_current.normalize();

//tf2::Transform map_base_link_trans(q_current, t_current);
//tf2::Transform map_odom_trans;
//map_odom_trans.mult(map_base_link_trans, odom_base_link_trans_.inverse());

//geometry_msgs::msg::TransformStamped map_odom_tf;
//map_odom_tf.header.stamp = rclcpp::Node::now();
//map_odom_tf.header.frame_id = "map";
//map_odom_tf.child_frame_id = "odom";
//map_odom_tf.transform = tf2::toMsg(map_odom_trans);

//// send the transformation
//tf_broadcaster_->sendTransform(map_odom_tf);

}

}  // namespace ekf_localizer
