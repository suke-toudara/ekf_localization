#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <ekf_localization/ekf_localization_component.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using namespace Eigen;

namespace ekf_localization
{
ExtendedKalmanFilter::ExtendedKalmanFilter(const rclcpp::NodeOptions & options)
: Node("extended_kalman_filter", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("map_frame_id", "map");
  declare_parameter("robot_frame_id", "base_link");
  declare_parameter("broadcast_transform", true);
  
  get_parameter("map_frame_id", map_frame_id_);
  get_parameter("robot_frame_id", robot_frame_id_);
  get_parameter("broadcast_transform", broadcast_transform_);

  declare_parameter("var_imu", std::vector<double>{0.1, 0.1, 0.1});
  declare_parameter("var_wheel_odom", std::vector<double>{0.1, 0.1, 0.1});
  declare_parameter("var_ndt_odom", std::vector<double>{0.1, 0.1, 0.1});
  declare_parameter("ekf_frequency", 10.0);        // EKFの更新周波数 (Hz)
  declare_parameter("ndt_timeout_duration", 1.0);  // NDTタイムアウト時間 (秒)
  get_parameter("var_imu", var_imu_);
  get_parameter("var_wheel_odom", var_wheel_odom_);
  get_parameter("var_ndt_odom", var_ndt_odom_);
  get_parameter("ekf_frequency", ekf_frequency_);
  get_parameter("ndt_timeout_duration", ndt_timeout_duration_);

  // パラメータベクトルサイズの検証
  if (var_imu_.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "var_imu must have exactly 3 elements [x, y, theta]");
    var_imu_ = {0.1, 0.1, 0.1};
  }
  if (var_wheel_odom_.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "var_wheel_odom must have exactly 3 elements [x, y, theta]");
    var_wheel_odom_ = {0.1, 0.1, 0.1};
  }
  if (var_ndt_odom_.size() != 3) {
    RCLCPP_ERROR(this->get_logger(), "var_ndt_odom must have exactly 3 elements [x, y, theta]");
    var_ndt_odom_ = {0.1, 0.1, 0.1};
  }

  // パラメータ値のログ出力
  RCLCPP_INFO(this->get_logger(), "IMU noise variance: [%.3f, %.3f, %.3f]", 
              var_imu_[0], var_imu_[1], var_imu_[2]);
  RCLCPP_INFO(this->get_logger(), "Wheel odom noise variance: [%.3f, %.3f, %.3f]", 
              var_wheel_odom_[0], var_wheel_odom_[1], var_wheel_odom_[2]);
  RCLCPP_INFO(this->get_logger(), "NDT odom noise variance: [%.3f, %.3f, %.3f]", 
              var_ndt_odom_[0], var_ndt_odom_[1], var_ndt_odom_[2]);

  init();
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  qos.best_effort(); 
  
  // publisher
  ekf_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);

  // subscriber
  wheel_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "wheel_odom", 10, [this](const nav_msgs::msg::Odometry & msg) {
      wheel_odom_callback(msg);
      });

  ndt_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "ndt_odom", 10, [this](const nav_msgs::msg::Odometry & msg) {
      ndt_odom_callback(msg);
      });

  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, [this](const sensor_msgs::msg::Imu & msg) {
      imu_callback(msg);
      });

  initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
      initial_pose_callback(msg);
      });

  // タイマーの設定
  auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / ekf_frequency_));
  ekf_timer_ = this->create_wall_timer(timer_period, std::bind(&ExtendedKalmanFilter::ekf_timer_callback, this));

  // データフラグの初期化
  has_odom_data_ = false;
  has_imu_data_ = false;
  
  P_(0, 0) = 0.1;  
  P_(1, 1) = 0.1;  
  P_(2, 2) = 0.1;  
  
  // プロセスノイズ共分散行列(車輪odomのノイズ)
  Q_wheel_(0, 0) = var_wheel_odom_[0];  // Process noise for x
  Q_wheel_(1, 1) = var_wheel_odom_[1];  // Process noise for y
  Q_wheel_(2, 2) = var_wheel_odom_[2];  // Process noise for theta
  
  // 観測ノイズ共分散行列(NDT)
  R_ndt_(0, 0) = var_ndt_odom_[0];   // Observation noise for x
  R_ndt_(1, 1) = var_ndt_odom_[1];   // Observation noise for y
  R_ndt_(2, 2) = var_ndt_odom_[2];   // Observation noise for theta

  // imuの観測ノイズ共分散行列
  R_imu_(0, 0) = var_imu_[0];   // Observation noise for x
  R_imu_(1, 1) = var_imu_[1];   // Observation noise for y
  R_imu_(2, 2) = var_imu_[2];   // Observation noise for theta
}


void ExtendedKalmanFilter::init()
{
  init(0.0, 0.0, 0.0);  
}

void ExtendedKalmanFilter::init(double x, double y, double theta)
{
  X << x, y, theta;
  prev_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "initialized at position: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
}


void ExtendedKalmanFilter::wheel_odom_callback(const nav_msgs::msg::Odometry & msg)
{
  try {
    nav_msgs::msg::Odometry transformed_msg;
    geometry_msgs::msg::TwistStamped twist_in, twist_out;
    geometry_msgs::msg::PoseStamped pose_in, pose_out;

    // Twist情報の変換
    twist_in.header = msg.header;
    twist_in.twist = msg.twist.twist;
    
    // Pose情報の変換
    pose_in.header = msg.header;
    pose_in.pose = msg.pose.pose;

    tf2::TimePoint time_point = tf2::TimePoint(
      std::chrono::seconds(msg.header.stamp.sec) +
      std::chrono::nanoseconds(msg.header.stamp.nanosec));
    const geometry_msgs::msg::TransformStamped transform =
      tfbuffer_.lookupTransform(
      robot_frame_id_,
      msg.header.frame_id,
      time_point);
    
    tf2::doTransform(twist_in, twist_out, transform);
    tf2::doTransform(pose_in, pose_out, transform);

    transformed_msg.header.stamp = msg.header.stamp;
    transformed_msg.header.frame_id = robot_frame_id_;
    transformed_msg.child_frame_id = msg.child_frame_id;
    transformed_msg.pose.pose = pose_out.pose;
    transformed_msg.pose.covariance = msg.pose.covariance;
    transformed_msg.twist.twist = twist_out.twist;
    transformed_msg.twist.covariance = msg.twist.covariance;

    latest_wheel_odom_msg_ = transformed_msg;
    if (!wheel_odom_received_) wheel_odom_received_ = true;
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(this->get_logger(), "Wheel odom transform error: %s", e.what());
    return;
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "Wheel odom runtime error: %s", e.what());
    return;
  }
}

void ExtendedKalmanFilter::ndt_odom_callback(const nav_msgs::msg::Odometry & msg)
{
  try {
    nav_msgs::msg::Odometry transformed_msg;
    geometry_msgs::msg::TwistStamped twist_in, twist_out;
    geometry_msgs::msg::PoseStamped pose_in, pose_out;

    // Twist情報の変換
    twist_in.header = msg.header;
    twist_in.twist = msg.twist.twist;
    
    // Pose情報の変換
    pose_in.header = msg.header;
    pose_in.pose = msg.pose.pose;

    tf2::TimePoint time_point = tf2::TimePoint(
      std::chrono::seconds(msg.header.stamp.sec) +
      std::chrono::nanoseconds(msg.header.stamp.nanosec));
    const geometry_msgs::msg::TransformStamped transform =
      tfbuffer_.lookupTransform(
      robot_frame_id_,
      msg.header.frame_id,
      time_point);
    
    tf2::doTransform(twist_in, twist_out, transform);
    tf2::doTransform(pose_in, pose_out, transform);

    transformed_msg.header.stamp = msg.header.stamp;
    transformed_msg.header.frame_id = robot_frame_id_;
    transformed_msg.child_frame_id = msg.child_frame_id;
    transformed_msg.pose.pose = pose_out.pose;
    transformed_msg.pose.covariance = msg.pose.covariance;
    transformed_msg.twist.twist = twist_out.twist;
    transformed_msg.twist.covariance = msg.twist.covariance;

    latest_ndt_odom_msg_ = transformed_msg;
    if (!ndt_odom_received_) ndt_odom_received_ = true;
    
    // NDTデータ受信時刻を更新
    last_ndt_time_ = this->now();
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(this->get_logger(), "NDT odom transform error: %s", e.what());
    return;
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "NDT odom runtime error: %s", e.what());
    return;
  }
}

void ExtendedKalmanFilter::imu_callback(const sensor_msgs::msg::Imu & msg)
{
  try {
    sensor_msgs::msg::Imu transformed_msg;
    geometry_msgs::msg::Vector3Stamped acc_in, acc_out, w_in, w_out;
    geometry_msgs::msg::QuaternionStamped orientation_in, orientation_out;

    acc_in.vector.x = msg.linear_acceleration.x;
    acc_in.vector.y = msg.linear_acceleration.y;
    acc_in.vector.z = msg.linear_acceleration.z;
    w_in.vector.x = msg.angular_velocity.x;
    w_in.vector.y = msg.angular_velocity.y;
    w_in.vector.z = msg.angular_velocity.z;
    orientation_in.quaternion = msg.orientation; 
    orientation_in.header = msg.header; 

    tf2::TimePoint time_point = tf2::TimePoint(
      std::chrono::seconds(msg.header.stamp.sec) +
      std::chrono::nanoseconds(msg.header.stamp.nanosec));
    const geometry_msgs::msg::TransformStamped transform =
      tfbuffer_.lookupTransform(
      robot_frame_id_,
      msg.header.frame_id,
      time_point);
    tf2::doTransform(acc_in, acc_out, transform);
    tf2::doTransform(w_in, w_out, transform);
    tf2::doTransform(orientation_in, orientation_out, transform);

    transformed_msg.header.stamp = msg.header.stamp;
    transformed_msg.angular_velocity.x = w_out.vector.x;
    transformed_msg.angular_velocity.y = w_out.vector.y;
    transformed_msg.angular_velocity.z = w_out.vector.z;
    transformed_msg.linear_acceleration.x = acc_out.vector.x;
    transformed_msg.linear_acceleration.y = acc_out.vector.y;
    transformed_msg.linear_acceleration.z = acc_out.vector.z; 
    transformed_msg.orientation = orientation_out.quaternion;

    latest_imu_msg_ = transformed_msg;
    if (!imu_received_) imu_received_ = true;
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return;
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return;
  }
}

void ExtendedKalmanFilter::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  tf2::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  init(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw);  
}

void ExtendedKalmanFilter::ekf_timer_callback()
{
  if (!wheel_odom_received_ || !ndt_odom_received_ || !imu_received_) {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for all sensor data to be initialized...");
    return;
  }

  Pre_X = X;
  rclcpp::Duration duration = this->now() - prev_time_;
  double dt = duration.seconds();
  predictUpdate(dt); 
  measurementUpdate(latest_imu_msg_, dt);

  wheel_odom_received_ = false;
  ndt_odom_received_ = false;
  imu_received_ = false;
}

void ExtendedKalmanFilter::predictUpdate(double dt)
{
  // 予測ステップ
  /*
  ------------------------------
  1.運動方程式　　 
  2.ヤコビアン計算 
  3.事前誤差共分散行列の推定 
  ------------------------------
  F:ヤコビアン(運動方程式の)
  P_:事前誤差共分散行列
  Q_wheel_:プロセスノイズ共分散行列
  ------------------------------
  */

  //1.運動モデルによる予測(事前推定値)
  float v = latest_odom_msg_.twist.twist.linear.x;
  double omega = latest_odom_msg_.twist.twist.angular.z;
  double pre_theta = Pre_X(2);
  
  // 運動モデルによる状態予測（プロセスノイズを含む）
  X(0) = Pre_X(0) + v * cos(pre_theta  + omega * dt /2) * dt;  // x位置の予測
  X(1) = Pre_X(1) + v * sin(pre_theta  + omega * dt /2) * dt;  // y位置の予測  
  X(2) = Pre_X(2) + omega * dt;                   // 姿勢角の予測
  
  // 角度を-πからπの範囲に正規化
  while (X(2) > M_PI) X(2) -= 2.0 * M_PI;
  while (X(2) < -M_PI) X(2) += 2.0 * M_PI;

  //2.ヤコビアン計算（運動モデルの線形化）
  Matrix3d F;
  F << 1, 0, -v*sin(pre_theta + omega * dt /2)*dt ,
       0, 1,  v*cos(pre_theta + omega * dt /2)*dt ,
       0, 0, 1;

  //3.事前誤差共分散行列の推定
  P_ = F * P_ * F.transpose() + Q_wheel_;
}

void ExtendedKalmanFilter::measurementUpdate(double dt)
{   
  /*
  Update : 観測後の信念分布の更新
  1.観測ヤコビアンの計算 :
  2.
  3.カルマンゲインの計算 :
  4.観測予測値の計算
  5.共分散の更新　 :
  */

  //ndtを使うかどうかの判定
  rclcpp::Duration time_since_last_ndt = this->now() - last_ndt_time_;
  bool use_ndt = time_since_last_ndt.seconds() <= ndt_timeout_duration_;  
  if (use_ndt) {
    update_with_ndt(double dt);
  } else {
    update_with_imu(double dt);
  }

  // publish pose_ekf
  geometry_msgs::msg::PoseWithCovarianceStamped pose_ekf;
  pose_ekf.header.stamp = imutimestamp;
  pose_ekf.header.frame_id = map_frame_id_;
  pose_ekf.pose.pose.position.x = X(0);
  pose_ekf.pose.pose.position.y = X(1);
  pose_ekf.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, X(2)); 
  q.normalize(); 

  pose_ekf.pose.pose.orientation.w = q.w();
  pose_ekf.pose.pose.orientation.x = q.x();
  pose_ekf.pose.pose.orientation.y = q.y();
  pose_ekf.pose.pose.orientation.z = q.z();
  // pose_ekf.pose.covariance = {
  //   P_(0, 0), P_(0, 1), 0, 0, 0, P_(0, 2),
  //   P_(1, 0), P_(1, 1), 0, 0, 0, P_(1, 2),
  //   0,       0,       0, 0, 0, 0,
  //   0,       0,       0, 0, 0, 0,
  //   0,       0,       0, 0, 0, 0,
  //   P_(2, 0), P_(2, 1), 0, 0, 0, P_(2, 2),
  // };
  ekf_pose_publisher_->publish(pose_ekf);

  if (broadcast_transform_) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.frame_id = map_frame_id_;
      transform_stamped.header.stamp = pose_ekf.header.stamp;
      transform_stamped.child_frame_id = robot_frame_id_;
      transform_stamped.transform.translation.x = pose_ekf.pose.pose.position.x;
      transform_stamped.transform.translation.y = pose_ekf.pose.pose.position.y;
      transform_stamped.transform.translation.z = pose_ekf.pose.pose.position.z;
      transform_stamped.transform.rotation = pose_ekf.pose.pose.orientation;
      broadcaster_.sendTransform(transform_stamped);
  }
}


void ExtendedKalmanFilter::update_with_ndt(double dt)
{ 
  double ndt_dt = latest_ndt_msg_.header.stamp.sec + latest_ndt_msg_.header.stamp.nanosec * 1e-9
                 - previous_ndt_msg_.header.stamp.sec - previous_ndt_msg_.header.stamp.nanosec * 1e-9;
  double v = latest_ndt_msg_.pose.pose.position.x - previous_ndt_msg_.pose.pose.position.x / ndt_dt;
  double omega = latest_ndt_msg_.pose.pose.orientation.z - previous_ndt_msg_.pose.pose.orientation.z / ndt_dt;
  double pre_theta = Pre_X(2);
  previous_ndt_msg_ = latest_ndt_msg_;

  //1.観測予測値の計算
  Z(0) = Pre_X(0) + v * cos(pre_theta  + omega * dt /2) * dt;  
  Z(1) = Pre_X(1) + v * sin(pre_theta  + omega * dt /2) * dt;   
  Z(2) = Pre_X(2) + omega * dt;

  //2.観測ヤコビアンの計算
  H_ << 1, 0, -v*sin(pre_theta + omega * dt /2)*dt ,
        0, 1,  v*cos(pre_theta + omega * dt /2)*dt ,
        0, 0, 1;
  
  //3.観測予測値の計算
  Eigen::MatrixXd h = Eigen::MatrixXd::Identity(3,3);
  Eigen::VectorXd Y = Z - h * X;

  //3.カルマンゲインの計算
  MatrixXd S = H_ * P_ * H_.transpose() + R_ndt_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //3.観測予測値の計算
  X = X + K * Y;
  P_ = (Matrix3d::Identity() - K * H_) * P_;


}

void ExtendedKalmanFilter::update_with_imu(double dt)
{
  double v = latest_imu_msg_.linear_acceleration.x;
  double omega = latest_imu_msg_.angular_velocity.z;
  double pre_theta = Pre_X(2);
 
  //1.観測予測値の計算
  Z(0) = Pre_X(0) + v * cos(pre_theta  + omega * dt /2) * dt;  
  Z(1) = Pre_X(1) + v * sin(pre_theta  + omega * dt /2) * dt;   
  Z(2) = Pre_X(2) + omega * dt;                  

  //2.観測ヤコビアンの計算
  H_ << 1, 0, -v*sin(pre_theta + omega * dt /2)*dt ,
        0, 1,  v*cos(pre_theta + omega * dt /2)*dt ,
        0, 0, 1;

  //3.観測予測値の計算
  Eigen::MatrixXd h = Eigen::MatrixXd::Identity(3,3);
  Eigen::VectorXd Y = Z - h * X;

  //3.カルマンゲインの計算
  MatrixXd S = H_ * P_ * H_.transpose() + R_imu_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  //3.観測予測値の計算
  X = X + K * Y;
  P_ = (Matrix3d::Identity() - K * H_) * P_;
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ekf_localization::ExtendedKalmanFilter)


