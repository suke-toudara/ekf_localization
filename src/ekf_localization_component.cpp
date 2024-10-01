#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <ekf_localization/ekf_localization_component.hpp>

using namespace std::chrono_literals;
using namespace Eigen;

namespace ekf_localization
{
ExtendedKalmanFilter::ExtendedKalmanFilter(const rclcpp::NodeOptions & options)
    : Node("extended_kalman_filter", options)
{
  declare_parameter("reference_frame_id", "map");
  declare_parameter("robot_frame_id", "base_link");
  declare_parameter("initial_pose_topic","/initial_pose");
  declare_parameter("imu_topic","/imu");
  declare_parameter("odom_topic","/odom");
  get_parameter("reference_frame_id", reference_frame_id_);
  get_parameter("robot_frame_id", robot_frame_id_);
  get_parameter("initial_pose_topic", initial_pose_topic_);
  get_parameter("imu_topic", imu_topic_);
  get_parameter("odom_topic", odom_topic_);

  declare_parameter("pub_period", 10);
  declare_parameter("var_imu_w", 0.01);
  declare_parameter("var_imu_acc", 0.01);
  declare_parameter("var_odom_xyz", 0.2);
  get_parameter("pub_period", pub_period_);
  get_parameter("var_imu_w", var_imu_w_);
  get_parameter("var_imu_acc", var_imu_acc_);
  get_parameter("var_odom_xyz", var_odom_xyz_);
 
  // publisher
  posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);

  // subscriber
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_callback(msg);
      });
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      imu_callback(msg);
      });
}


void ExtendedKalmanFilter::init()
{
  X_ << 0, 0, 0;
  initialized = true;
}


void ExtendedKalmanFilter::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!initialized) {
    init();
  } else {
  measurementUpdate(msg);
  }
}


void ExtendedKalmanFilter::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!initialized) {
    init();
  } else {
    sensor_msgs::msg::Imu transformed_msg;
    try {
      geometry_msgs::msg::Vector3Stamped acc_in, acc_out, w_in, w_out;
      acc_in.vector.x = msg->linear_acceleration.x;
      acc_in.vector.y = msg->linear_acceleration.y;
      acc_in.vector.z = msg->linear_acceleration.z;
      w_in.vector.x = msg->angular_velocity.x;
      w_in.vector.y = msg->angular_velocity.y;
      w_in.vector.z = msg->angular_velocity.z;
      tf2::TimePoint time_point = tf2::TimePoint(
        std::chrono::seconds(msg->header.stamp.sec) +
        std::chrono::nanoseconds(msg->header.stamp.nanosec));
      const geometry_msgs::msg::TransformStamped transform =
        tfbuffer_.lookupTransform(
        robot_frame_id_,
        msg->header.frame_id,
        time_point);
      tf2::doTransform(acc_in, acc_out, transform);
      tf2::doTransform(w_in, w_out, transform);
      transformed_msg.header.stamp = msg->header.stamp;
      transformed_msg.angular_velocity.x = w_out.vector.x;
      transformed_msg.angular_velocity.y = w_out.vector.y;
      transformed_msg.angular_velocity.z = w_out.vector.z;
      transformed_msg.linear_acceleration.x = acc_out.vector.x;
      transformed_msg.linear_acceleration.y = acc_out.vector.y;
      transformed_msg.linear_acceleration.z = acc_out.vector.z;
      measurementUpdate(transformed_msg);
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      return;
    } catch (std::runtime_error & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      return;
    }
  }
}


void ExtendedKalmanFilter::predictUpdate(nav_msgs::msg::Odometry::SharedPtr msg)
{

  // 予測ステップ
  /*
  ------------------------------
  1.運動方程式　　 
  2.ヤコビアン計算 
  3.事前誤差共分散行列の推定 
  ------------------------------
  F:ヤコビアン(運動方程式の)
  P:事前誤差共分散行列
  Q:プロセスノイズ共分散行列
  ------------------------------
  */

  //1.運動モデルによる予測(事前推定値)
  odomtimestamp = msg->header.stamp;
  double dt = odomtimestamp - old_timestamp
  old_timestamp = odomtimestamp;
  X(0) = msg->pose.pose.position.x;
  X(1) = msg->pose.pose.position.y;
  X(2) = msg->pose.pose.position.z;
  double v = msg->twist.twist.linear.z;
  double theta = msg->twist.twist.angular.z;
  //2.ヤコビアン計算
  Matrix3d F;
  F << 1, 0, -v*sin(theta)*dt ,
        0, 1,  v*cos(theta)*dt ,
        0, 0, 1;

  //3.事前誤差共分散行列の推定
  P = F * P * F.transpose() + Q;

}

void ExtendedKalmanFilter::measurementUpdate(sensor_msgs::msg::Imu msg)
{   
  /*
  Update : 観測後の信念分布の更新
  1.観測ヤコビアンの計算 :
  2.
  3.カルマンゲインの計算 :
  4.観測予測値の計算
  5.共分散の更新　 :
  */

  //1.観測予測値の計算
  imutimestamp = msg->header.stamp;
  double dt = imutimestamp - old_timestamp;
  old_timestamp = imutimestamp;

  double v += u(0)*dt
  double x = X(0), y = X(1), theta = X(2);
  double dx = v*cos(theta);
  double dy = v*sin(theta);
  double dtheta = u(5);
  Z(0) = x + dx * dt;
  Z(1) = y + dy * dt;
  Z(2) = theta + dtheta * dt;

  //2.観測ヤコビアンの計算
  Matrix3d H;
  H << 1, 0, -v*sin(theta)*dt ,
        0, 1,  v*cos(theta)*dt ,
        0, 0, 1;

  //3.観測予測値の計算
  Eigen::MatrixXd h = Eigen::MatrixXd::Identity(3,3);
  VectorXdy = Z - h * X;

  //3.カルマンゲインの計算
  MatrixXd S = H * P * H.transpose() + R;
  MatrixXd K = P * H.transpose() * S.inverse();
  //3.観測予測値の計算
  X = X + K * y;
  P = (Matrix3d::Identity() - K * H) * P;
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ekf_localization::ExtendedKalmanFilter)


