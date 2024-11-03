#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <ekf_localization/ekf_localization_component.hpp>

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
  declare_parameter("initial_pose_topic","/initial_pose");
  declare_parameter("ekf_pose_topic","/estimated_pose");
  declare_parameter("imu_topic","/imu");
  declare_parameter("odom_topic","/odom");
  declare_parameter("broadcast_transform", true);
  
  get_parameter("map_frame_id", map_frame_id_);
  get_parameter("robot_frame_id", robot_frame_id_);
  get_parameter("initial_pose_topic", initial_pose_topic_);
  get_parameter("ekf_pose_topic", ekf_pose_topic_);
  get_parameter("imu_topic", imu_topic_);
  get_parameter("odom_topic", odom_topic_);
  get_parameter("broadcast_transform", broadcast_transform_);

  declare_parameter("var_imu_w", 0.1);
  declare_parameter("var_imu_acc", 0.01);
  declare_parameter("var_odom_xyz", 0.1);
  get_parameter("var_imu_w", var_imu_w_);
  get_parameter("var_imu_acc", var_imu_acc_);
  get_parameter("var_odom_xyz", var_odom_xyz_);
 
  // declare_parameter("pub_period", 10);
  // get_parameter("pub_period", pub_period_);
  
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  qos.best_effort(); 
  
  // publisher
  ekf_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);

  // subscriber
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, [this](const nav_msgs::msg::Odometry & msg) {
      odom_callback(msg);
      });
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10, [this](const sensor_msgs::msg::Imu & msg) {
      imu_callback(msg);
      });
  
  P(0, 0) = 0.1;  
  P(1, 1) = 0.1;  
  P(2, 2) = 0.1;  
  
  // プロセスノイズ共分散行列
  Q(0, 0) = var_odom_xyz_;  // Process noise for x
  Q(1, 1) = var_odom_xyz_;  // Process noise for y
  Q(2, 2) = var_odom_xyz_;  // Process noise for theta
  
  // 観測ノイズ共分散行列
  R(0, 0) = var_imu_w_;   // Process noise for x
  R(1, 1) = var_imu_w_;   // Process noise for y
  R(2, 2) = var_imu_acc_; // Process noise for theta
}


void ExtendedKalmanFilter::init()
{
  X << 0, 0, 0;
  initialized = true;
  prev_time_ = this->now();
}


void ExtendedKalmanFilter::odom_callback(const nav_msgs::msg::Odometry & msg)
{
  if (!initialized) {
    init();
  } else {
    nav_msgs::msg::Odometry odom_msg = msg; 
    predictUpdate(odom_msg);
  }
}

void ExtendedKalmanFilter::imu_callback(const sensor_msgs::msg::Imu & msg)
{
  if (!initialized) {
    init();
  } else {
    // measurementUpdate(msg);
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
      tf2::doTransform(orientation_in, orientation_out, transform); //追加

      transformed_msg.header.stamp = msg.header.stamp;
      transformed_msg.angular_velocity.x = w_out.vector.x;
      transformed_msg.angular_velocity.y = w_out.vector.y;
      transformed_msg.angular_velocity.z = w_out.vector.z;
      transformed_msg.linear_acceleration.x = acc_out.vector.x;
      transformed_msg.linear_acceleration.y = acc_out.vector.y;
      transformed_msg.linear_acceleration.z = acc_out.vector.z; 
      transformed_msg.orientation = orientation_out.quaternion; //追加

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


void ExtendedKalmanFilter::predictUpdate(const nav_msgs::msg::Odometry & msg)
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
  odomtimestamp = msg.header.stamp;
  rclcpp::Duration duration = odomtimestamp - prev_time_;
  double dt = duration.seconds(); 
  prev_time_ = odomtimestamp;
  v = msg.twist.twist.linear.x;
  
  tf2::Quaternion q(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  X(0) = msg.pose.pose.position.x;
  X(1) = msg.pose.pose.position.y;
  X(2) = yaw;

  //2.ヤコビアン計算
  Matrix3d F;
  F << 1, 0, -v*sin(yaw)*dt ,
        0, 1,  v*cos(yaw)*dt ,
        0, 0, 1;

  //3.事前誤差共分散行列の推定
  P = F * P * F.transpose() + Q;
}

void ExtendedKalmanFilter::measurementUpdate(const sensor_msgs::msg::Imu & msg)
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
  imutimestamp = msg.header.stamp;
  rclcpp::Duration duration = imutimestamp - prev_time_;
  double dt = duration.seconds(); 
  prev_time_ = imutimestamp;
  double x = X(0), y = X(1);
  v += msg.linear_acceleration.x*dt;
  tf2::Quaternion q_imu(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
  );
  tf2::Matrix3x3 m(q_imu);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double dx = v*cos(yaw);
  double dy = v*sin(yaw);

  Z(0) = x + dx * dt;
  Z(1) = y + dy * dt;
  Z(2) = yaw;

  //2.観測ヤコビアンの計算
  Matrix3d H;
  H << 1, 0, -v*sin(yaw)*dt ,
        0, 1,  v*cos(yaw)*dt ,
        0, 0, 1;

  //3.観測予測値の計算
  Eigen::MatrixXd h = Eigen::MatrixXd::Identity(3,3);
  Eigen::VectorXd Y = Z - h * X;

  //3.カルマンゲインの計算
  MatrixXd S = H * P * H.transpose() + R;
  MatrixXd K = P * H.transpose() * S.inverse();
  //3.観測予測値の計算
  X = X + K * Y;
  P = (Matrix3d::Identity() - K * H) * P;

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
  //   P(0, 0), P(0, 1), 0, 0, 0, P(0, 2),
  //   P(1, 0), P(1, 1), 0, 0, 0, P(1, 2),
  //   0,       0,       0, 0, 0, 0,
  //   0,       0,       0, 0, 0, 0,
  //   0,       0,       0, 0, 0, 0,
  //   P(2, 0), P(2, 1), 0, 0, 0, P(2, 2),
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

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ekf_localization::ExtendedKalmanFilter)


