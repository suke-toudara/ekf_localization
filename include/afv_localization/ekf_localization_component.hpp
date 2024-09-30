#include <Eigen/Dense>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace robotx_ekf
{
class EKFComponent : public rclcpp::Node
{
public:
  ROBOTX_EKF_EKF_COMPONENT_PUBLIC
  explicit EKFComponent(const rclcpp::NodeOptions & options);


  //追加
  Eigen::Vector3d X; // 状態ベクトル (x, y, theta)
  Eigen::Matrix3d P; // 共分散行列
  Eigen::Matrix3d Q; // プロセスノイズ共分散行列
  Eigen::Matrix3d R; // 観測ノイズ共分散行列

  Eigen::Vector2d u;
  Eigen::VectorXd y; //Eigen::Vector3d y;

  rclcpp::Time odomtimestamp;
  rclcpp::Time imutimestamp;

private:
  double dt = 0.01;  // looprate

  //
  MatrixXd F = MatrixXd::Identity(3, 3);  // State Matrix
  MatrixXd H = MatrixXd::Identity(3, 3);  // Output Matrix
  
  // Process noise covariance Q
  MatrixXd Q = MatrixXd::Zero(3, 3);
  Q(0, 0) = 0.1;  // Process noise for x
  Q(1, 1) = 0.1;  // Process noise for y
  Q(2, 2) = 0.01; // Process noise for theta
  
  MatrixXd R = MatrixXd::Zero(3, 3);
  R(0, 0) = 0.1;  // Process noise for x
  R(1, 1) = 0.1;  // Process noise for y
  R(2, 2) = 0.01; // Process noise for theta
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
 
  bool init();
  void update();
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odomsubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMUsubscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr Posepublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace robotx_ekf

#endif  // ROBOTX_EKF__EKF_COMPONENT_HPP_