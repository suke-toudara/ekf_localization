#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace ekf_localization
{
class ExtendedKalmanFilter : public rclcpp::Node
{
public:
  explicit ExtendedKalmanFilter(const rclcpp::NodeOptions & options);

private:
  bool initialized = false;
  // 状態ベクトル (x, y, theta)
  Eigen::Vector3d X; 
  
  // プロセスノイズ共分散行列
  MatrixXd Q = MatrixXd::Zero(3, 3);
  Q(0, 0) = 0.1;  // Process noise for x
  Q(1, 1) = 0.1;  // Process noise for y
  Q(2, 2) = 0.01; // Process noise for theta
  
  // 観測ノイズ共分散行列
  MatrixXd R = MatrixXd::Zero(3, 3);
  R(0, 0) = 0.1;  // Process noise for x
  R(1, 1) = 0.1;  // Process noise for y
  R(2, 2) = 0.01; // Process noise for theta
  // 共分散行列
  MatrixXd P = MatrixXd::Zero(3, 3);

  timestamp;
  rclcpp::Time odomtimestamp;
  rclcpp::Time imutimestamp;

  void init();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
 
  void predictUpdate(nav_msgs::msg::Odometry::SharedPtr msg);
  void measurementUpdate(sensor_msgs::msg::Imu msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posepublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace 

#endif  