#include <iostream>
#include <chrono>
#include <functional>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

using Eigen::MatrixXd;

namespace ekf_localization
{
class ExtendedKalmanFilter : public rclcpp::Node
{
public:
  explicit ExtendedKalmanFilter(const rclcpp::NodeOptions & options);

private:
  Eigen::Vector3d X; 
  Eigen::Vector3d Pre_X;
  Eigen::Vector3d Z; 

  // noise_pram
  std::vector<double> var_imu_;
  std::vector<double> var_wheel_odom_;
  std::vector<double> var_ndt_odom_;
  MatrixXd Q_wheel_ = MatrixXd::Zero(3, 3);
  MatrixXd R_ndt_ = MatrixXd::Zero(3, 3);
  MatrixXd R_imu_ = MatrixXd::Zero(3, 3);

  MatrixXd P_ = MatrixXd::Zero(3, 3);
  MatrixXd H_ = MatrixXd::Zero(3, 3);

  rclcpp::Time prev_time_;
  
  rclcpp::Time last_ndt_time_;
  double ndt_timeout_duration_;  

  void init();
  void init(double x, double y, double theta); 
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  void wheel_odom_callback(const nav_msgs::msg::Odometry & msg);
  void ndt_odom_callback(const nav_msgs::msg::Odometry & msg);
  void imu_callback(const sensor_msgs::msg::Imu & msg);
 
  void predictUpdate(double dt);
  void measurementUpdate(double dt);
  void ekf_timer_callback();  

  void update_with_ndt(double dt);
  void update_with_imu(double dt);
  
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;

  bool broadcast_transform_;

  std::string map_frame_id_;
  std::string robot_frame_id_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ndt_odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_publisher_;
  
  rclcpp::TimerBase::SharedPtr ekf_timer_;
  double ekf_frequency_;
  
  nav_msgs::msg::Odometry latest_wheel_odom_msg_;
  nav_msgs::msg::Odometry latest_ndt_msg_;
  nav_msgs::msg::Odometry previous_ndt_msg_;
  sensor_msgs::msg::Imu latest_imu_msg_;
  bool wheel_odom_received_;
  bool ndt_odom_received_;
  bool imu_received_;
};
}  // namespace 
