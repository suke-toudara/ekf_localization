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
  bool initialized = false;

  float v;
  float theta;
  Eigen::Vector3d X; 
  Eigen::Vector3d Z; 

  // noise_pram
  float var_imu_w_;
  float var_imu_acc_;
  float var_odom_xyz_;
  MatrixXd Q = MatrixXd::Zero(3, 3);
  MatrixXd R = MatrixXd::Zero(3, 3);

  // 共分散行列
  MatrixXd P = MatrixXd::Zero(3, 3);

  rclcpp::Time prev_time_;
  rclcpp::Time odomtimestamp;
  rclcpp::Time imutimestamp;

  void init();
  void odom_callback(const nav_msgs::msg::Odometry & msg);
  void imu_callback(const sensor_msgs::msg::Imu & msg);
 
  void predictUpdate(const nav_msgs::msg::Odometry & msg);
  void measurementUpdate(const sensor_msgs::msg::Imu & msg);
  
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;

  bool broadcast_transform_;

  std::string map_frame_id_;
  std::string robot_frame_id_;
  std::string initial_pose_topic_;
  std::string ekf_pose_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_publisher_;
};
}  // namespace 
