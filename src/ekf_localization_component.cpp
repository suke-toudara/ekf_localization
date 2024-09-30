#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <ekf_localization/ekf_localization_component.hpp>

using namespace std::chrono_literals;
using namespace Eigen;

xtendedKalmanFilterComponent::ExtendedKalmanFilter(const rclcpp::NodeOptions & options)
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
 
  X << 0, 0, 0; // 初期位置と姿勢
  P = Matrix3d::Identity(); // 初期共分散行列

  double Q_x = 3;
  Q << Q_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0, M_x, 0, 0, 0, 0, 0, 0,
      M_x, 0, 0, 0, 0, 0, 0, M_x;
      
  double M_x = 50;
  R << R_x, 0, 0, 0, 0, 0, 
        0, R_x, 0, 0, 0, 0, 
        0, 0, R_x, 0, 0, 0, 
        0, 0, 0, 10 * R_x, 0, 0, 
        0, 0, 0, 0, 10 * R_x, 0, 
        0, 0, 0, 0, 0, 10 * R_x;

  // publisher
  Posepublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);

  // subscriber
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      odom_callback(msg);
      });

  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
      odom_callback(msg);
      });

  timer_ = this->create_wall_timer(10ms, std::bind(&ExtendedKalmanFilterComponent::update, this));
}


bool ExtendedKalmanFilterComponent::init()
{
  if (Y(0) != 0 && u(0) != 0) {
    X << Y(0), Y(1), Y(2), 0, 0, 0, 1, 0, 0, 0;
    initialized = true;
  } else {
    initialized = false;
  }
  return initialized;
}


void ExtendedKalmanFilterComponent::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  measurementUpdate(msg);
}


void EKFComponent::IMUtopic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  //

  if (initial_pose_recieved_) {
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
      predictUpdate(transformed_msg);
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      return;
    } catch (std::runtime_error & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      return;
    }
  }
}

void ExtendedKalmanFilterComponent::step()
{
  if (!initialized) {
    init();
  } else {
    predict();
    update();

    geometry_msgs::msg::PoseWithCovarianceStamped pose_ekf;
    pose_ekf.header.stamp = odomtimestamp;
   
    pose_ekf.header.frame_id = "/map";
    pose_ekf.pose.pose.position.x = X(0);
    pose_ekf.pose.pose.position.y = X(1);
    pose_ekf.pose.pose.position.z = X(2);

    pose_ekf.pose.pose.orientation.w = X(3);
    pose_ekf.pose.pose.orientation.x = X(4);
    pose_ekf.pose.pose.orientation.y = X(5);
    pose_ekf.pose.pose.orientation.z = X(6);

    // pose_ekf.pose.covariance = {
    //   P(0, 0), P(0, 1), P(0, 2), P(0, 7), P(0, 8), P(0, 9), P(1, 0), P(1, 1), P(1, 2),
    //   P(1, 7), P(1, 8), P(1, 9), P(2, 0), P(2, 1), P(2, 2), P(2, 7), P(2, 8), P(2, 9),
    //   P(7, 0), P(7, 1), P(7, 2), P(7, 7), P(7, 8), P(7, 9), P(8, 0), P(8, 1), P(8, 2),
    //   P(8, 7), P(8, 8), P(8, 9), P(9, 0), P(9, 1), P(9, 2), P(9, 7), P(9, 8), P(9, 9)};
    Posepublisher_->publish(pose_ekf);
  }
}

void ExtendedKalmanFilterComponent::predictUpdate(nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!initialized) {
    init();
  } else {
    // 予測ステップ
    /*
    ------------------------------
    1.運動方程式　　 : X_Pred = motion_model(xEst, u)
    2.ヤコビアン計算 : jF = jacobF(xPred, u)
    3.事前誤差共分散行列の推定 : PPred = jF * PEst * jF.T + Q
    ------------------------------
    F:ヤコビアン(運動方程式の)
    P:事前誤差共分散行列
    Q:プロセスノイズ共分散行列
    ------------------------------
    */

    //1.運動モデルによる予測(事前推定値)
    odomtimestamp = msg->header.stamp;
    X(0) = msg->pose.pose.position.x;
    X(1) = msg->pose.pose.position.y;
    X(2) = msg->pose.pose.position.z;
    double v = msg->twist.twist.linear.z;
    double theta = msg->twist.twist.angle.z;
    //2.ヤコビアン計算
    Matrix3d F;
    F << 1, 0, -v*sin(theta)*dt_ ,
         0, 1,  v*cos(theta)*dt_ ,
         0, 0, 1;

    //3.事前誤差共分散行列の推定
    P_ = F * P_ * F.transpose() + Q_;
  }
}

void ExtendedKalmanFilterComponent::measurementUpdatepredictUpdate(const Vector3d& z)
{   
    /*
    Update : 観測後の信念分布の更新
    1.観測予測値の計算　　 : 
    2.観測ヤコビアンの計算 :
    3.カルマンゲインの計算 :
    4.状態と共分散の更新　 :
    */
    double v += u(0)
    double x = X(0), y = X(1), theta = X(2);
    double dx = v*cos(theta);
    double dy = v*sin(theta);
    double dtheta =u(5);
    Y = 0;
    //予測値と観測値の誤差
    VectorXd y = z - H * x_;

    // Innovation covariance
    MatrixXd S = H * P_ * H.transpose() + R;
    //カルマンゲイン計算
    MatrixXd K = P_ * H.transpose() * S.inverse();

    // Correct the state
    X_ = X_ + K * y;
    
    //状態推定値(事後信念bel(xt)の平均µtを計算) x_kalman = X_model + k*(Y_obserb-X_model)
    X = X + (1 - K) * Y;
    
    //事後誤差共分散行列(事後信念bel(xt)の共分散行列Σtを計算)
    P = (Matrix3d::Identity() - G * H) * P;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ekf_localization::ExtendedKalmanFilterComponent)


