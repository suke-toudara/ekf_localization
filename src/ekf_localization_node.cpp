#include <ekf_localization/ekf_localization_component.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<ekf_localization::ExtendedKalmanFilter>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}