#include "theta_driver/theta_driver_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<theta_driver::ThetaDriverNode>());
  rclcpp::shutdown();
  return 0;
}