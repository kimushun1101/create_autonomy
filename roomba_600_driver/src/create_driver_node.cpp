#include <rclcpp/rclcpp.hpp>

#include "roomba_600_driver/create_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<create_driver::CreateDriver>(rclcpp::NodeOptions()));
  rclcpp::shutdown();

  return 0;
}
