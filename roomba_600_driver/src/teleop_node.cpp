#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "roomba_600_driver/teleop_twist_joy.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<teleop_twist_joy::TeleopTwistJoy>(rclcpp::NodeOptions()));
  rclcpp::shutdown();

  return 0;
}
