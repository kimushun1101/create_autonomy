#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "roomba_600_driver/teleop_twist_joy.hpp"

namespace teleop_twist_joy
{
  TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions& options)
   : Node("teleop_twist_joy", options)
  {
    using namespace std::chrono_literals;

    std::cout<<"constructer"<<std::endl;

    declare_parameter("scale_angular", 0.5);
    declare_parameter("scale_linear", 0.1);
    declare_parameter("connection_mode", 1);

    a_scale_ = this->get_parameter("scale_angular").as_double();
    l_scale_ = this->get_parameter("scale_linear").as_double();
    connection_mode_ = this->get_parameter("connection_mode").as_int();

    std::cout<<"scale linear  : "<<l_scale_<<std::endl;
    std::cout<<"scale angular : "<<a_scale_<<std::endl;

    if (connection_mode_ != 0 && connection_mode_ != 1)
    {
      RCLCPP_FATAL(get_logger(),
          "connecion_mode is nether 0 or 1. set 1 as default.");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(get_logger(), "Mode : %d", connection_mode_);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", std::bind(&TeleopTwistJoy::joyCallback, this, std::placeholders::_1));

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel");

    timer_ = create_wall_timer(5ms, std::bind(&TeleopTwistJoy::update, this));
  }

  void TeleopTwistJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    geometry_msgs::msg::Twist vel;

    double joy_L_ver, joy_L_hor, joy_L2, joy_R2;
    if (connection_mode_ == 1){
      joy_L_ver = msg->axes[LEFT_STICK_VERTICAL_1];
      joy_L_hor = msg->axes[LEFT_STICK_HORIZONTAL_1];
      joy_L2 = msg->axes[L2_1];
      joy_R2 = msg->axes[R2_1];
    } else if (connection_mode_ == 0) {
      joy_L_ver = msg->axes[LEFT_STICK_VERTICAL_0];
      joy_L_hor = msg->axes[LEFT_STICK_HORIZONTAL_0];
      joy_L2 = msg->axes[L2_0];
      joy_R2 = msg->axes[R2_0];
    }

    vel_.linear.x = l_scale_ * joy_L_ver;
    vel_.angular.z = a_scale_ * joy_L_hor;

    // vel_pub_->publish(vel_);
  }

  void TeleopTwistJoy::update()
  {
    vel_pub_->publish(vel_);
  }

  TeleopTwistJoy::~TeleopTwistJoy()
  {
    std::cout<<"deconstruct"<<std::endl;
  }
} // namespace teleop_twist_joy
