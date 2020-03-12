#ifndef TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
#define TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace teleop_twist_joy
{
  class TeleopTwistJoy : public rclcpp::Node
  {
    public:
      explicit TeleopTwistJoy(const rclcpp::NodeOptions & options);
      virtual ~TeleopTwistJoy();

    private:
      double l_scale_;
      double a_scale_;
      int connection_mode_;

      rclcpp::TimerBase::SharedPtr timer_;
      geometry_msgs::msg::Twist vel_;

      enum JoyAxesJS1 {
        LEFT_STICK_HORIZONTAL_1 = 0,
        LEFT_STICK_VERTICAL_1 = 1,
        L2_1 = 2,
        RIGHT_STICK_HORIZONTAL_1 = 3,
        RIGHT_STICK_VERTICAL_1 = 4,
        R2_1 = 5,
        CROSS_HORIZONTAL_1 = 6,
        CROSS_VERTICAL_1 = 7
      };

      enum JoyAxisJS0 {
        LEFT_STICK_HORIZONTAL_0 = 0,
        LEFT_STICK_VERTICAL_0 = 1,
        RIGHT_STICK_HORIZONTAL_0 = 2,
        L2_0 = 3,
        R2_0 = 4,
        RIGHT_STICK_VERTICAL_0 = 5,
      };

      void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

      void update();

    protected:
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  };
} // namespace teleop_twist_joy

#endif  // TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
