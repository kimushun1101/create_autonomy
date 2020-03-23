#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "roomba_600_driver/create_driver.hpp"

namespace create_driver
{
  CreateDriver::CreateDriver(const rclcpp::NodeOptions& options)
    : Node("create_driver", options),
      model_(create::RobotModel::CREATE_2),
      ros_clock_(RCL_ROS_TIME)
  {
      std::cout<<"constructer"<<std::endl;

      using namespace std::chrono_literals;

      std::string robot_model_name;

      declare_parameter("dev", "/dev/ttyUSB0");
      declare_parameter("robot_model", "CREATE_2");
      declare_parameter("base_frame", "base_footprint");
      declare_parameter("odom_frame", "odom");
      declare_parameter("latch_cmd_duration", 0.2);
      declare_parameter("loop_hz", 10.0);
      declare_parameter("publish_tf", true);

      dev_ = this->get_parameter("dev").as_string();
      robot_model_name = this->get_parameter("robot_model").as_string();
      base_frame_ = this->get_parameter("base_frame").as_string();
      odom_frame_ = this->get_parameter("odom_frame").as_string();
      latch_duration_ = this->get_parameter("latch_cmd_duration").as_double();
      loop_hz_ = this->get_parameter("loop_hz").as_double();
      publish_tf_ = this->get_parameter("publish_tf").as_bool();

      if (robot_model_name == "ROOMBA_400")
      {
        model_ = create::RobotModel::ROOMBA_400;
      }
      else if (robot_model_name == "CREATE_1")
      {
        model_ = create::RobotModel::CREATE_1;
      }
      else if (robot_model_name == "CREATE_2")
      {
        model_ = create::RobotModel::CREATE_2;
      }
      else
      {
        RCLCPP_FATAL(get_logger(), "[CREATE] Robot model \"%s\" is not known.",
                     robot_model_name.c_str());
        rclcpp::shutdown();
        return;
      }

      RCLCPP_INFO(get_logger(), "[CREATE] \"%s\" selected",
                  robot_model_name.c_str());

      // comment out by Yudai Sadakuni
      // get_parameter_or<int>("baud", baud_, model_.getBaud());

      declare_parameter("baud", int(model_.getBaud()));
      baud_ = this->get_parameter("baud").as_int();

      robot_ = std::make_unique<create::Create>(model_);

      if (!robot_->connect(dev_, baud_))
      {
        RCLCPP_FATAL(get_logger(),
            "[CREATE] Failed to establish serial connection with Create.");
        rclcpp::shutdown();
      }

      RCLCPP_INFO(get_logger(), "[CREATE] Connection established.");

      // Start in full control mode
      robot_->setMode(create::MODE_FULL);

      // Show robot's battery level
      RCLCPP_INFO(get_logger(), "[CREATE] Battery level %.2f %%",
          (robot_->getBatteryCharge() / robot_->getBatteryCapacity()) * 100.0);

      counter_ = 0;

      timer_ = create_wall_timer(100ms, std::bind(&CreateDriver::update, this));

      RCLCPP_INFO(get_logger(), "[CREATE] Ready.");

      last_timer_ = ros_clock_.now();
  }

  void CreateDriver::update()
  {
    std::cout<<"update"<<std::endl;
    test();
  }

  void CreateDriver::test()
  {
    real_timer_ = ros_clock_.now();
    auto dif = ros_clock_.now() - last_timer_;

    if(dif >= rclcpp::Duration(0.001))
      std::cout<<"dif"<<std::endl;

    if(counter_<1000)
      robot_->drive(0.1, 0.0);
    else
      robot_->drive(0.0, 0.0);

    counter_++;
  }

  CreateDriver::~CreateDriver()
  {
    std::cout<<"deconstructer"<<std::endl;
    robot_->drive(0.0, 0.0);
  }
}
