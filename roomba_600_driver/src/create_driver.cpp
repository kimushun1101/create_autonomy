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
      using namespace std::chrono_literals;

      std::cout<<"constructer"<<std::endl;

      counter_ = 0;

      timer_ = create_wall_timer(100ms, std::bind(&CreateDriver::update, this));

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
  }
}
