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
    declare_parameter("baud", int(model_.getBaud()));

    dev_ = this->get_parameter("dev").as_string();
    robot_model_name = this->get_parameter("robot_model").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    latch_duration_ = this->get_parameter("latch_cmd_duration").as_double();
    loop_hz_ = this->get_parameter("loop_hz").as_double();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    baud_ = this->get_parameter("baud").as_int();

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

    // Set frame_id's
    mode_msg_.header.frame_id = base_frame_;
    bumper_msg_.header.frame_id = base_frame_;
    charging_state_msg_.header.frame_id = base_frame_;
    tf_odom_.header.frame_id = odom_frame_;
    tf_odom_.child_frame_id = base_frame_;
    odom_msg_.header.frame_id = odom_frame_;
    odom_msg_.child_frame_id = base_frame_;
    joint_state_msg_.name.resize(2);
    joint_state_msg_.position.resize(2);
    joint_state_msg_.velocity.resize(2);
    joint_state_msg_.effort.resize(2);
    joint_state_msg_.name[0] = "left_wheel_joint";
    joint_state_msg_.name[1] = "right_wheel_joint";

    // Populate intial covariances
    for (int i = 0; i < 36; i++)
    {
      odom_msg_.pose.covariance[i] = COVARIANCE[i];
      odom_msg_.twist.covariance[i] = COVARIANCE[i];
    }

    // add by Yudai Sadakuni
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Setup subscribers
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", std::bind(&CreateDriver::cmdVelCallback, this, std::placeholders::_1));
    debris_led_sub_ = create_subscription<std_msgs::msg::Bool>(
        "debris_led", std::bind(&CreateDriver::debrisLEDCallback, this, std::placeholders::_1));
    spot_led_sub_ = create_subscription<std_msgs::msg::Bool>(
        "spot_led", std::bind(&CreateDriver::spotLEDCallback, this, std::placeholders::_1));
    dock_led_sub_ = create_subscription<std_msgs::msg::Bool>(
        "dock_led", std::bind(&CreateDriver::dockLEDCallback, this, std::placeholders::_1));
    check_led_sub_ = create_subscription<std_msgs::msg::Bool>(
        "check_led", std::bind(&CreateDriver::checkLEDCallback, this, std::placeholders::_1));
    power_led_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "power_led", std::bind(&CreateDriver::powerLEDCallback, this, std::placeholders::_1));
    set_ascii_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "set_ascii", std::bind(&CreateDriver::setASCIICallback, this, std::placeholders::_1));
    dock_sub_ = create_subscription<std_msgs::msg::Empty>(
        "dock", std::bind(&CreateDriver::dockCallback, this, std::placeholders::_1));
    undock_sub_ = create_subscription<std_msgs::msg::Empty>(
        "undock", std::bind(&CreateDriver::undockCallback, this, std::placeholders::_1));
    define_song_sub_ = create_subscription<ca_msgs::msg::DefineSong>(
        "define_song", std::bind(&CreateDriver::defineSongCallback, this, std::placeholders::_1));
    play_song_sub_ = create_subscription<ca_msgs::msg::PlaySong>(
        "play_song", std::bind(&CreateDriver::playSongCallback, this, std::placeholders::_1));

    // Setup publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 30);
    clean_btn_pub_ = create_publisher<std_msgs::msg::Empty>("clean_button");
    day_btn_pub_ = create_publisher<std_msgs::msg::Empty>("day_button");
    hour_btn_pub_ = create_publisher<std_msgs::msg::Empty>("hour_button");
    min_btn_pub_ = create_publisher<std_msgs::msg::Empty>("minute_button");
    dock_btn_pub_ = create_publisher<std_msgs::msg::Empty>("dock_button");
    spot_btn_pub_ = create_publisher<std_msgs::msg::Empty>("spot_button");
    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("battery/voltage");
    current_pub_ = create_publisher<std_msgs::msg::Float32>("battery/current");
    charge_pub_ = create_publisher<std_msgs::msg::Float32>("battery/charge");
    charge_ratio_pub_ = create_publisher<std_msgs::msg::Float32>("battery/charge_ratio");
    capacity_pub_ = create_publisher<std_msgs::msg::Float32>("battery/capacity");
    temperature_pub_ = create_publisher<std_msgs::msg::Int16>("battery/temperature");
    charging_state_pub_ = create_publisher<ca_msgs::msg::ChargingState>("battery/charging_state");
    omni_char_pub_ = create_publisher<std_msgs::msg::UInt16>("ir_omni");
    mode_pub_ = create_publisher<ca_msgs::msg::Mode>("mode");
    bumper_pub_ = create_publisher<ca_msgs::msg::Bumper>("bumper");
    wheeldrop_pub_ = create_publisher<std_msgs::msg::Empty>("wheeldrop");
    wheel_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states");

    timer_ = create_wall_timer(50ms,
      std::bind(&CreateDriver::update, this));

    RCLCPP_INFO(get_logger(), "[CREATE] Ready.");
  }

  CreateDriver::~CreateDriver()
  {
    std::cout<<"deconstructer"<<std::endl;
  }

  void CreateDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    robot_->drive(msg->linear.x, msg->angular.z);
    last_timer_ = ros_clock_.now();
  }

  void CreateDriver::debrisLEDCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    robot_->enableDebrisLED(msg->data);
  }

  void CreateDriver::spotLEDCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    robot_->enableSpotLED(msg->data);
  }

  void CreateDriver::dockLEDCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    robot_->enableDockLED(msg->data);
  }

  void CreateDriver::checkLEDCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    robot_->enableCheckRobotLED(msg->data);
  }

  void CreateDriver::powerLEDCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (msg->data.empty())
    {
      RCLCPP_ERROR(get_logger(), "[CREATE] No values provided to set power LED");
    }
    else
    {
      if (msg->data.size() < 2)
      {
        robot_->setPowerLED(msg->data[0]);
      }
      else
      {
        robot_->setPowerLED(msg->data[0], msg->data[1]);
      }
    }
  }

  void CreateDriver::setASCIICallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    bool result = false;
    if (msg->data.empty())
    {
      RCLCPP_ERROR(get_logger(), "[CREATE] No ASCII digits provided");
    }
    else if (msg->data.size() < 2)
    {
      result = robot_->setDigitsASCII(msg->data[0], ' ', ' ', ' ');
    }
    else if (msg->data.size() < 3)
    {
      result = robot_->setDigitsASCII(msg->data[0], msg->data[1], ' ', ' ');
    }
    else if (msg->data.size() < 4)
    {
      result = robot_->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], ' ');
    }
    else
    {
      result = robot_->setDigitsASCII(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    }

    if (!result)
    {
      RCLCPP_ERROR(get_logger(), "[CREATE] ASCII character out of range [32, 126]");
    }
  }

  void CreateDriver::dockCallback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    robot_->setMode(create::MODE_PASSIVE);

    if (model_.getVersion() <= create::V_2)
      usleep(1000000);  // Create 1 requires a delay (1 sec)

    // Call docking behaviour
    robot_->dock();
  }

  void CreateDriver::undockCallback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    // Switch robot back to FULL mode
    robot_->setMode(create::MODE_FULL);
  }

  void CreateDriver::defineSongCallback(const ca_msgs::msg::DefineSong::SharedPtr msg)
  {
    if (!robot_->defineSong(msg->song, msg->length, &(msg->notes.front()), &(msg->durations.front())))
    {
      RCLCPP_ERROR(get_logger(), "[CREATE] Failed to define song %d of length %d",
                   msg->song, msg->length);
    }
  }

  void CreateDriver::playSongCallback(const ca_msgs::msg::PlaySong::SharedPtr msg)
  {
    if (!robot_->playSong(msg->song))
    {
      RCLCPP_ERROR(get_logger(), "[CREATE] Failed to play song %d", msg->song);
    }
  }

  void CreateDriver::update()
  {
    std::cout<<"update"<<std::endl;

    if(ros_clock_.now() - last_timer_ >= rclcpp::Duration(latch_duration_)){
      std::cout<<"cmd slow or stop"<<std::endl;
      robot_->drive(0, 0);
    }
  }

  void CreateDriver::test()
  {
    real_timer_ = ros_clock_.now();
    last_timer_ = ros_clock_.now();
    auto dif = ros_clock_.now() - last_timer_;

    if(ros_clock_.now() - last_timer_ >= rclcpp::Duration(0.1))
    //if(dif >= rclcpp::Duration(0.001))
      std::cout<<"dif"<<std::endl;

    counter_++;
  }
}
