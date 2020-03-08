#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include "roomba_600_driver/tf_broadcast.h"

TfBroadcaster::TfBroadcaster(const std::string & name)
  : Node(name),
    ros_clock_(RCL_ROS_TIME)
{
  using namespace std::chrono_literals;

  tf_odom_.header.frame_id = "odom";
  tf_odom_.child_frame_id = "base_footprint";

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  timer_ = create_wall_timer(100ms,
      [this]() -> void {
        this->update();
    });

  RCLCPP_INFO(get_logger(), "Ready");
}

TfBroadcaster::~TfBroadcaster()
{
  RCLCPP_INFO(get_logger(), "Destruct sequence initiated");
}

void TfBroadcaster::update()
{
  publishOdom();
}

void TfBroadcaster::publishOdom()
{
  std::cout<<"publish Odometry"<<std::endl;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);

  tf_odom_.header.stamp = ros_clock_.now();
  tf_odom_.transform.translation.x = 1;
  tf_odom_.transform.translation.y = 0;
  tf_odom_.transform.rotation.x = quat.x();
  tf_odom_.transform.rotation.y = quat.y();
  tf_odom_.transform.rotation.z = quat.z();
  tf_odom_.transform.rotation.w = quat.w();
  tf_broadcaster_->sendTransform(tf_odom_);
}

int main(int argc, char** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TfBroadcaster>("tf_broadcast");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
