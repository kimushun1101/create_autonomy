#include <limits>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose.hpp>


class TfBroadcaster : public rclcpp::Node
{
private:
  //std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  //tf2_ros::TransformBroadcaster br_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock ros_clock_;
  geometry_msgs::msg::TransformStamped tf_odom_;
  geometry_msgs::msg::Pose pose_;
  int count;

  void update();
  void publishOdom();

protected:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  explicit TfBroadcaster(const std::string & name);
  ~TfBroadcaster();
};
