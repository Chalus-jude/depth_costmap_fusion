#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>

class OdomTfBroadcaster : public rclcpp::Node
{
public:
  OdomTfBroadcaster()
  : Node("odom_tf_broadcaster")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/r1a006/wheel_odom");

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OdomTfBroadcaster::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "odom_tf_broadcaster up, subscribed to %s", odom_topic_.c_str());
  }

private:
  void callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = msg->header.frame_id;       // "odom"
    t.child_frame_id = msg->child_frame_id;          // "base_link"
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;
    broadcaster_->sendTransform(t);
  }

  std::string odom_topic_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTfBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
