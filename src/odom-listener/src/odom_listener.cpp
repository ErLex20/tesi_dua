#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomListener : public rclcpp::Node
{
public:
  OdomListener()
  : Node("odom_listener")
  {
    // gazebo_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   "/seppia/odom", 10, std::bind(&OdomListener::gazebo_callback, this, std::placeholders::_1));
      
    lio_sam_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/lio_sam/mapping/odometry", 10, std::bind(&OdomListener::lio_sam_callback, this, std::placeholders::_1));
  }

private:
  void gazebo_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received /seppia/odom message: position: [%.2f, %.2f, %.2f]", 
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  void lio_sam_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received /lio_sam/mapping/odometry message: position: [%.2f, %.2f, %.2f]", 
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gazebo_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sam_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomListener>());
  rclcpp::shutdown();
  return 0;
}
