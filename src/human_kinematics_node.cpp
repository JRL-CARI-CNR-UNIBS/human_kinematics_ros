#include <rclcpp/rclcpp.hpp>
#include <human_kinematics_ros/human_kinematics_publisher.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting human_kinematics_publisher node...");
  rclcpp::spin(std::make_shared<human_kinematics_ros::HumanKinematicsPublisher>());
  rclcpp::shutdown();
  return 0;
}