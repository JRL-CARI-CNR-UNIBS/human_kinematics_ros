#include "rclcpp/rclcpp.hpp"
#include "human_kinematics_ros/human_kinematics_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanKinematicsPublisher>());
  rclcpp::shutdown();
  return 0;
}