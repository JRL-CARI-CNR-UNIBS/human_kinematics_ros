#include <rclcpp/rclcpp.hpp>
#include <human_kinematics_ros/human_kinematics_publisher.hpp>

int main(int argc, char * argv[])
{
  const std::string& node_name("human_kinematics_publisher");
  const std::string& origin_frame("world");
  const std::string& camera_frame("zed_left_camera_frame");
  const std::string& configuration_topic_name("body_configuration");
  const std::string& param_topic_name("body_parameters");
  const std::string& keypoints_topic_name("/zed/zed_node/body_trk/skeletons");

  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting human_kinematics_publisher node...");
  
  rclcpp::spin(std::make_shared<human_kinematics_ros::HumanKinematicsPublisher>(
    node_name,
    origin_frame,
    camera_frame,
    configuration_topic_name,
    param_topic_name,
    keypoints_topic_name));
  
  rclcpp::shutdown();
  return 0;
}