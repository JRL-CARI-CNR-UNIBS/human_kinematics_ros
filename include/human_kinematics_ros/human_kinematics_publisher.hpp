#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <human_model/human_model.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>
#include <zed_interfaces/msg/keypoint3_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#define N_DOF 28
#define N_PARAM 8


namespace human_kinematics_ros
{

class HumanKinematicsPublisher : public rclcpp::Node
{
public:
  HumanKinematicsPublisher();

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::ConstSharedPtr skeleton_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr configuration_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr param_publisher_;

  std_msgs::msg::Header current_header_;

	// Keypoints from the ZED camera in camera frame
  std::vector<zed_interfaces::msg::Keypoint3D> zed_kpts_;

	// 3D Transformations
	Eigen::Affine3d TF_world_camera_;
	Eigen::Affine3d T_ext_rshoulder;
  Eigen::Affine3d T_ext_lshoulder;
  Eigen::Affine3d T_ext_rhip;
  Eigen::Affine3d T_ext_lhip;
  Eigen::Affine3d T_ext_chest;
  Eigen::Affine3d T_ext_head;
  Eigen::Affine3d T_ext_relbow;
  Eigen::Affine3d T_ext_rwrist;
  Eigen::Affine3d T_ext_lelbow;
  Eigen::Affine3d T_ext_lwrist;
  Eigen::Affine3d T_ext_rknee;
  Eigen::Affine3d T_ext_rankle;
  Eigen::Affine3d T_ext_lknee;
  Eigen::Affine3d T_ext_lankle;

	// Human model
  human_model::keypoints keypoints_;
	human_model::Human28DOF human_model_;
	std::vector<human_model::JointLimits> joint_limits_;
	Eigen::VectorXd configuration_;
	Eigen::VectorXd param_;

	// Body geometrical transformations
	std::map<std::string, geometry_msgs::msg::TransformStamped> body_transforms_;

	void skeleton_callback(const zed_interfaces::msg::ObjectsStamped::ConstSharedPtr msg);
	void publish_transforms();
	void publish_configuration_and_param();
	geometry_msgs::msg::TransformStamped affine3d_to_tf(const Eigen::Affine3d& T,
                                                      const std::string& parent_frame,
                                                      const std::string& child_frame);
};

} // end namespace human_kinematics_ros