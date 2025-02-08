#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <human_model/human_model.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <zed_msgs/msg/keypoint3_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define N_DOF 28
#define N_PARAM 8


namespace human_kinematics_ros
{

class HumanKinematicsPublisher : public rclcpp::Node
{
public:
  HumanKinematicsPublisher();

private:
  std::string origin_frame_;
  std::string camera_frame_;
  std::string configuration_topic_name_;
  std::string param_topic_name_;
  std::string chest_q_rotated_topic_name_;
  std::string keypoints_topic_name_;

  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::ConstSharedPtr skeleton_subscription_;

	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr configuration_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr param_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr chest_q_rotated_publisher_;
  rclcpp::TimerBase::SharedPtr configuration_timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr transform_timer_;

	std_msgs::msg::Header current_header_;

	// Keypoints from the ZED camera in camera frame
  std::vector<zed_msgs::msg::Keypoint3D> zed_kpts_;

	// 3D Transformations
	Eigen::Affine3d T_world_camera_;
	Eigen::Affine3d T_ext_rshoulder_;
  Eigen::Affine3d T_ext_lshoulder_;
  Eigen::Affine3d T_ext_rhip_;
  Eigen::Affine3d T_ext_lhip_;
  Eigen::Affine3d T_ext_chest_;
  Eigen::Affine3d T_ext_head_;
  Eigen::Affine3d T_ext_rshoulderRotated_;
  Eigen::Affine3d T_ext_relbow_;
  Eigen::Affine3d T_ext_rwrist_;
  Eigen::Affine3d T_ext_lshoulderRotated_;
  Eigen::Affine3d T_ext_lelbow_;
  Eigen::Affine3d T_ext_lwrist_;
  Eigen::Affine3d T_ext_rhipRotated_;
  Eigen::Affine3d T_ext_rknee_;
  Eigen::Affine3d T_ext_rankle_;
  Eigen::Affine3d T_ext_lhipRotated_;
  Eigen::Affine3d T_ext_lknee_;
  Eigen::Affine3d T_ext_lankle_;

	// Human model
  human_model::keypoints keypoints_;
	human_model::Human28DOF human_model_;
	std::vector<human_model::JointLimits> joint_limits_;
	Eigen::VectorXd configuration_ = Eigen::VectorXd::Zero(N_DOF);
	Eigen::VectorXd param_ = Eigen::VectorXd::Zero(N_PARAM);
  Eigen::Vector4d chest_q_rotated_ = Eigen::Vector4d::Zero();

	// Body geometrical transformations
	std::map<std::string, geometry_msgs::msg::TransformStamped> body_transforms_;
  std::map<std::string, bool> body_transforms_nan_;

	void skeleton_callback(const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg);
	void publish_transforms();
	void publish_configuration_and_param();
	geometry_msgs::msg::TransformStamped affine3d_to_tf(const Eigen::Affine3d& T,
                                                      const std::string& parent_frame,
                                                      const std::string& child_frame,
                                                      bool& has_nan);
};

} // end namespace human_kinematics_ros