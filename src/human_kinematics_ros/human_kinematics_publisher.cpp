#include <human_kinematics_ros/human_kinematics_publisher.hpp>


namespace human_kinematics_ros
{

HumanKinematicsPublisher::HumanKinematicsPublisher()
  : Node("human_kinematics_publisher")
{
  // Subscribe to the ZED skeleton topic
  skeleton_subscription_ = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
    "/zed/zed_node/body_trk/skeletons", 10,
    std::bind(&HumanKinematicsPublisher::skeleton_callback, this, std::placeholders::_1));

  // Initialize configuration and param vectors
  configuration_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("body_configuration", 10);
  param_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("body_param", 10);

  // Initialize the transformation matrix from the camera frame to the world frame
  TF_world_camera_.translation() << 0.100575, -0.9304, 2.31042;
  TF_world_camera_.linear() = Eigen::Quaterniond(0.828395, 0.180663, 0.516604, 0.119341).toRotationMatrix();

  // Instantiate the human model
  human_model_ = human_model::Human28DOF();

  // Set the joint limits
  std::vector<human_model::JointLimits> joint_limits_(N_DOF,
                                                      human_model::JointLimits(-M_PI, M_PI));

  // Set the shoulder rot y joint limits
  joint_limits_[12]=human_model::JointLimits(-M_PI/2,M_PI/2); // right shoulder
  joint_limits_[16]=human_model::JointLimits(-M_PI/2,M_PI/2); // left shoulder
  joint_limits_[20]=human_model::JointLimits(-M_PI/2,M_PI/2); // right hip
  joint_limits_[24]=human_model::JointLimits(-M_PI/2,M_PI/2); // left hip

  // Publish transformations
  // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  // timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(100),
  //   std::bind(&HumanKinematicsPublisher::publish_transforms, this));
}


void HumanKinematicsPublisher::skeleton_callback(const zed_interfaces::msg::ObjectsStamped::ConstSharedPtr msg)
{
  if (msg->objects.size() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Received empty ObjectsStamped message");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Received ObjectsStamped message with %zu objects", msg->objects.size());
  
  // Store the current header
  current_header_ = msg->header;

  // Iterate over the objects
  for (const auto& object : msg->objects)
  {
    RCLCPP_INFO(this->get_logger(), "Object ID: %d", object.label_id);
    
    if (object.skeleton_available)
    {
      // Get the raw keypoints
      zed_kpts_ = std::vector<zed_interfaces::msg::Keypoint3D>(
        object.skeleton_3d.keypoints.begin(), object.skeleton_3d.keypoints.end());

      for (auto& kpt : zed_kpts_)
      {
         // Check if the raw keypoints contain nan values
        if (std::isnan(kpt.kp[0]) || std::isnan(kpt.kp[1]) || std::isnan(kpt.kp[2]))
        {
          RCLCPP_WARN(this->get_logger(), "Received NaN keypoints");
          return;
        }

        // Transform the keypoints in the world frame
        Eigen::Vector3d kpt_camera(kpt.kp[0], kpt.kp[1], kpt.kp[2]);
        Eigen::Vector3d kpt_world = TF_world_camera_ * kpt_camera;
        kpt.kp[0] = kpt_world[0];
        kpt.kp[1] = kpt_world[1];
        kpt.kp[2] = kpt_world[2];
      }

      // Select the appropriate keypoints to pass to the human model
      keypoints_.head = Eigen::Vector3d(zed_kpts_[0].kp[0],
                                        zed_kpts_[0].kp[1],
                                        zed_kpts_[0].kp[2]);

      keypoints_.left_shoulder = Eigen::Vector3d(zed_kpts_[5].kp[0],
                                                 zed_kpts_[5].kp[1],
                                                 zed_kpts_[5].kp[2]);

      keypoints_.left_elbow = Eigen::Vector3d(zed_kpts_[6].kp[0],
                                              zed_kpts_[6].kp[1],
                                              zed_kpts_[6].kp[2]);
      keypoints_.left_wrist = Eigen::Vector3d(zed_kpts_[7].kp[0],
                                              zed_kpts_[7].kp[1],
                                              zed_kpts_[7].kp[2]);

      keypoints_.left_hip = Eigen::Vector3d(zed_kpts_[11].kp[0],
                                            zed_kpts_[11].kp[1],
                                            zed_kpts_[11].kp[2]);
      keypoints_.left_knee = Eigen::Vector3d(zed_kpts_[12].kp[0],
                                             zed_kpts_[12].kp[1],
                                             zed_kpts_[12].kp[2]);
      keypoints_.left_ankle = Eigen::Vector3d(zed_kpts_[13].kp[0],
                                              zed_kpts_[13].kp[1],
                                              zed_kpts_[13].kp[2]);

      keypoints_.right_shoulder = Eigen::Vector3d(zed_kpts_[2].kp[0],
                                                  zed_kpts_[2].kp[1],
                                                  zed_kpts_[2].kp[2]);
      keypoints_.right_elbow = Eigen::Vector3d(zed_kpts_[3].kp[0],
                                               zed_kpts_[3].kp[1],
                                               zed_kpts_[3].kp[2]);
      keypoints_.right_wrist = Eigen::Vector3d(zed_kpts_[4].kp[0],
                                               zed_kpts_[4].kp[1],
                                               zed_kpts_[4].kp[2]);

      keypoints_.right_hip = Eigen::Vector3d(zed_kpts_[8].kp[0],
                                             zed_kpts_[8].kp[1],
                                             zed_kpts_[8].kp[2]);
      keypoints_.right_knee = Eigen::Vector3d(zed_kpts_[9].kp[0],
                                              zed_kpts_[9].kp[1],
                                              zed_kpts_[9].kp[2]);
      keypoints_.right_ankle = Eigen::Vector3d(zed_kpts_[10].kp[0],
                                               zed_kpts_[10].kp[1],
                                               zed_kpts_[10].kp[2]);

      RCLCPP_INFO(this->get_logger(), "Keypoints [WORLD FRAME]: %s", keypoints_.toString().c_str());

      // Call the inverse kinematics
      human_model_.ik(keypoints_, joint_limits_, configuration_, param_);  
    }
  }
}


void HumanKinematicsPublisher::publish_configuration_and_param()
{
  // Convert Eigen::VectorXd to std_msgs::msg::Float64MultiArray
  std_msgs::msg::Float64MultiArray configuration_msg;
  configuration_msg.data.resize(configuration_.size());
  Eigen::VectorXd::Map(&configuration_msg.data[0], configuration_.size()) = configuration_;

  std_msgs::msg::Float64MultiArray param_msg;
  param_msg.data.resize(param_.size());
  Eigen::VectorXd::Map(&param_msg.data[0], param_.size()) = param_;

  // Publish the messages
  configuration_publisher_->publish(configuration_msg);
  param_publisher_->publish(param_msg);
}


void HumanKinematicsPublisher::publish_transforms()
{
  RCLCPP_INFO(this->get_logger(), "Publishing transforms with timestamp: %f. Last stored header: %d.%d",
    this->get_clock()->now().seconds(), current_header_.stamp.sec, current_header_.stamp.nanosec);

  // Compute the transformations applying the forward kinematics
  human_model_.fk_tfs(
    configuration_,
    param_,
    T_ext_rshoulder,
    T_ext_lshoulder,
    T_ext_rhip,
    T_ext_lhip,
    T_ext_chest,
    T_ext_head,
    T_ext_relbow,
    T_ext_rwrist,
    T_ext_lelbow,
    T_ext_lwrist,
    T_ext_rknee,
    T_ext_rankle,
    T_ext_lknee,
    T_ext_lankle
  );

  // Store the transformations in the map
  body_transforms_["rshoulder"] = affine3d_to_tf(T_ext_rshoulder, "world", "rshoulder");
  body_transforms_["lshoulder"] = affine3d_to_tf(T_ext_lshoulder, "world", "lshoulder");
  body_transforms_["rhip"] = affine3d_to_tf(T_ext_rhip, "world", "rhip");
  body_transforms_["lhip"] = affine3d_to_tf(T_ext_lhip, "world", "lhip");
  body_transforms_["chest"] = affine3d_to_tf(T_ext_chest, "world", "chest");
  body_transforms_["head"] = affine3d_to_tf(T_ext_head, "world", "head");
  body_transforms_["relbow"] = affine3d_to_tf(T_ext_relbow, "world", "relbow");
  body_transforms_["rwrist"] = affine3d_to_tf(T_ext_rwrist, "world", "rwrist");
  body_transforms_["lelbow"] = affine3d_to_tf(T_ext_lelbow, "world", "lelbow");
  body_transforms_["lwrist"] = affine3d_to_tf(T_ext_lwrist, "world", "lwrist");
  body_transforms_["rknee"] = affine3d_to_tf(T_ext_rknee, "world", "rknee");
  body_transforms_["rankle"] = affine3d_to_tf(T_ext_rankle, "world", "rankle");
  body_transforms_["lknee"] = affine3d_to_tf(T_ext_lknee, "world", "lknee");
  body_transforms_["lankle"] = affine3d_to_tf(T_ext_lankle, "world", "lankle");

  // Publish all transformations
  for (const auto& transform : body_transforms_)
  {
    tf_broadcaster_->sendTransform(transform.second);
  }
}


geometry_msgs::msg::TransformStamped HumanKinematicsPublisher::affine3d_to_tf(const Eigen::Affine3d& T,
                                                                              const std::string& parent_frame,
                                                                              const std::string& child_frame)
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;

  transform.transform.translation.x = T.translation().x();
  transform.transform.translation.y = T.translation().y();
  transform.transform.translation.z = T.translation().z();

  Eigen::Quaterniond q(T.linear());
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  return transform;
}

} // end namespace human_kinematics_ros