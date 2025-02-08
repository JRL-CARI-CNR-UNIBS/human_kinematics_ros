#include <human_kinematics_ros/human_kinematics_publisher.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace human_kinematics_ros
{

HumanKinematicsPublisher::HumanKinematicsPublisher() : Node("human_kinematics_publisher")
{
  // Declare parameters
  this->declare_parameter<std::string>("origin_frame", origin_frame_);
  this->declare_parameter<std::string>("camera_frame", camera_frame_);
  this->declare_parameter<std::string>("configuration_topic_name", configuration_topic_name_);
  this->declare_parameter<std::string>("param_topic_name", param_topic_name_);
  this->declare_parameter<std::string>("chest_q_rotated_topic_name", chest_q_rotated_topic_name_);
  this->declare_parameter<std::string>("keypoints_topic_name", keypoints_topic_name_);

  // Get parameters
  if(this->get_parameter("origin_frame", origin_frame_) && !origin_frame_.empty())
    RCLCPP_INFO(this->get_logger(), "Parameter origin_frame: %s", origin_frame_.c_str());
  else
    RCLCPP_FATAL(this->get_logger(), "Failed to get parameter origin_frame");

  if(this->get_parameter("camera_frame", camera_frame_) && !camera_frame_.empty())
    RCLCPP_INFO(this->get_logger(), "Parameter camera_frame: %s", camera_frame_.c_str());
  else
    RCLCPP_FATAL(this->get_logger(), "Failed to get parameter camera_frame");

  if(this->get_parameter("configuration_topic_name", configuration_topic_name_) && !configuration_topic_name_.empty())
    RCLCPP_INFO(this->get_logger(), "Parameter configuration_topic_name: %s", configuration_topic_name_.c_str());
  else
    RCLCPP_FATAL(this->get_logger(), "Failed to get parameter configuration_topic_name");

  if(this->get_parameter("param_topic_name", param_topic_name_) && !param_topic_name_.empty())
    RCLCPP_INFO(this->get_logger(), "Parameter param_topic_name: %s", param_topic_name_.c_str());
  else
    RCLCPP_FATAL(this->get_logger(), "Failed to get parameter param_topic_name");

  if(this->get_parameter("chest_q_rotated_topic_name", chest_q_rotated_topic_name_) && !chest_q_rotated_topic_name_.empty())
    RCLCPP_INFO(this->get_logger(), "Parameter chest_q_rotated_topic_name: %s", chest_q_rotated_topic_name_.c_str());
  else
    RCLCPP_FATAL(this->get_logger(), "Failed to get parameter chest_q_rotated_topic_name");
  
  if(this->get_parameter("keypoints_topic_name", keypoints_topic_name_) && !keypoints_topic_name_.empty())
    RCLCPP_INFO(this->get_logger(), "Parameter keypoints_topic_name: %s", keypoints_topic_name_.c_str());
  else
    RCLCPP_FATAL(this->get_logger(), "Failed to get parameter keypoints_topic_name");

  // Subscribe to the ZED skeleton topic
  skeleton_subscription_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
    keypoints_topic_name_, 10,
    std::bind(&HumanKinematicsPublisher::skeleton_callback, this, std::placeholders::_1));
  
  // Initialize the transformation matrix from the camera frame to the world frame
  tf2_ros::Buffer tf_buffer(this->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    // Add a delay to ensure the transform is available
    std::this_thread::sleep_for(std::chrono::seconds(1));

    transform_stamped = tf_buffer.lookupTransform(origin_frame_, camera_frame_, tf2::TimePointZero);
    T_world_camera_.translation() << transform_stamped.transform.translation.x,
                                     transform_stamped.transform.translation.y,
                                     transform_stamped.transform.translation.z;
    Eigen::Quaterniond q(transform_stamped.transform.rotation.w,
                         transform_stamped.transform.rotation.x,
                         transform_stamped.transform.rotation.y,
                         transform_stamped.transform.rotation.z);
    T_world_camera_.linear() = q.toRotationMatrix();
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "While creating T_world_camera_. Could not transform: %s", ex.what());
  }

  // Instantiate the human model
  human_model_ = human_model::Human28DOF();

  // Set the joint limits
  joint_limits_.resize(N_DOF, human_model::JointLimits(-M_PI, M_PI));

  // Set the chest translation and rotation limits -> NOT USED BY THE HUMAN MODEL
  joint_limits_[0] = human_model::JointLimits( 0.0, 2.0);               // q0_chest_x
  joint_limits_[1] = human_model::JointLimits(-2.5, 2.5);               // q1_chest_y
  joint_limits_[2] = human_model::JointLimits( 0.8, 1.8);               // q2_chest_z
  joint_limits_[3] = human_model::JointLimits(-1.0, 1.0);               // q3_chest_qx
  joint_limits_[4] = human_model::JointLimits(-1.0, 1.0);               // q4_chest_qy
  joint_limits_[5] = human_model::JointLimits(-1.0, 1.0);               // q5_chest_qz
  joint_limits_[6] = human_model::JointLimits(-1.0, 1.0);               // q6_chest_qw
  
  // Set the shoulder axis limits    
  joint_limits_[7] = human_model::JointLimits(-M_PI / 4, M_PI / 4);     // q7_shoulder_rotx
  
  // Set the hip axis limits    
  joint_limits_[8] = human_model::JointLimits(-M_PI / 2, M_PI / 2);     // q8_hip_rotz
  joint_limits_[9] = human_model::JointLimits(-M_PI / 4, M_PI / 4);     // q9_hip_rotx

  // Set the right and left shoulder limits 
  joint_limits_[10] = human_model::JointLimits(-M_PI, M_PI/2 );         // q10_rshoulder_rotz
  joint_limits_[11] = human_model::JointLimits(-M_PI, 0.0);             // q11_rshoulder_rotx
  joint_limits_[12] = human_model::JointLimits(-0.75*M_PI, 0.75*M_PI);  // q12_rshoulder_roty
  joint_limits_[14] = human_model::JointLimits(-M_PI, M_PI/2 );         // q14_lshoulder_rotz
  joint_limits_[15] = human_model::JointLimits(-M_PI, 0.0);             // q15_lshoulder_rotx
  joint_limits_[16] = human_model::JointLimits(-0.75*M_PI, 0.75*M_PI);  // q16_lshoulder_roty
  
  // Set the right and left elbow limits 
  joint_limits_[13] = human_model::JointLimits(-M_PI, 0.0);             // q13_relbow_rotz
  joint_limits_[17] = human_model::JointLimits(-M_PI, 0.0);             // q17_lelbow_rotz

  // Set the right and left hip limits 
  joint_limits_[18] = human_model::JointLimits(-M_PI/4, M_PI *.75);     // q18_rhip_rotz
  joint_limits_[19] = human_model::JointLimits(-M_PI / 2, 0.0);         // q19_rhip_rotx
  joint_limits_[20] = human_model::JointLimits(0.0, M_PI / 2);          // q20_rhip_roty
  joint_limits_[22] = human_model::JointLimits(-M_PI/4, M_PI *.75);     // q22_lhip_rotz
  joint_limits_[23] = human_model::JointLimits(-M_PI, 0.0);             // q23_lhip_rotx
  joint_limits_[24] = human_model::JointLimits(0.0, M_PI / 2);          // q24_lhip_roty

  // Set the right and left knee limits 
  joint_limits_[21] = human_model::JointLimits(0.0, M_PI);              // q21_rknee_rotz
  joint_limits_[25] = human_model::JointLimits(0.0, M_PI);              // q25_lknee_rotz

  // Set the head limits
  joint_limits_[26] = human_model::JointLimits(-M_PI / 2, M_PI / 2);    // q26_head_rotx
  joint_limits_[27] = human_model::JointLimits(-M_PI / 2, M_PI / 2);    // q27_head_roty

  // Publish the configuration and param messages
  configuration_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/"+configuration_topic_name_, 10);
  param_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/"+param_topic_name_, 10);
  chest_q_rotated_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/"+chest_q_rotated_topic_name_, 10);
  configuration_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&HumanKinematicsPublisher::publish_configuration_and_param, this)
  );

  // Publish transformations
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&HumanKinematicsPublisher::publish_transforms, this)
  );
}


void HumanKinematicsPublisher::skeleton_callback(const zed_msgs::msg::ObjectsStamped::ConstSharedPtr msg)
{
  if (msg->objects.size() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Received empty ObjectsStamped message");
    return;
  }
  // RCLCPP_INFO(this->get_logger(), "Received ObjectsStamped message with %zu objects", msg->objects.size());
  
  // Store the current header
  current_header_ = msg->header;

  // Iterate over the objects
  // #TODO: for now, it only considers the last object since the zed_kpts_ vector is overwritten
  for (const auto& object : msg->objects)
  {
    RCLCPP_INFO(this->get_logger(), "Object ID: %d", object.label_id);
    
    if (object.skeleton_available)
    {
      // Get the raw keypoints
      zed_kpts_ = std::vector<zed_msgs::msg::Keypoint3D>(
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
        Eigen::Vector3d kpt_world = T_world_camera_ * kpt_camera;
        kpt.kp[0] = kpt_world[0];
        kpt.kp[1] = kpt_world[1];
        kpt.kp[2] = kpt_world[2];
      }

      // Find the center of the head as midpoint between the two ears
      Eigen::Vector3d head_center;
      head_center.x() = 0.5 * (zed_kpts_[16].kp[0] + zed_kpts_[17].kp[0]);
      head_center.y() = 0.5 * (zed_kpts_[16].kp[1] + zed_kpts_[17].kp[1]);
      head_center.z() = 0.5 * (zed_kpts_[16].kp[2] + zed_kpts_[17].kp[2]);
      
      // Select the appropriate keypoints to pass to the human model
      keypoints_.head = head_center;
        
      keypoints_.left_shoulder  << zed_kpts_[5].kp[0],  zed_kpts_[5].kp[1],  zed_kpts_[5].kp[2];
      keypoints_.left_elbow     << zed_kpts_[6].kp[0],  zed_kpts_[6].kp[1],  zed_kpts_[6].kp[2];
      keypoints_.left_wrist     << zed_kpts_[7].kp[0],  zed_kpts_[7].kp[1],  zed_kpts_[7].kp[2];
        
      keypoints_.left_hip       << zed_kpts_[11].kp[0], zed_kpts_[11].kp[1], zed_kpts_[11].kp[2];
      keypoints_.left_knee      << zed_kpts_[12].kp[0], zed_kpts_[12].kp[1], zed_kpts_[12].kp[2];
      keypoints_.left_ankle     << zed_kpts_[13].kp[0], zed_kpts_[13].kp[1], zed_kpts_[13].kp[2];
       
      keypoints_.right_shoulder << zed_kpts_[2].kp[0],  zed_kpts_[2].kp[1],  zed_kpts_[2].kp[2];
      keypoints_.right_elbow    << zed_kpts_[3].kp[0],  zed_kpts_[3].kp[1],  zed_kpts_[3].kp[2];
      keypoints_.right_wrist    << zed_kpts_[4].kp[0],  zed_kpts_[4].kp[1],  zed_kpts_[4].kp[2];

      keypoints_.right_hip      << zed_kpts_[8].kp[0],  zed_kpts_[8].kp[1],  zed_kpts_[8].kp[2];
      keypoints_.right_knee     << zed_kpts_[9].kp[0],  zed_kpts_[9].kp[1],  zed_kpts_[9].kp[2];
      keypoints_.right_ankle    << zed_kpts_[10].kp[0], zed_kpts_[10].kp[1], zed_kpts_[10].kp[2];

      // RCLCPP_INFO(this->get_logger(), "Keypoints [WORLD FRAME]:\n%s", keypoints_.toString().c_str());

      // Call the inverse kinematics
      const Eigen::VectorXd prev_configuration = configuration_;
      human_model_.ik(keypoints_, joint_limits_, prev_configuration, configuration_, param_, chest_q_rotated_);  

      // Compute the transformations applying the forward kinematics
      human_model_.fk_tfs(
        configuration_,
        param_,
        T_ext_rshoulder_,
        T_ext_lshoulder_,
        T_ext_rhip_,
        T_ext_lhip_,
        T_ext_chest_,
        T_ext_head_,
        T_ext_rshoulderRotated_,
        T_ext_relbow_,
        T_ext_rwrist_,
        T_ext_lshoulderRotated_,
        T_ext_lelbow_,
        T_ext_lwrist_,
        T_ext_rhipRotated_,
        T_ext_rknee_,
        T_ext_rankle_,
        T_ext_lhipRotated_,
        T_ext_lknee_,
        T_ext_lankle_
      );
    }
  }
}


void HumanKinematicsPublisher::publish_configuration_and_param()
{
  // Create a JointState message for the configuration data
  sensor_msgs::msg::JointState config_msg;
  config_msg.header.stamp = this->now();
  config_msg.name = {
    "q0_chest_x",
    "q1_chest_y",
    "q2_chest_z",
    "q3_chest_qx",
    "q4_chest_qy",
    "q5_chest_qz",
    "q6_chest_qw",
    "q7_shoulder_rotx",
    "q8_hip_rotz",
    "q9_hip_rotx",
    "q10_rshoulder_rotz",
    "q11_rshoulder_rotx",
    "q12_rshoulder_roty",
    "q13_relbow_rotz",
    "q14_lshoulder_rotz",
    "q15_lshoulder_rotx",
    "q16_lshoulder_roty",
    "q17_lelbow_rotz",
    "q18_rhip_rotz",
    "q19_rhip_rotx",
    "q20_rhip_roty",
    "q21_rknee_rotz",
    "q22_lhip_rotz",
    "q23_lhip_rotx",
    "q24_lhip_roty",
    "q25_lknee_rotz",
    "q26_head_rotx",
    "q27_head_roty"
  };
  config_msg.position.resize(configuration_.size());
  Eigen::VectorXd::Map(&config_msg.position[0], configuration_.size()) = configuration_;

  // Create a JointState message for the param data
  sensor_msgs::msg::JointState param_msg;
  param_msg.header.stamp = this->now();
  param_msg.name = {
    "shoulder_distance",
    "chest_hip_distance",
    "hip_distance",
    "upper_arm_length",
    "lower_arm_length",
    "upper_leg_length",
    "lower_leg_length",
    "head_distance",
  };
  param_msg.position.resize(param_.size());
  Eigen::VectorXd::Map(&param_msg.position[0], param_.size()) = param_;

  // Create a JointState message for the chest_q_rotated data
  sensor_msgs::msg::JointState chest_q_rotated_msg;
  chest_q_rotated_msg.header.stamp = this->now();
  chest_q_rotated_msg.name = {
    "chest_q_rotated_x",
    "chest_q_rotated_y",
    "chest_q_rotated_z",
    "chest_q_rotated_w"
  };
  chest_q_rotated_msg.position.resize(chest_q_rotated_.size());
  Eigen::VectorXd::Map(&chest_q_rotated_msg.position[0], chest_q_rotated_.size()) = chest_q_rotated_;

  // Publish the messages
  configuration_publisher_->publish(config_msg);
  param_publisher_->publish(param_msg);
  chest_q_rotated_publisher_->publish(chest_q_rotated_msg);
}


void HumanKinematicsPublisher::publish_transforms()
{
  // RCLCPP_INFO(this->get_logger(), "Publishing transforms with timestamp: %f. Last stored header: %d.%d",
  //   this->get_clock()->now().seconds(), current_header_.stamp.sec, current_header_.stamp.nanosec);

  // // [DEBUG] Function to convert Eigen matrix to string
  // auto matrix_to_string = [](const Eigen::Matrix4d& matrix) {
  //     std::stringstream ss;
  //     ss << matrix.format(Eigen::IOFormat(Eigen::FullPrecision));
  //     return ss.str();
  // };

  // // [DEBUG] Print the transformation matrices
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_rshoulder_:\n%s", matrix_to_string(T_ext_rshoulder_.matrix()).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_lshoulder_:\n%s", matrix_to_string(T_ext_lshoulder_.matrix()).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_rhip_:\n%s",      matrix_to_string(T_ext_rhip_.matrix()     ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_lhip_:\n%s",      matrix_to_string(T_ext_lhip_.matrix()     ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_chest_:\n%s",     matrix_to_string(T_ext_chest_.matrix()    ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_head_:\n%s",      matrix_to_string(T_ext_head_.matrix()     ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_relbow_:\n%s",    matrix_to_string(T_ext_relbow_.matrix()   ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_rwrist_:\n%s",    matrix_to_string(T_ext_rwrist_.matrix()   ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_lelbow_:\n%s",    matrix_to_string(T_ext_lelbow_.matrix()   ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_lwrist_:\n%s",    matrix_to_string(T_ext_lwrist_.matrix()   ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_rknee_:\n%s",     matrix_to_string(T_ext_rknee_.matrix()    ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_rankle_:\n%s",    matrix_to_string(T_ext_rankle_.matrix()   ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_lknee_:\n%s",     matrix_to_string(T_ext_lknee_.matrix()    ).c_str());
  // RCLCPP_INFO(this->get_logger(), "\nT_ext_lankle_:\n%s",    matrix_to_string(T_ext_lankle_.matrix()   ).c_str());

  // Store the transformations in the map
  body_transforms_["rshoulder"]        = affine3d_to_tf(T_ext_rshoulder_,        origin_frame_, "rshoulder",        body_transforms_nan_["rshoulder"]);
  body_transforms_["lshoulder"]        = affine3d_to_tf(T_ext_lshoulder_,        origin_frame_, "lshoulder",        body_transforms_nan_["lshoulder"]);
  body_transforms_["rhip"]             = affine3d_to_tf(T_ext_rhip_,             origin_frame_, "rhip",             body_transforms_nan_["rhip"]);
  body_transforms_["lhip"]             = affine3d_to_tf(T_ext_lhip_,             origin_frame_, "lhip",             body_transforms_nan_["lhip"]);
  body_transforms_["chest"]            = affine3d_to_tf(T_ext_chest_,            origin_frame_, "chest",            body_transforms_nan_["chest"]);
  body_transforms_["head"]             = affine3d_to_tf(T_ext_head_,             origin_frame_, "head",             body_transforms_nan_["head"]);
  body_transforms_["rshoulderRotated"] = affine3d_to_tf(T_ext_rshoulderRotated_, origin_frame_, "rshoulderRotated", body_transforms_nan_["rshoulderRotated"]);
  body_transforms_["relbow"]           = affine3d_to_tf(T_ext_relbow_,           origin_frame_, "relbow",           body_transforms_nan_["relbow"]);
  body_transforms_["rwrist"]           = affine3d_to_tf(T_ext_rwrist_,           origin_frame_, "rwrist",           body_transforms_nan_["rwrist"]);
  body_transforms_["lshoulderRotated"] = affine3d_to_tf(T_ext_lshoulderRotated_, origin_frame_, "lshoulderRotated", body_transforms_nan_["lshoulderRotated"]);
  body_transforms_["lelbow"]           = affine3d_to_tf(T_ext_lelbow_,           origin_frame_, "lelbow",           body_transforms_nan_["lelbow"]);
  body_transforms_["lwrist"]           = affine3d_to_tf(T_ext_lwrist_,           origin_frame_, "lwrist",           body_transforms_nan_["lwrist"]);
  body_transforms_["rknee"]            = affine3d_to_tf(T_ext_rknee_,            origin_frame_, "rknee",            body_transforms_nan_["rknee"]);
  body_transforms_["rankle"]           = affine3d_to_tf(T_ext_rankle_,           origin_frame_, "rankle",           body_transforms_nan_["rankle"]);
  body_transforms_["lknee"]            = affine3d_to_tf(T_ext_lknee_,            origin_frame_, "lknee",            body_transforms_nan_["lknee"]);
  body_transforms_["lankle"]           = affine3d_to_tf(T_ext_lankle_,           origin_frame_, "lankle",           body_transforms_nan_["lankle"]);

  // Publish all transformations if they do not contain NaN values
  for (const auto& transform : body_transforms_)
  {
    if (body_transforms_nan_[transform.first])
      RCLCPP_WARN(this->get_logger(), "Transform T_ext_%s_ contains NaN values", transform.first.c_str());
    else
      tf_broadcaster_->sendTransform(transform.second);
  }
}


geometry_msgs::msg::TransformStamped HumanKinematicsPublisher::affine3d_to_tf(const Eigen::Affine3d& T,
                                                                              const std::string& parent_frame,
                                                                              const std::string& child_frame,
                                                                              bool& has_nan)
{
  if (T.matrix().hasNaN())
  {
    has_nan = true;
    return geometry_msgs::msg::TransformStamped();
  }

  has_nan = false;
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = std::string("human0_") + child_frame;

  transform.transform.translation.x = T.translation().x();
  transform.transform.translation.y = T.translation().y();
  transform.transform.translation.z = T.translation().z();

  Eigen::Quaterniond q(T.linear());
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  // RCLCPP_INFO(this->get_logger(), "Transform from %s to %s", parent_frame.c_str(), child_frame.c_str());
  // RCLCPP_INFO(this->get_logger(), "Rotation: x=%f, y=%f, z=%f, w=%f", q.x(), q.y(), q.z(), q.w());

  return transform;
}

} // end namespace human_kinematics_ros