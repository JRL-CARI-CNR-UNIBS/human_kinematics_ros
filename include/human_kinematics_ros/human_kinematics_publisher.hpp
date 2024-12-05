#pragma once

#include "human_model/human_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zed_interfaces/msg/skeleton3_d.hpp"


class SkeletonSubscriber : public rclcpp::Node
{
public:
  SkeletonSubscriber()
  : Node("skeleton_subscriber")
  {
    subscription_ = this->create_subscription<zed_interfaces::msg::MISSINGGGGGGGGGGGGGGGGGGGGGG>(
      "/zed/zed_node/body_trk/skeletons", 10,
      std::bind(&SkeletonSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


class HumanKinematicsPublisher : public rclcpp::Node
{
public:
  HumanKinematicsPublisher()
  : Node("human_kinematics_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/zed/zed_node/body_trk/skeletons", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&HumanKinematicsPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world!";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
