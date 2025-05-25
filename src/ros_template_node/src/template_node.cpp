#include "ros_template_node/template_node.hpp"

#include <cmath>

TemplateNode::TemplateNode() : Node("template_node"), count_(0)
{
  // Declare parameters with default values
  this->declare_parameter("publish_rate", 1.0);
  this->declare_parameter("topic_prefix", "template");

  // Get parameter values
  auto publish_rate = this->get_parameter("publish_rate").as_double();
  auto topic_prefix = this->get_parameter("topic_prefix").as_string();

  // Create publishers
  string_publisher_ = this->create_publisher<std_msgs::msg::String>(
      topic_prefix + "/status", 10);

  counter_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
      topic_prefix + "/counter", 10);

  temperature_publisher_ =
      this->create_publisher<sensor_msgs::msg::Temperature>(
          topic_prefix + "/temperature", 10);

  // Create subscriber
  cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      topic_prefix + "/cmd_vel", 10,
      std::bind(&TemplateNode::cmd_callback, this, std::placeholders::_1));

  // Create timer
  auto timer_period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
  timer_ = this->create_wall_timer(
      timer_period, std::bind(&TemplateNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
              "Template node initialized with rate: %.2f Hz", publish_rate);
  RCLCPP_INFO(this->get_logger(), "Publishing to topics with prefix: %s",
              topic_prefix.c_str());
}

void TemplateNode::timer_callback()
{
  // Publish status message
  auto status_msg = std_msgs::msg::String();
  status_msg.data = "Template node running - count: " + std::to_string(count_);
  string_publisher_->publish(status_msg);

  // Publish counter
  auto counter_msg = std_msgs::msg::Int32();
  counter_msg.data = count_;
  counter_publisher_->publish(counter_msg);

  // Publish simulated temperature data
  auto temp_msg = sensor_msgs::msg::Temperature();
  temp_msg.header.stamp = this->get_clock()->now();
  temp_msg.header.frame_id = "base_link";
  temp_msg.temperature =
      20.0 + 5.0 * std::sin(count_ * 0.1);  // Simulated temperature
  temp_msg.variance = 0.1;
  temperature_publisher_->publish(temp_msg);

  if (count_ % 10 == 0) {
    RCLCPP_INFO(this->get_logger(), "Published count: %zu, temp: %.2f°C",
                count_, temp_msg.temperature);
  }

  count_++;
}

void TemplateNode::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),
              "Received cmd_vel - linear: [%.2f, %.2f, %.2f], angular: [%.2f, "
              "%.2f, %.2f]",
              msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x,
              msg->angular.y, msg->angular.z);

  // Example of processing received command
  if (std::abs(msg->linear.x) > 0.1 || std::abs(msg->angular.z) > 0.1) {
    RCLCPP_INFO(this->get_logger(), "Robot is moving!");
  }
}
