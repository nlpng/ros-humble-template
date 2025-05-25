#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class TemplateNode : public rclcpp::Node 
{
public:
  TemplateNode();

private:
  void timer_callback();
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr counter_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;

  // Data
  size_t count_;
};
