#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <fstream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

namespace ros_template_node {

class PerformanceMonitor
{
public:
  explicit PerformanceMonitor(rclcpp::Node* node);
  ~PerformanceMonitor() = default;

  // Performance tracking methods
  void record_publish(const std::string& topic);
  void record_callback_start();
  void record_callback_end();
  void record_message_received(const std::string& topic);

  // Metrics collection
  void publish_metrics();
  nlohmann::json get_metrics_json();

private:
  // Node reference
  rclcpp::Node* node_;

  // Timing tracking
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point callback_start_time_;
  std::deque<double> callback_times_;
  mutable std::mutex metrics_mutex_;

  // Message counters
  std::atomic<size_t> total_messages_published_{0};
  std::atomic<size_t> total_messages_received_{0};
  std::unordered_map<std::string, size_t> topic_publish_counts_;
  std::unordered_map<std::string, size_t> topic_receive_counts_;

  // ROS integration
  rclcpp::TimerBase::SharedPtr metrics_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      metrics_publisher_;

  // System metrics
  double get_cpu_usage();
  size_t get_memory_usage_mb();
  double get_uptime_seconds();

  // Metrics calculation
  double calculate_average_callback_time();
  double calculate_max_callback_time();
  double calculate_publish_rate();

  // Internal tracking
  std::chrono::steady_clock::time_point last_metrics_time_;
  size_t last_published_count_{0};
};

}  // namespace ros_template_node