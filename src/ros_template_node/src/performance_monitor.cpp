#include "ros_template_node/performance_monitor.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace ros_template_node {

PerformanceMonitor::PerformanceMonitor(rclcpp::Node* node)
    : node_(node),
      start_time_(std::chrono::steady_clock::now()),
      last_metrics_time_(std::chrono::steady_clock::now())
{
  // Create metrics publisher
  metrics_publisher_ =
      node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "performance_metrics", 10);

  // Create metrics timer (10 second intervals)
  metrics_timer_ = node_->create_wall_timer(
      std::chrono::seconds(10), [this]() { this->publish_metrics(); });
}

void PerformanceMonitor::record_publish(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  total_messages_published_++;
  topic_publish_counts_[topic]++;
}

void PerformanceMonitor::record_callback_start()
{
  callback_start_time_ = std::chrono::steady_clock::now();
}

void PerformanceMonitor::record_callback_end()
{
  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                      end_time - callback_start_time_)
                      .count() /
                  1000.0;  // Convert to milliseconds

  std::lock_guard<std::mutex> lock(metrics_mutex_);
  callback_times_.push_back(duration);

  // Keep only last 100 measurements
  if (callback_times_.size() > 100) {
    callback_times_.pop_front();
  }
}

void PerformanceMonitor::record_message_received(const std::string& topic)
{
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  total_messages_received_++;
  topic_receive_counts_[topic]++;
}

double PerformanceMonitor::get_cpu_usage()
{
  static std::chrono::steady_clock::time_point last_time =
      std::chrono::steady_clock::now();
  static long long last_total_time = 0;
  static long long last_idle_time = 0;

  std::ifstream stat_file("/proc/stat");
  std::string line;
  if (!std::getline(stat_file, line)) {
    return 0.0;
  }

  std::istringstream iss(line);
  std::string cpu;
  long long user, nice, system, idle, iowait, irq, softirq, steal;

  iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >>
      steal;

  long long total_time =
      user + nice + system + idle + iowait + irq + softirq + steal;
  long long current_idle = idle + iowait;

  auto current_time = std::chrono::steady_clock::now();
  auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                       current_time - last_time)
                       .count();

  double cpu_percent = 0.0;
  if (time_diff > 500 &&
      last_total_time > 0) {  // Only calculate if >500ms passed
    long long total_diff = total_time - last_total_time;
    long long idle_diff = current_idle - last_idle_time;

    if (total_diff > 0) {
      cpu_percent = 100.0 * (total_diff - idle_diff) / total_diff;
    }

    last_time = current_time;
    last_total_time = total_time;
    last_idle_time = current_idle;
  }

  return std::max(0.0, std::min(100.0, cpu_percent));
}

size_t PerformanceMonitor::get_memory_usage_mb()
{
  std::ifstream status_file("/proc/self/status");
  std::string line;

  while (std::getline(status_file, line)) {
    if (line.find("VmRSS:") == 0) {
      std::istringstream iss(line);
      std::string label, value, unit;
      iss >> label >> value >> unit;
      return std::stoull(value) / 1024;  // Convert KB to MB
    }
  }
  return 0;
}

double PerformanceMonitor::get_uptime_seconds()
{
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::seconds>(now - start_time_)
      .count();
}

double PerformanceMonitor::calculate_average_callback_time()
{
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  if (callback_times_.empty()) return 0.0;

  double sum = 0.0;
  for (double time : callback_times_) {
    sum += time;
  }
  return sum / callback_times_.size();
}

double PerformanceMonitor::calculate_max_callback_time()
{
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  if (callback_times_.empty()) return 0.0;

  return *std::max_element(callback_times_.begin(), callback_times_.end());
}

double PerformanceMonitor::calculate_publish_rate()
{
  auto current_time = std::chrono::steady_clock::now();
  auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
                       current_time - last_metrics_time_)
                       .count();

  if (time_diff <= 0) return 0.0;

  size_t current_count = total_messages_published_.load();
  double rate =
      static_cast<double>(current_count - last_published_count_) / time_diff;

  last_metrics_time_ = current_time;
  last_published_count_ = current_count;

  return rate;
}

nlohmann::json PerformanceMonitor::get_metrics_json()
{
  std::lock_guard<std::mutex> lock(metrics_mutex_);

  nlohmann::json metrics;
  metrics["system"]["cpu_percent"] = get_cpu_usage();
  metrics["system"]["memory_mb"] = get_memory_usage_mb();
  metrics["system"]["uptime_seconds"] = get_uptime_seconds();

  metrics["ros"]["messages_published_total"] = total_messages_published_.load();
  metrics["ros"]["messages_received_total"] = total_messages_received_.load();
  metrics["ros"]["publish_rate_hz"] = calculate_publish_rate();
  metrics["ros"]["avg_callback_ms"] = calculate_average_callback_time();
  metrics["ros"]["max_callback_ms"] = calculate_max_callback_time();

  // Topic-specific metrics
  nlohmann::json topics;
  for (const auto& [topic, count] : topic_publish_counts_) {
    topics[topic]["published_count"] = count;
  }
  for (const auto& [topic, count] : topic_receive_counts_) {
    topics[topic]["received_count"] = count;
  }
  metrics["topics"] = topics;

  return metrics;
}

void PerformanceMonitor::publish_metrics()
{
  auto metrics_json = get_metrics_json();

  // Create diagnostic message
  auto diagnostic_array = diagnostic_msgs::msg::DiagnosticArray();
  diagnostic_array.header.stamp = node_->get_clock()->now();

  auto status = diagnostic_msgs::msg::DiagnosticStatus();
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = std::string(node_->get_name()) + "_performance";
  status.message = "Performance metrics";
  status.hardware_id = "performance_monitor";

  // Add key metrics as key-value pairs
  auto add_kv = [&](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };

  add_kv("cpu_percent", std::to_string(static_cast<double>(
                            metrics_json["system"]["cpu_percent"])));
  add_kv(
      "memory_mb",
      std::to_string(static_cast<size_t>(metrics_json["system"]["memory_mb"])));
  add_kv("uptime_seconds", std::to_string(static_cast<double>(
                               metrics_json["system"]["uptime_seconds"])));
  add_kv("messages_published_total",
         std::to_string(static_cast<size_t>(
             metrics_json["ros"]["messages_published_total"])));
  add_kv("publish_rate_hz", std::to_string(static_cast<double>(
                                metrics_json["ros"]["publish_rate_hz"])));
  add_kv("avg_callback_ms", std::to_string(static_cast<double>(
                                metrics_json["ros"]["avg_callback_ms"])));

  diagnostic_array.status.push_back(status);
  metrics_publisher_->publish(diagnostic_array);
}

}  // namespace ros_template_node