#include "ros_template_node/template_node.hpp"

#include <cmath>

TemplateNode::TemplateNode()
    : Node("template_node"), count_(0), start_time_(this->get_clock()->now())
{
  // Initialize structured logger
  structured_logger_ =
      std::make_unique<ros_template_node::StructuredLogger>("template_node");

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

  health_publisher_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
          topic_prefix + "/health", 10);

  // Create subscriber
  cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      topic_prefix + "/cmd_vel", 10,
      std::bind(&TemplateNode::cmd_callback, this, std::placeholders::_1));

  // Create timer
  auto timer_period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
  timer_ = this->create_wall_timer(
      timer_period, std::bind(&TemplateNode::timer_callback, this));

  // Create health timer (30 seconds)
  health_timer_ =
      this->create_wall_timer(std::chrono::seconds(30),
                              std::bind(&TemplateNode::health_callback, this));

  // Log structured initialization
  nlohmann::json init_context;
  init_context["publish_rate"] = publish_rate;
  init_context["topic_prefix"] = topic_prefix;
  structured_logger_->info(
      ros_template_node::StructuredLogger::Component::STARTUP,
      ros_template_node::StructuredLogger::EventType::INITIALIZATION,
      "Template node initialized successfully", init_context);

  RCLCPP_INFO(this->get_logger(),
              "Template node initialized with rate: %.2f Hz", publish_rate);
  RCLCPP_INFO(this->get_logger(), "Publishing to topics with prefix: %s",
              topic_prefix.c_str());
}

void TemplateNode::timer_callback()
{
  // Log timer event
  nlohmann::json timer_context;
  timer_context["count"] = count_;
  structured_logger_->debug(
      ros_template_node::StructuredLogger::Component::TIMER,
      ros_template_node::StructuredLogger::EventType::PUBLISH,
      "Timer callback executing", timer_context);

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
    nlohmann::json publish_context;
    publish_context["count"] = count_;
    publish_context["temperature"] = temp_msg.temperature;
    publish_context["topics"] =
        nlohmann::json::array({"status", "counter", "temperature"});
    structured_logger_->info(
        ros_template_node::StructuredLogger::Component::TIMER,
        ros_template_node::StructuredLogger::EventType::PUBLISH,
        "Periodic data published", publish_context);

    RCLCPP_INFO(this->get_logger(), "Published count: %zu", count_);
  }

  count_++;
}

void TemplateNode::health_callback()
{
  auto uptime = (this->get_clock()->now() - start_time_).seconds();

  // Log health check
  nlohmann::json health_context;
  health_context["uptime_seconds"] = static_cast<int>(uptime);
  health_context["message_count"] = count_;
  health_context["status"] = "operational";
  structured_logger_->info(
      ros_template_node::StructuredLogger::Component::HEALTH,
      ros_template_node::StructuredLogger::EventType::HEALTH_CHECK,
      "Node health check completed", health_context);

  auto health_msg = diagnostic_msgs::msg::DiagnosticStatus();
  health_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  health_msg.name = this->get_name();
  health_msg.message = "Node operational";
  health_msg.hardware_id = "template_node_container";

  diagnostic_msgs::msg::KeyValue uptime_kv;
  uptime_kv.key = "uptime_seconds";
  uptime_kv.value = std::to_string(static_cast<int>(uptime));
  health_msg.values.push_back(uptime_kv);

  diagnostic_msgs::msg::KeyValue count_kv;
  count_kv.key = "message_count";
  count_kv.value = std::to_string(count_);
  health_msg.values.push_back(count_kv);

  health_publisher_->publish(health_msg);
}

void TemplateNode::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Log structured command reception
  nlohmann::json cmd_context;
  cmd_context["linear_x"] = msg->linear.x;
  cmd_context["linear_y"] = msg->linear.y;
  cmd_context["linear_z"] = msg->linear.z;
  cmd_context["angular_x"] = msg->angular.x;
  cmd_context["angular_y"] = msg->angular.y;
  cmd_context["angular_z"] = msg->angular.z;
  structured_logger_->debug(
      ros_template_node::StructuredLogger::Component::SUBSCRIBER,
      ros_template_node::StructuredLogger::EventType::RECEIVE,
      "Command velocity received", cmd_context);

  RCLCPP_INFO(this->get_logger(),
              "Received cmd_vel - linear: [%.2f, %.2f, %.2f], angular: [%.2f, "
              "%.2f, %.2f]",
              msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x,
              msg->angular.y, msg->angular.z);

  // Example of processing received command
  if (std::abs(msg->linear.x) > 0.1 || std::abs(msg->angular.z) > 0.1) {
    nlohmann::json movement_context;
    movement_context["is_moving"] = true;
    movement_context["linear_x"] = msg->linear.x;
    movement_context["angular_z"] = msg->angular.z;
    structured_logger_->info(
        ros_template_node::StructuredLogger::Component::SUBSCRIBER,
        ros_template_node::StructuredLogger::EventType::RECEIVE,
        "Robot movement detected", movement_context);
    RCLCPP_INFO(this->get_logger(), "Robot is moving!");
  }
}
