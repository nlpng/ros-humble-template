#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_template_node/template_node.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TemplateNodeIntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TemplateNode>();

    // Create a publisher to send commands to the node
    cmd_publisher_ = std::make_shared<rclcpp::Node>("test_commander");
    cmd_pub_ = cmd_publisher_->create_publisher<geometry_msgs::msg::Twist>(
        "template/cmd_vel", 10);

    // Create subscriber to monitor temperature messages
    temp_subscriber_ = std::make_shared<rclcpp::Node>("test_temp_subscriber");
    temp_sub_ =
        temp_subscriber_->create_subscription<sensor_msgs::msg::Temperature>(
            "template/temperature", 10,
            [this](const sensor_msgs::msg::Temperature::SharedPtr msg) {
              last_temperature_ = msg->temperature;
              temp_received_ = true;
            });
  }

  void TearDown() override {
    node_.reset();
    cmd_publisher_.reset();
    temp_subscriber_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<TemplateNode> node_;
  rclcpp::Node::SharedPtr cmd_publisher_;
  rclcpp::Node::SharedPtr temp_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;

  double last_temperature_ = 0.0;
  bool temp_received_ = false;
};

TEST_F(TemplateNodeIntegrationTest, CommandVelocityProcessing) {
  // Create a twist message
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 1.0;
  twist_msg.angular.z = 0.5;

  // Publish the command
  cmd_pub_->publish(twist_msg);

  // Spin nodes to process the message
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 1s) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(cmd_publisher_);
    std::this_thread::sleep_for(10ms);
  }

  // Test passes if no exceptions are thrown during message processing
  SUCCEED();
}

TEST_F(TemplateNodeIntegrationTest, TemperaturePublishing) {
  // Spin nodes to allow temperature publishing
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 5s) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(temp_subscriber_);
    std::this_thread::sleep_for(100ms);

    if (temp_received_) {
      break;
    }
  }

  // Verify temperature message was received
  EXPECT_TRUE(temp_received_);

  // Temperature should be in a reasonable range (simulated values are around
  // 15-25°C)
  EXPECT_GT(last_temperature_, 10.0);
  EXPECT_LT(last_temperature_, 30.0);
}

TEST_F(TemplateNodeIntegrationTest, NodeParameterConfiguration) {
  // Test changing parameters while node is running
  auto param = rclcpp::Parameter("topic_prefix", "test_prefix");
  auto result = node_->set_parameter(param);

  EXPECT_TRUE(result.successful);

  auto updated_prefix = node_->get_parameter("topic_prefix").as_string();
  EXPECT_EQ(updated_prefix, "test_prefix");
}

TEST_F(TemplateNodeIntegrationTest, MultipleMessageTypes) {
  // Test that the node publishes multiple message types
  int status_count = 0;
  int counter_count = 0;
  int temp_count = 0;

  auto status_sub =
      temp_subscriber_->create_subscription<std_msgs::msg::String>(
          "template/status", 10,
          [&status_count](const std_msgs::msg::String::SharedPtr) {
            status_count++;
          });

  auto counter_sub =
      temp_subscriber_->create_subscription<std_msgs::msg::Int32>(
          "template/counter", 10,
          [&counter_count](const std_msgs::msg::Int32::SharedPtr) {
            counter_count++;
          });

  auto temp_sub =
      temp_subscriber_->create_subscription<sensor_msgs::msg::Temperature>(
          "template/temperature", 10,
          [&temp_count](const sensor_msgs::msg::Temperature::SharedPtr) {
            temp_count++;
          });

  // Run for a few seconds to collect messages
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 3s) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(temp_subscriber_);
    std::this_thread::sleep_for(100ms);
  }

  // All message types should have been published
  EXPECT_GT(status_count, 0);
  EXPECT_GT(counter_count, 0);
  EXPECT_GT(temp_count, 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
