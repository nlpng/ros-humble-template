#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_template_node/template_node.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TemplateNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TemplateNode>();
  }

  void TearDown() override {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<TemplateNode> node_;
};

TEST_F(TemplateNodeTest, NodeCreation) {
  ASSERT_NE(node_, nullptr);
  EXPECT_STREQ(node_->get_name(), "template_node");
}

TEST_F(TemplateNodeTest, ParametersExist) {
  // Test that parameters are declared
  EXPECT_TRUE(node_->has_parameter("publish_rate"));
  EXPECT_TRUE(node_->has_parameter("topic_prefix"));

  // Test default values
  auto publish_rate = node_->get_parameter("publish_rate").as_double();
  auto topic_prefix = node_->get_parameter("topic_prefix").as_string();

  EXPECT_GT(publish_rate, 0.0);
  EXPECT_FALSE(topic_prefix.empty());
}

TEST_F(TemplateNodeTest, PublishersCreated) {
  // Give the node time to initialize
  rclcpp::spin_some(node_);

  // Check that publishers are created by checking topic count
  auto topic_names = node_->get_topic_names_and_types();

  // Should have at least 3 topics (status, counter, temperature)
  // Note: actual topic names depend on the topic_prefix parameter
  bool has_status_topic = false;
  bool has_counter_topic = false;
  bool has_temperature_topic = false;

  for (const auto &topic : topic_names) {
    if (topic.first.find("status") != std::string::npos) {
      has_status_topic = true;
    }
    if (topic.first.find("counter") != std::string::npos) {
      has_counter_topic = true;
    }
    if (topic.first.find("temperature") != std::string::npos) {
      has_temperature_topic = true;
    }
  }

  EXPECT_TRUE(has_status_topic);
  EXPECT_TRUE(has_counter_topic);
  EXPECT_TRUE(has_temperature_topic);
}

TEST_F(TemplateNodeTest, ParameterUpdate) {
  // Test parameter update
  auto param = rclcpp::Parameter("publish_rate", 5.0);
  node_->set_parameter(param);

  auto updated_rate = node_->get_parameter("publish_rate").as_double();
  EXPECT_DOUBLE_EQ(updated_rate, 5.0);
}

class MessageReceiver : public rclcpp::Node {
public:
  MessageReceiver() : Node("message_receiver") {
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "template/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          last_status_msg_ = msg->data;
          status_received_ = true;
        });

    counter_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "template/counter", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          last_counter_value_ = msg->data;
          counter_received_ = true;
        });
  }

  std::string last_status_msg_;
  int last_counter_value_ = -1;
  bool status_received_ = false;
  bool counter_received_ = false;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr counter_sub_;
};

TEST_F(TemplateNodeTest, MessagePublishing) {
  auto receiver = std::make_shared<MessageReceiver>();

  // Spin both nodes for a short time to allow message publishing
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 3s) {
    rclcpp::spin_some(node_);
    rclcpp::spin_some(receiver);
    std::this_thread::sleep_for(10ms);

    if (receiver->status_received_ && receiver->counter_received_) {
      break;
    }
  }

  // Verify messages were received
  EXPECT_TRUE(receiver->status_received_);
  EXPECT_TRUE(receiver->counter_received_);
  EXPECT_FALSE(receiver->last_status_msg_.empty());
  EXPECT_GE(receiver->last_counter_value_, 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
