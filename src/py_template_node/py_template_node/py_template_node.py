#!/usr/bin/env python3
"""Python template node for ROS 2 with publisher, subscriber, and timer functionality."""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Int32, String
from .structured_logger import StructuredLogger, Component, EventType


class PyTemplateNode(Node):
    """Python version of the template node with publisher, subscriber, and timer functionality."""

    def __init__(self):
        """Initialize the PyTemplateNode with publishers, subscribers, and timers."""
        super().__init__("py_template_node")

        # Declare parameters with default values
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("topic_prefix", "py_template")

        # Get parameter values
        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        topic_prefix = (
            self.get_parameter("topic_prefix").get_parameter_value().string_value
        )

        # Initialize counter and start time
        self.count = 0
        self.start_time = self.get_clock().now()
        
        # Initialize structured logger
        self.structured_logger = StructuredLogger("py_template_node")

        # Create publishers
        self.string_publisher = self.create_publisher(
            String, f"{topic_prefix}/status", 10
        )

        self.counter_publisher = self.create_publisher(
            Int32, f"{topic_prefix}/counter", 10
        )

        self.temperature_publisher = self.create_publisher(
            Temperature, f"{topic_prefix}/temperature", 10
        )

        self.health_publisher = self.create_publisher(
            DiagnosticStatus, f"{topic_prefix}/health", 10
        )

        # Create subscriber
        self.cmd_subscription = self.create_subscription(
            Twist, f"{topic_prefix}/cmd_vel", self.cmd_callback, 10
        )

        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create health timer (30 seconds)
        self.health_timer = self.create_timer(30.0, self.health_callback)

        # Log structured initialization
        init_context = {
            "publish_rate": publish_rate,
            "topic_prefix": topic_prefix
        }
        self.structured_logger.info(
            Component.STARTUP, EventType.INITIALIZATION,
            "Python template node initialized successfully", init_context
        )
        
        self.get_logger().info(
            f"Python template node initialized with rate: {publish_rate:.2f} Hz"
        )
        self.get_logger().info(f"Publishing to topics with prefix: {topic_prefix}")

    def timer_callback(self):
        """Publish data periodically via timer callback."""
        # Log timer event
        timer_context = {"count": self.count}
        self.structured_logger.debug(
            Component.TIMER, EventType.PUBLISH,
            "Timer callback executing", timer_context
        )
        
        # Publish status message
        status_msg = String()
        status_msg.data = f"Python template node running - count: {self.count}"
        self.string_publisher.publish(status_msg)

        # Publish counter
        counter_msg = Int32()
        counter_msg.data = self.count
        self.counter_publisher.publish(counter_msg)

        # Publish simulated temperature data
        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = "base_link"
        temp_msg.temperature = 20.0 + 5.0 * math.sin(
            self.count * 0.1
        )  # Simulated temperature
        temp_msg.variance = 0.1
        self.temperature_publisher.publish(temp_msg)

        if self.count % 10 == 0:
            publish_context = {
                "count": self.count,
                "temperature": temp_msg.temperature,
                "topics": ["status", "counter", "temperature"]
            }
            self.structured_logger.info(
                Component.TIMER, EventType.PUBLISH,
                "Periodic data published", publish_context
            )
            
            self.get_logger().info(
                f"Published count: {self.count}, temp: {temp_msg.temperature:.2f}°C"
            )

        self.count += 1

    def health_callback(self):
        """Publish node health status."""
        uptime = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Log health check
        health_context = {
            "uptime_seconds": int(uptime),
            "message_count": self.count,
            "status": "operational"
        }
        self.structured_logger.info(
            Component.HEALTH, EventType.HEALTH_CHECK,
            "Node health check completed", health_context
        )
        
        health_msg = DiagnosticStatus()
        health_msg.level = DiagnosticStatus.OK
        health_msg.name = self.get_name()
        health_msg.message = "Node operational"
        health_msg.hardware_id = "py_template_node_container"

        uptime_kv = KeyValue()
        uptime_kv.key = "uptime_seconds"
        uptime_kv.value = str(int(uptime))
        health_msg.values.append(uptime_kv)

        count_kv = KeyValue()
        count_kv.key = "message_count"
        count_kv.value = str(self.count)
        health_msg.values.append(count_kv)

        self.health_publisher.publish(health_msg)

    def cmd_callback(self, msg):
        """Process received command velocity messages."""
        # Log structured command reception
        cmd_context = {
            "linear_x": msg.linear.x,
            "linear_y": msg.linear.y,
            "linear_z": msg.linear.z,
            "angular_x": msg.angular.x,
            "angular_y": msg.angular.y,
            "angular_z": msg.angular.z
        }
        self.structured_logger.debug(
            Component.SUBSCRIBER, EventType.RECEIVE,
            "Command velocity received", cmd_context
        )
        
        self.get_logger().info(
            f"Received cmd_vel - linear: [{msg.linear.x:.2f}, "
            f"{msg.linear.y:.2f}, {msg.linear.z:.2f}], "
            f"angular: [{msg.angular.x:.2f}, {msg.angular.y:.2f}, "
            f"{msg.angular.z:.2f}]"
        )

        # Example of processing received command
        if abs(msg.linear.x) > 0.1 or abs(msg.angular.z) > 0.1:
            movement_context = {
                "is_moving": True,
                "linear_x": msg.linear.x,
                "angular_z": msg.angular.z
            }
            self.structured_logger.info(
                Component.SUBSCRIBER, EventType.RECEIVE,
                "Robot movement detected", movement_context
            )
            self.get_logger().info("Robot is moving!")


def main(args=None):
    """Initialize and run the PyTemplateNode."""
    rclpy.init(args=args)

    node = PyTemplateNode()

    node.get_logger().info("Starting Python template node...")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Python template node stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
