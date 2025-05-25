#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Temperature


class PyTemplateNode(Node):
    """Python version of the template node with publisher, subscriber, and timer functionality."""

    def __init__(self):
        super().__init__('py_template_node')
        
        # Declare parameters with default values
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('topic_prefix', 'py_template')
        
        # Get parameter values
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        topic_prefix = self.get_parameter('topic_prefix').get_parameter_value().string_value
        
        # Initialize counter
        self.count = 0
        
        # Create publishers
        self.string_publisher = self.create_publisher(
            String, f'{topic_prefix}/status', 10)
        
        self.counter_publisher = self.create_publisher(
            Int32, f'{topic_prefix}/counter', 10)
            
        self.temperature_publisher = self.create_publisher(
            Temperature, f'{topic_prefix}/temperature', 10)
        
        # Create subscriber
        self.cmd_subscription = self.create_subscription(
            Twist, f'{topic_prefix}/cmd_vel', self.cmd_callback, 10)
        
        # Create timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Python template node initialized with rate: {publish_rate:.2f} Hz')
        self.get_logger().info(f'Publishing to topics with prefix: {topic_prefix}')

    def timer_callback(self):
        """Timer callback function that publishes data periodically."""
        # Publish status message
        status_msg = String()
        status_msg.data = f'Python template node running - count: {self.count}'
        self.string_publisher.publish(status_msg)
        
        # Publish counter
        counter_msg = Int32()
        counter_msg.data = self.count
        self.counter_publisher.publish(counter_msg)
        
        # Publish simulated temperature data
        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = 'base_link'
        temp_msg.temperature = 20.0 + 5.0 * math.sin(self.count * 0.1)  # Simulated temperature
        temp_msg.variance = 0.1
        self.temperature_publisher.publish(temp_msg)
        
        if self.count % 10 == 0:
            self.get_logger().info(
                f'Published count: {self.count}, temp: {temp_msg.temperature:.2f}°C')
        
        self.count += 1

    def cmd_callback(self, msg):
        """Callback function for command velocity subscriber."""
        self.get_logger().info(
            f'Received cmd_vel - linear: [{msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}], '
            f'angular: [{msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f}]')
        
        # Example of processing received command
        if abs(msg.linear.x) > 0.1 or abs(msg.angular.z) > 0.1:
            self.get_logger().info('Robot is moving!')


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    
    node = PyTemplateNode()
    
    node.get_logger().info('Starting Python template node...')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Python template node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()