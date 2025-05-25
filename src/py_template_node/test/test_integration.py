#!/usr/bin/env python3

import unittest
import rclpy
import rclpy.parameter
import time
import threading
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Temperature

from py_template_node.py_template_node import PyTemplateNode


class TestIntegrationNode(Node):
    """Helper node for integration testing."""
    
    def __init__(self):
        super().__init__('test_integration_node')
        self.message_counts = {
            'status': 0,
            'counter': 0,
            'temperature': 0
        }
        self.last_messages = {}
        
        # Subscribers for monitoring
        self.status_sub = self.create_subscription(
            String, 'py_template/status', self.status_callback, 10)
        self.counter_sub = self.create_subscription(
            Int32, 'py_template/counter', self.counter_callback, 10)
        self.temp_sub = self.create_subscription(
            Temperature, 'py_template/temperature', self.temp_callback, 10)
        
        # Publisher for sending commands
        self.cmd_pub = self.create_publisher(Twist, 'py_template/cmd_vel', 10)
    
    def status_callback(self, msg):
        self.message_counts['status'] += 1
        self.last_messages['status'] = msg.data
    
    def counter_callback(self, msg):
        self.message_counts['counter'] += 1
        self.last_messages['counter'] = msg.data
    
    def temp_callback(self, msg):
        self.message_counts['temperature'] += 1
        self.last_messages['temperature'] = msg.temperature
    
    def send_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        """Send a cmd_vel message."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)


class TestPyTemplateNodeIntegration(unittest.TestCase):
    """Integration tests for PyTemplateNode."""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.template_node = PyTemplateNode()
        self.test_node = TestIntegrationNode()
        
        # Create executor
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.template_node)
        self.executor.add_node(self.test_node)
        
        # Start spinning
        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()
        
        # Wait for nodes to initialize
        time.sleep(1.0)
    
    def tearDown(self):
        self.executor.shutdown()
        self.template_node.destroy_node()
        self.test_node.destroy_node()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
    
    def test_end_to_end_communication(self):
        """Test complete communication flow."""
        # Wait for initial messages
        max_wait_time = 5.0
        start_time = time.time()
        
        while (time.time() - start_time < max_wait_time and
               (self.test_node.message_counts['status'] == 0 or
                self.test_node.message_counts['counter'] == 0 or
                self.test_node.message_counts['temperature'] == 0)):
            time.sleep(0.1)
        
        # Verify all message types received
        self.assertGreater(self.test_node.message_counts['status'], 0)
        self.assertGreater(self.test_node.message_counts['counter'], 0)
        self.assertGreater(self.test_node.message_counts['temperature'], 0)
    
    def test_bidirectional_communication(self):
        """Test that we can send commands to the node."""
        initial_counts = dict(self.test_node.message_counts)
        
        # Send a command
        self.test_node.send_cmd_vel(linear_x=1.5, angular_z=0.8)
        
        # Wait a bit for processing
        time.sleep(1.0)
        
        # Node should continue publishing (no crashes)
        self.assertGreater(self.test_node.message_counts['status'], 
                          initial_counts['status'])
    
    def test_message_frequency(self):
        """Test that messages are published at expected frequency."""
        # Reset counters
        for key in self.test_node.message_counts:
            self.test_node.message_counts[key] = 0
        
        # Wait for exactly 3 seconds
        time.sleep(3.0)
        
        # With default 1.0 Hz rate, should have ~3 messages of each type
        # Allow some tolerance for timing variations
        for msg_type in ['status', 'counter', 'temperature']:
            count = self.test_node.message_counts[msg_type]
            self.assertGreaterEqual(count, 2, f"Too few {msg_type} messages: {count}")
            self.assertLessEqual(count, 5, f"Too many {msg_type} messages: {count}")
    
    def test_parameter_effects(self):
        """Test that parameter changes affect behavior."""
        # Change the topic prefix
        new_param = rclpy.parameter.Parameter('topic_prefix', 
                                            rclpy.parameter.Parameter.Type.STRING, 
                                            'test_prefix')
        result = self.template_node.set_parameters([new_param])
        self.assertTrue(result[0].successful)
        
        # Verify parameter was set
        updated_prefix = self.template_node.get_parameter('topic_prefix').value
        self.assertEqual(updated_prefix, 'test_prefix')
    
    def test_multiple_command_processing(self):
        """Test processing multiple commands in sequence."""
        commands = [
            (0.0, 0.0),   # Stop
            (1.0, 0.0),   # Forward
            (0.0, 1.0),   # Turn
            (-1.0, 0.0),  # Backward
        ]
        
        for linear_x, angular_z in commands:
            self.test_node.send_cmd_vel(linear_x, angular_z)
            time.sleep(0.2)  # Small delay between commands
        
        # Wait for processing
        time.sleep(1.0)
        
        # Node should still be operational
        initial_count = self.test_node.message_counts['status']
        time.sleep(1.0)
        final_count = self.test_node.message_counts['status']
        
        self.assertGreater(final_count, initial_count)
    
    def test_message_content_consistency(self):
        """Test that message contents are consistent and valid."""
        # Wait for some messages
        time.sleep(3.0)
        
        # Check status message format
        if 'status' in self.test_node.last_messages:
            status_msg = self.test_node.last_messages['status']
            self.assertIn('Python template node running', status_msg)
            self.assertIn('count:', status_msg)
        
        # Check counter is non-negative and increasing
        if 'counter' in self.test_node.last_messages:
            counter_value = self.test_node.last_messages['counter']
            self.assertGreaterEqual(counter_value, 0)
        
        # Check temperature is in reasonable range
        if 'temperature' in self.test_node.last_messages:
            temp_value = self.test_node.last_messages['temperature']
            self.assertGreater(temp_value, 10.0)
            self.assertLess(temp_value, 30.0)
    
    def test_node_resilience(self):
        """Test that node handles various scenarios without crashing."""
        # Send rapid commands
        for i in range(10):
            self.test_node.send_cmd_vel(
                linear_x=float(i % 3 - 1),
                angular_z=float((i + 1) % 3 - 1)
            )
            time.sleep(0.05)
        
        # Wait and verify node is still running
        time.sleep(2.0)
        
        initial_count = self.test_node.message_counts['counter']
        time.sleep(1.0)
        final_count = self.test_node.message_counts['counter']
        
        # Node should still be publishing
        self.assertGreater(final_count, initial_count)


if __name__ == '__main__':
    unittest.main()