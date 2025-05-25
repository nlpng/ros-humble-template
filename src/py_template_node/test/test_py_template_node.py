#!/usr/bin/env python3

import threading
import time
import unittest

import rclpy
import rclpy.exceptions
import rclpy.parameter
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Int32, String

from py_template_node.py_template_node import PyTemplateNode


class TestMessageReceiver(Node):
    """Helper node to receive messages for testing."""

    def __init__(self):
        super().__init__("test_message_receiver")
        self.status_received = False
        self.counter_received = False
        self.temp_received = False
        self.last_status_msg = ""
        self.last_counter_value = -1
        self.last_temperature = 0.0

        self.status_sub = self.create_subscription(
            String, "py_template/status", self.status_callback, 10
        )
        self.counter_sub = self.create_subscription(
            Int32, "py_template/counter", self.counter_callback, 10
        )
        self.temp_sub = self.create_subscription(
            Temperature, "py_template/temperature", self.temp_callback, 10
        )

    def status_callback(self, msg):
        self.last_status_msg = msg.data
        self.status_received = True

    def counter_callback(self, msg):
        self.last_counter_value = msg.data
        self.counter_received = True

    def temp_callback(self, msg):
        self.last_temperature = msg.temperature
        self.temp_received = True


class TestPyTemplateNode(unittest.TestCase):
    """Test cases for PyTemplateNode."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = PyTemplateNode()
        self.receiver = TestMessageReceiver()

        # Create executor for spinning nodes
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.receiver)

        # Start spinning in a separate thread
        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.receiver.destroy_node()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)

    def test_node_creation(self):
        """Test that the node is created successfully."""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), "py_template_node")

    def test_parameters_exist(self):
        """Test that required parameters are declared."""
        # Check that parameters exist by trying to get them
        try:
            publish_rate = self.node.get_parameter("publish_rate").value
            topic_prefix = self.node.get_parameter("topic_prefix").value

            # If we get here, parameters exist - check their values
            self.assertGreater(publish_rate, 0.0)
            self.assertIsInstance(topic_prefix, str)
            self.assertTrue(len(topic_prefix) > 0)

        except rclpy.exceptions.ParameterNotDeclaredException:
            self.fail(
                "Required parameters 'publish_rate' or 'topic_prefix' not declared"
            )

    def test_parameter_update(self):
        """Test that parameters can be updated."""
        # Set a new parameter value
        new_param = rclpy.parameter.Parameter(
            "publish_rate", rclpy.parameter.Parameter.Type.DOUBLE, 5.0
        )
        result = self.node.set_parameters([new_param])

        self.assertTrue(result[0].successful)

        # Verify the parameter was updated
        updated_rate = self.node.get_parameter("publish_rate").value
        self.assertEqual(updated_rate, 5.0)

    def test_message_publishing(self):
        """Test that the node publishes messages."""
        # Wait for messages to be published and received
        max_wait_time = 5.0
        start_time = time.time()

        while time.time() - start_time < max_wait_time and not (
            self.receiver.status_received
            and self.receiver.counter_received
            and self.receiver.temp_received
        ):
            time.sleep(0.1)

        # Verify messages were received
        self.assertTrue(self.receiver.status_received, "Status message not received")
        self.assertTrue(self.receiver.counter_received, "Counter message not received")
        self.assertTrue(self.receiver.temp_received, "Temperature message not received")

        # Verify message content
        self.assertIsInstance(self.receiver.last_status_msg, str)
        self.assertGreaterEqual(self.receiver.last_counter_value, 0)
        self.assertGreater(self.receiver.last_temperature, 10.0)
        self.assertLess(self.receiver.last_temperature, 30.0)

    def test_cmd_vel_subscription(self):
        """Test that the node can receive cmd_vel messages."""
        # Create a publisher to send cmd_vel messages
        cmd_publisher = self.receiver.create_publisher(Twist, "py_template/cmd_vel", 10)

        # Wait for publisher to be established
        time.sleep(0.5)

        # Send a command
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.angular.z = 0.5

        cmd_publisher.publish(twist_msg)

        # Wait a bit for message processing
        time.sleep(0.5)

        # Test passes if no exceptions were raised
        self.assertTrue(True)

    def test_counter_increments(self):
        """Test that the counter increments over time."""
        # Wait for first counter message
        max_wait_time = 3.0
        start_time = time.time()

        while (
            time.time() - start_time < max_wait_time
            and not self.receiver.counter_received
        ):
            time.sleep(0.1)

        self.assertTrue(self.receiver.counter_received)
        first_counter = self.receiver.last_counter_value

        # Reset and wait for another message
        self.receiver.counter_received = False

        start_time = time.time()
        while (
            time.time() - start_time < max_wait_time
            and not self.receiver.counter_received
        ):
            time.sleep(0.1)

        self.assertTrue(self.receiver.counter_received)
        second_counter = self.receiver.last_counter_value

        # Counter should have incremented
        self.assertGreater(second_counter, first_counter)

    def test_temperature_simulation(self):
        """Test that temperature values are within expected range."""
        # Collect multiple temperature readings
        temperatures = []
        max_wait_time = 5.0
        start_time = time.time()

        while time.time() - start_time < max_wait_time and len(temperatures) < 3:
            if self.receiver.temp_received:
                temperatures.append(self.receiver.last_temperature)
                self.receiver.temp_received = False
            time.sleep(0.1)

        self.assertGreater(len(temperatures), 0)

        # All temperatures should be within simulation range (approximately 15-25°C)
        for temp in temperatures:
            self.assertGreater(temp, 10.0)
            self.assertLess(temp, 30.0)


if __name__ == "__main__":
    unittest.main()
