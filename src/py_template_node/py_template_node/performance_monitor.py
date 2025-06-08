#!/usr/bin/env python3
"""Performance monitoring for ROS 2 Python nodes."""

import time
from collections import defaultdict, deque
from typing import Dict, Any, Optional
import threading

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node


class PerformanceMonitor:
    """Performance monitoring for ROS 2 Python nodes."""

    def __init__(self, node: Node):
        """Initialize the performance monitor.
        
        Args:
            node: The ROS 2 node to monitor
        """
        self.node = node
        self.start_time = time.time()
        self.last_metrics_time = time.time()
        
        # Message counters
        self.total_messages_published = 0
        self.total_messages_received = 0
        self.topic_publish_counts = defaultdict(int)
        self.topic_receive_counts = defaultdict(int)
        self.last_published_count = 0
        
        # Callback timing
        self.callback_times = deque(maxlen=100)
        self.callback_start_time: Optional[float] = None
        
        # Thread safety
        self.lock = threading.Lock()
        
        # System monitoring
        self.process = psutil.Process()
        
        # ROS integration
        self.metrics_publisher = self.node.create_publisher(
            DiagnosticArray, 'performance_metrics', 10
        )
        
        # Create metrics timer (10 second intervals)
        self.metrics_timer = self.node.create_timer(10.0, self.publish_metrics)

    def record_publish(self, topic: str) -> None:
        """Record a message publication.
        
        Args:
            topic: The topic name where message was published
        """
        with self.lock:
            self.total_messages_published += 1
            self.topic_publish_counts[topic] += 1

    def record_callback_start(self) -> None:
        """Record the start of a callback execution."""
        self.callback_start_time = time.time()

    def record_callback_end(self) -> None:
        """Record the end of a callback execution."""
        if self.callback_start_time is not None:
            duration_ms = (time.time() - self.callback_start_time) * 1000
            with self.lock:
                self.callback_times.append(duration_ms)
            self.callback_start_time = None

    def record_message_received(self, topic: str) -> None:
        """Record a message reception.
        
        Args:
            topic: The topic name where message was received
        """
        with self.lock:
            self.total_messages_received += 1
            self.topic_receive_counts[topic] += 1

    def get_system_metrics(self) -> Dict[str, float]:
        """Get current system metrics.
        
        Returns:
            Dictionary containing CPU, memory, and uptime metrics
        """
        try:
            cpu_percent = self.process.cpu_percent()
            memory_info = self.process.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024  # Convert bytes to MB
            uptime_seconds = time.time() - self.start_time
            
            return {
                'cpu_percent': cpu_percent,
                'memory_mb': memory_mb,
                'uptime_seconds': uptime_seconds
            }
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return {
                'cpu_percent': 0.0,
                'memory_mb': 0.0,
                'uptime_seconds': time.time() - self.start_time
            }

    def get_ros_metrics(self) -> Dict[str, float]:
        """Get ROS-specific performance metrics.
        
        Returns:
            Dictionary containing ROS message and timing metrics
        """
        with self.lock:
            # Calculate publish rate
            current_time = time.time()
            time_diff = current_time - self.last_metrics_time
            
            if time_diff > 0:
                publish_rate = (self.total_messages_published - self.last_published_count) / time_diff
                self.last_metrics_time = current_time
                self.last_published_count = self.total_messages_published
            else:
                publish_rate = 0.0
            
            # Calculate callback timing statistics
            avg_callback_ms = 0.0
            max_callback_ms = 0.0
            if self.callback_times:
                avg_callback_ms = sum(self.callback_times) / len(self.callback_times)
                max_callback_ms = max(self.callback_times)
            
            return {
                'messages_published_total': self.total_messages_published,
                'messages_received_total': self.total_messages_received,
                'publish_rate_hz': publish_rate,
                'avg_callback_ms': avg_callback_ms,
                'max_callback_ms': max_callback_ms
            }

    def get_topic_metrics(self) -> Dict[str, Dict[str, int]]:
        """Get per-topic metrics.
        
        Returns:
            Dictionary containing per-topic message counts
        """
        with self.lock:
            topics = {}
            
            # Combine publish and receive counts
            all_topics = set(self.topic_publish_counts.keys()) | set(self.topic_receive_counts.keys())
            
            for topic in all_topics:
                topics[topic] = {
                    'published_count': self.topic_publish_counts.get(topic, 0),
                    'received_count': self.topic_receive_counts.get(topic, 0)
                }
            
            return topics

    def get_metrics_dict(self) -> Dict[str, Any]:
        """Get all metrics as a structured dictionary.
        
        Returns:
            Complete metrics dictionary for JSON serialization
        """
        return {
            'system': self.get_system_metrics(),
            'ros': self.get_ros_metrics(),
            'topics': self.get_topic_metrics()
        }

    def publish_metrics(self) -> None:
        """Publish performance metrics as ROS diagnostic messages."""
        metrics = self.get_metrics_dict()
        
        # Create diagnostic array message
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.node.get_clock().now().to_msg()
        
        # Create performance status
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = f"{self.node.get_name()}_performance"
        status.message = "Performance metrics"
        status.hardware_id = "performance_monitor"
        
        # Add key metrics as key-value pairs
        def add_kv(key: str, value: Any) -> None:
            kv = KeyValue()
            kv.key = key
            kv.value = str(value)
            status.values.append(kv)
        
        # System metrics
        system_metrics = metrics['system']
        add_kv('cpu_percent', f"{system_metrics['cpu_percent']:.2f}")
        add_kv('memory_mb', f"{system_metrics['memory_mb']:.2f}")
        add_kv('uptime_seconds', f"{system_metrics['uptime_seconds']:.1f}")
        
        # ROS metrics
        ros_metrics = metrics['ros']
        add_kv('messages_published_total', ros_metrics['messages_published_total'])
        add_kv('messages_received_total', ros_metrics['messages_received_total'])
        add_kv('publish_rate_hz', f"{ros_metrics['publish_rate_hz']:.2f}")
        add_kv('avg_callback_ms', f"{ros_metrics['avg_callback_ms']:.2f}")
        add_kv('max_callback_ms', f"{ros_metrics['max_callback_ms']:.2f}")
        
        diagnostic_array.status.append(status)
        self.metrics_publisher.publish(diagnostic_array)
        
        # Log metrics summary periodically
        self.node.get_logger().info(
            f"Performance: CPU {system_metrics['cpu_percent']:.1f}%, "
            f"Memory {system_metrics['memory_mb']:.1f}MB, "
            f"Rate {ros_metrics['publish_rate_hz']:.1f}Hz, "
            f"Avg callback {ros_metrics['avg_callback_ms']:.1f}ms"
        )