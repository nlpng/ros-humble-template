#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in Hz for the Python template node'
    )
    
    topic_prefix_arg = DeclareLaunchArgument(
        'topic_prefix',
        default_value='py_template',
        description='Prefix for all topics published by the Python template node'
    )
    
    enable_foxglove_arg = DeclareLaunchArgument(
        'enable_foxglove',
        default_value='false',
        description='Enable Foxglove bridge for visualization (shared with C++ node)'
    )
    
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='WebSocket port for Foxglove bridge'
    )
    
    # Python template node
    py_template_node = Node(
        package='py_template_node',
        executable='py_template_node',
        name='py_template_node',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'topic_prefix': LaunchConfiguration('topic_prefix'),
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Foxglove bridge node (optional, usually run separately)
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge_py',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'address': '0.0.0.0',
            'tls': False,
            'certfile': '',
            'keyfile': '',
            'topic_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_compression': False,
            'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 'services', 'connectionGraph'],
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_foxglove'))
    )

    return LaunchDescription([
        publish_rate_arg,
        topic_prefix_arg,
        enable_foxglove_arg,
        foxglove_port_arg,
        py_template_node,
        foxglove_bridge,
    ])