#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="WebSocket port for Foxglove bridge",
    )

    # Foxglove bridge node
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[
            {
                "port": LaunchConfiguration("foxglove_port"),
                "address": "0.0.0.0",
                "tls": False,
                "certfile": "",
                "keyfile": "",
                "topic_whitelist": [".*"],
                "send_buffer_limit": 10000000,
                "use_compression": False,
                "capabilities": [
                    "clientPublish",
                    "parameters",
                    "parametersSubscribe",
                    "services",
                    "connectionGraph",
                ],
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            foxglove_port_arg,
            foxglove_bridge,
        ]
    )
