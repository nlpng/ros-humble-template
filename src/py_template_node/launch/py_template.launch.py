#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="1.0",
        description="Publishing rate in Hz for the Python template node",
    )

    topic_prefix_arg = DeclareLaunchArgument(
        "topic_prefix",
        default_value="py_template",
        description="Prefix for all topics published by the Python template node",
    )


    # Python template node
    py_template_node = Node(
        package="py_template_node",
        executable="py_template_node",
        name="py_template_node",
        parameters=[
            {
                "publish_rate": LaunchConfiguration("publish_rate"),
                "topic_prefix": LaunchConfiguration("topic_prefix"),
            }
        ],
        output="screen",
        emulate_tty=True,
    )


    return LaunchDescription(
        [
            publish_rate_arg,
            topic_prefix_arg,
            py_template_node,
        ]
    )
