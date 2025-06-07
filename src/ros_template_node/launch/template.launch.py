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
        default_value="2.0",
        description="Publishing rate in Hz for the template node",
    )

    topic_prefix_arg = DeclareLaunchArgument(
        "topic_prefix",
        default_value="template",
        description="Prefix for all topics published by the template node",
    )


    # Template node
    template_node = Node(
        package="ros_template_node",
        executable="template_node",
        name="template_node",
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
            template_node,
        ]
    )
