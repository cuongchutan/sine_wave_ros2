#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sine_wave',
            executable='sine_wave_publisher.py',
            name='sine_wave_publisher',
        ),
        Node(
            package='sine_wave',
            executable='sine_wave_subscriber.py',
            name='sine_wave_subscriber'
        ),
    ])
