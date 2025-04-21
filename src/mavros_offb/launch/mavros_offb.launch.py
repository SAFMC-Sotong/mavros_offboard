#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for offboard control."""
    # Flight Patterns
	# 0: CIRCULAR
	# 1: SPIRAL
	# 2: CLOUD (frequency 7.0 for clear pattern)
	# 3: SINE (speed 0.3, frequency 3.0 for displacement of 1.26)
	# 4: N-GRAM (hepta-gram, vertices 7)

    # Define the offboard control node
    offboard_control_node = Node(
        package='mavros_offb',
        executable='mavros_offb_node',
        name='offboard_control_node',
        output='screen',
        parameters=[{
            'flight_pattern': 0,
            'max_iter': 2,
            'dt': 0.05,
            'radius': 1.00,
            'height': 3.00,
            'speed': 0.50,
            'min_speed': 0.05,
            'offset_x': 0.00,
            'offset_y': 0.00,
            'offset_z': 0.00,
            'frequency': 7.00,
            'ngram_vertices': 7,
            'ngram_step': 2,
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Create and return launch description
    return LaunchDescription([
        offboard_control_node,
    ])