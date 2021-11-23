from launch import LaunchDescription
from launch_ros.actions import Node

import os

def generate_launch_description():

    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity} {time}]: {message}"

    return LaunchDescription([
        Node(
            package='rodel_screwbot',
            namespace='',
            executable='rf_node',
        ),
        Node(
            package='rodel_screwbot',
            namespace='',
            executable='imu_node',
        ),
        Node(
            package='rodel_screwbot',
            namespace='',
            executable='display_node',
        ),
        Node(
            package='rodel_screwbot',
            namespace='',
            executable='maxon_node',
        ),
        Node(
            package='rodel_screwbot',
            namespace='',
            executable='md200t_node',
        ),
    ])