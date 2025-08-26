#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('davo_bridge')
    
    # Launch arguments
    rpc_url_arg = DeclareLaunchArgument(
        'rpc_url',
        default_value='https://sepolia.base.org',
        description='Base Sepolia RPC URL'
    )
    
    private_key_arg = DeclareLaunchArgument(
        'private_key',
        default_value='',
        description='Bridge private key'
    )
    
    robot_private_key_arg = DeclareLaunchArgument(
        'robot_private_key',
        default_value='',
        description='Robot private key'
    )
    
    # Bridge node
    bridge_node = Node(
        package='davo_bridge',
        executable='bridge_node',
        name='davo_bridge_node',
        output='screen',
        parameters=[{
            'rpc_url': LaunchConfiguration('rpc_url'),
            'private_key': LaunchConfiguration('private_key'),
            'robot_private_key': LaunchConfiguration('robot_private_key'),
        }],
        env=[
            ('RPC_URL', LaunchConfiguration('rpc_url')),
            ('PRIVATE_KEY', LaunchConfiguration('private_key')),
            ('ROBOT_PRIVATE_KEY', LaunchConfiguration('robot_private_key')),
        ]
    )
    
    return LaunchDescription([
        rpc_url_arg,
        private_key_arg,
        robot_private_key_arg,
        bridge_node,
    ])
