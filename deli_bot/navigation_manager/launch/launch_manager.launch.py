import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Robot ID
    robot_id = 'delibot_1'

    # camera_streamer node
    camera_stareamer_node = Node(
        package='navigation_manager',
        executable='camera_streamer',
        name=f'{robot_id}_camera_streamer',
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    # task_handler node
    task_handler_node = Node(
        package='navigation_manager',
        executable='task_handler',
        name=f'{robot_id}_task_handler',
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    # navigation_manager node
    navigation_manager_node = Node(
        package='navigation_manager',
        executable='navigation_manager',
        name=f'{robot_id}_navigation_manager',
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    # robot_monitor node
    robot_monitor_node = Node(
        package='navigation_manager',
        executable='robot_monitor',
        name=f'{robot_id}_monitor',
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    return LaunchDescription([
        task_handler_node,
        navigation_manager_node,
        robot_monitor_node
    ])
