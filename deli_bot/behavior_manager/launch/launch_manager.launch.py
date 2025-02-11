import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Robot ID
    robot_id = 'delibot_1'

    # TaskServer Node
    task_server_node = Node(
        package='behavior_manager',
        executable='task_server',
        name=f'{robot_id}_task_server',
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    # BehaviorManager Node
    behavior_manager_node = Node(
        package='behavior_manager',
        executable='behavior_manager',
        name=f'{robot_id}_behavior_manager',
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    # RobotAdapter Node
    robot_adapter_node = Node(
        package='behavior_manager',
        executable='robot_adapter',
        name=f'{robot_id}_adapter',
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    return LaunchDescription([
        task_server_node,
        behavior_manager_node,
        robot_adapter_node
    ])
