import os
from launch import LaunchDescription
from launch_ros.actions import Node

"""
Test Command:
$ ros2 action send_goal /delibot_1/dispatch_delivery_task task_manager_msgs/action/DispatchDeliveryTask "{pickups: [{station: '일반', handler: 'delibot_1', payload: [{sku: 'apple', quantity: 5}]}]}" --feedback

"""

def generate_launch_description():
    # Robot ID
    robot_id = 'delibot_1'

    # camera_streamer node
    camera_streamer_node = Node(
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

    # motion_planner_manager node
    motion_planner_manager_node = Node(
        package='navigation_manager',
        executable='motion_planner_manager',
        name=f'{robot_id}_motion_planner_manager',
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
        camera_streamer_node,
        # motion_planner_manager_node,
        navigation_manager_node,
        robot_monitor_node,
        task_handler_node,
    ])
