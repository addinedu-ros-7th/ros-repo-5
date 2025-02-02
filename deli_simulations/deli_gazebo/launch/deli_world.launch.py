import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from ros_gz_sim.actions import GzServer
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package and file names
    pkg_name="deli_gazebo"
    world_file_name = "deli_world.world"

    # Paths
    package_share_path = get_package_share_directory(pkg_name)
    gz_model_path = os.path.join(package_share_path, 'models')
    world_path = os.path.join(package_share_path, 'worlds', world_file_name)
    print(f"package_share_path: {package_share_path}")
    print(f"gz_model_path: {gz_model_path}")
    print(f"world_path: {world_path}")

    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Path to the Gazebo world file"
    )

    gz_resource_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", 
        value=gz_model_path)

    rviz2 = ExecuteProcess(
        cmd=["ros2", "run", "rviz2", "rviz2"],
        output="screen",
    )

    ld = LaunchDescription(
            declare_world_arg,
            gz_resource_path,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                    ('gz_args',
                     [LaunchConfiguration('world'),
                      '.world',
                      ' -v 4',
                      ' -r',
                      ' --physics-engine gz-physics-bullet-featherstone-plugin']
                    )
             ]
    )

    # create and return launch description object
    return ld