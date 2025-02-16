from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'navigation_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_manager.launch.py']),
        ('share/' + package_name + '/commons', glob('commons/*.py')),  # 수정된 부분
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naon',
    maintainer_email='jonaon611@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_streamer = navigation_manager.camera_streamer:main',
            'navigation_manager = navigation_manager.navigation_manager:main',
            'motion_planner_manager = navigation_manager.motion_planner_manager:main',
            'task_handler = navigation_manager.task_handler:main',
            'robot_monitor = navigation_manager.robot_monitor:main',
            'start_points_navigator = navigation_manager.start_points_navigator:main'
        ],
    },
)
