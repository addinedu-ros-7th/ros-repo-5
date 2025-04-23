from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'perception_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/commons', glob('commons/*.py')),
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
            'camera_streamer = perception_manager.camera_streamer:main',
            'camera_obstacle_detector = perception_manager.camera_obstacle_detector:main',
            'lidar_obstacle_detector = perception_manager.lidar_obstacle_detector:main'
        ],
    },
)
