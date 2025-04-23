from setuptools import find_packages, setup

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
            'navigation_manager = navigation_manager.navigation_manager:main',
            'motion_planner = navigation_manager.motion_planner:main',
            'task_handler = navigation_manager.task_handler:main',
            'robot_monitor = navigation_manager.robot_monitor:main',
            'start_points_navigator = navigation_manager.start_points_navigator:main'
        ],
    },
)
