from setuptools import find_packages, setup

package_name = 'behavior_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'behavior_manager = behavior_manager.BehaviorManager:main',
            'nav_client = behavior_manager.NavClient:main',
            'task_server = behavior_manager.TaskServer:main',
            'traffic_client = behavior_manager.TrafficClient:main',
            'goal_listener = behavior_manager.GoalListener:main'
        ],
    },
)
