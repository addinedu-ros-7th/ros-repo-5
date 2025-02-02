import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'deli_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        package_name: [
            'models/deli_world/**/*',
            'launch/*.launch.py',
            'worlds/*',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naon',
    maintainer_email='jonaon611@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
