from setuptools import find_packages, setup

package_name = 'nav_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugins.xml']),
    ],
    install_requires=['setuptools', 'py_trees', 'rclpy'],
    zip_safe=True,
    maintainer='naon',
    maintainer_email='jonaon611@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'align_with_path = nav_controller.AlignWithPath:main',
        ],
        'nav2_behavior_tree.plugins':[
            'AlignWithPath = nav_controller.AlignWithPath:AlignWithPath'
        ]
    },
)
