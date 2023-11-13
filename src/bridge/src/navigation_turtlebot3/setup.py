import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'navigation_turtlebot3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henrique Marlon',
    maintainer_email='henriquemarloncs@gmail.com',
    description='Autonomous robot on Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pose_queue=navigation_turtlebot3.pose_queue:main",
            "interface=navigation_turtlebot3.interface:main",
            "initial_pose=navigation_turtlebot3.initial_pose:main",
            "vallet=navigation_turtlebot3.vallet:main",
        ],
    },
)
