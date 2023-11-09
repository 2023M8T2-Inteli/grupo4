import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elisa',
    maintainer_email='elisa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "chatbot=navigator.chatbot:main",
            "vallet=navigator.vallet:main",
            "init_pose=navigator.initialize_pose:main",
            "queue=navigator.poseQueue:main",
            "coordinatesChatbot=navigator.coordinateChatbot:main"
        ],
    },
)
