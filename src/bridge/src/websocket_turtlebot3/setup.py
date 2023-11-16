from setuptools import setup

package_name = 'websocket_turtlebot3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henriquematias',
    maintainer_email='henrique.matias@sou.inteli.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "websocket_streamer=websocket_turtlebot3.websocket_streamer:main",
        ],
    },
)
