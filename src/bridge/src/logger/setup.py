import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'assets'), glob('assets/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henriquemarlon',
    maintainer_email='henriquemarloncs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "logger=logger.main:main",
        ],
    },
)
