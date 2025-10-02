import os
from glob import glob

from setuptools import setup

package_name = 'ros2_joy_twist'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # install the YAML
        (os.path.join('share', package_name), ['ros2_joy_twist/mappings.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jordi',
    maintainer_email='test@example.com',
    description='Joystick to Twist (mecanum) with test and robot launch files',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joy_to_twist = ros2_joy_twist.joy_to_twist:main',
        ],
    },
)
