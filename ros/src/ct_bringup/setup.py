from setuptools import find_packages, setup
import os
import glob
package_name = 'ct_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        #installing launch files
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        #installing config files
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuele',
    maintainer_email='s.galanti@student.fontys.nl',
    description='Cybetruck the real thing',
    license='No license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         # 'teleop_serial_node = ct_bringup.teleop_serial_node:main', 
          'motion_controller_node = ct_bringup.motion_controller_node:main',  
          'odometry_node = ct_bringup.odometry_node:main',  ],
      
    },
)
