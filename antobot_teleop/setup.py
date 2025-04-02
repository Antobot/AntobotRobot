from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'antobot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='So Young Kim',
    maintainer_email='soyoung.kim@antobot.ai',
    description='The antobot_teleop package, which contains the code for robot teleoperation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ # executables
            'joy_remap = antobot_teleop.joy_remap:main',
            'teleop = antobot_teleop.teleop:main'
        ],
    },
    data_files=[
        # Standard ROS 2 package resource files
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install the config directory (including gamepad.yaml)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
)
