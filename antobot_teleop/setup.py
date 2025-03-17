from setuptools import find_packages, setup

package_name = 'antobot_teleop'

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
    maintainer='So Young Kim',
    maintainer_email='soyoung.kim@antobot.ai',
    description='The antobot_teleop package, which contains the code for robot teleoperation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = antobot_teleop.teleop:main'
        ],
    },
)
